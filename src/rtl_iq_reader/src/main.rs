// Cargo.toml
// [dependencies]
// rtlsdr = "0.1.4"

use rtlsdr::{get_device_count, open, RTLSDRError};
use std::time::{SystemTime, Instant};
use std::fs::File;
use std::io::Write;
use std::io::Read;
use std::env;
use std::error::Error;

/// --- Tunables -------------------------------------------------------------

// Sample rate in Hz (match how you captured the .cu8 file)
// Common for RTL-SDR ADS-B captures: 2_400_000
const SAMPLE_RATE_HZ: f32 = 2_400_000.0;

// Number of bits to extract after preamble: 112 (long) or 56 (short)
// You can try 112 first; if you want to also try 56, call extract twice.
const MSG_BITS: usize = 112;

// Energy thresholds for preamble detection; tweak if you get misses
const PREAMBLE_HIGH: f32 = 0.55; // fraction of local peak considered "high"
const PREAMBLE_LOW: f32 = 0.30;  // fraction considered "low" in gap positions

// Oversampling windowing factors when sampling bit halves (in microseconds)
const HALF_US: f32 = 0.5; // first/second half of each 1 µs bit

/// --- Structs --------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct AdsbMessage {
    pub df: u8,
    pub ca: u8,
    pub icao: String,
    pub tc: Option<u8>,
    pub payload: Option<String>,
}

/// --- Helpers --------------------------------------------------------------

fn generate_capture(path: &str, chunks: usize) -> Result<Vec<u8>, RTLSDRError> {
    let count = get_device_count();
    if count == 0 {
        eprintln!("No RTL-SDR devices found");
        return Ok(Vec::new());
    }

    let mut dev = open(0)?;

    // Optional: get and display USB strings
    match dev.get_usb_strings() {
        Ok(info) => {
            println!("Device info:\n  Manufacturer: {}\n  Product: {}\n  Serial: {}",
                    info.manufacturer, info.product, info.serial);
        }
        Err(e) => eprintln!("Failed to get USB strings: {:?}", e),
    }

    dev.set_center_freq(1_090_000_000)?;
    dev.set_sample_rate(2_400_000)?;
    dev.set_tuner_gain_mode(true)?;
    let gains = dev.get_tuner_gains()?;
    if let Some(&g) = gains.iter().max() { dev.set_tuner_gain(g)?; }
    dev.reset_buffer()?;

    // Open output file
    let mut file = File::create(path)
        .expect("failed to create output file");

    let start = Instant::now(); // Capture the starting time

    // Capture N chunks and write them
    let block_size = 16 * 16384; // samples per block
    let mut all_data = Vec::with_capacity(chunks * block_size);
    for n in 0..chunks {
        let buf = dev.read_sync(block_size)?;
        file.write_all(&buf).expect("write failed");
        all_data.extend_from_slice(&buf);
        println!("Captured block {} ({} bytes)", n + 1, buf.len());
    }
    let duration = start.elapsed();

    println!("Capture complete {:?} time elapsed: {:?}", path, duration);
    Ok(all_data)
}

#[inline]
fn u8iq_to_power(i: u8, q: u8) -> f32 {
    // Convert unsigned [0..255] to signed centered at ~0, then power = I^2 + Q^2
    let ii = (i as i16 - 127) as f32;
    let qq = (q as i16 - 127) as f32;
    ii * ii + qq * qq
}

fn load_cu8(path: &str) -> Vec<u8> {
    let mut f = File::open(path).expect("failed to open .cu8 file");
    let mut buf = Vec::new();
    f.read_to_end(&mut buf).expect("failed to read file");
    if buf.len() % 2 != 0 {
        eprintln!("Warning: input length is odd; last byte will be ignored");
        buf.pop();
    }
    buf
}

fn iq_power_series(cu8: &[u8]) -> Vec<f32> {
    let n_pairs = cu8.len() / 2;
    let mut p = Vec::with_capacity(n_pairs);
    for k in 0..n_pairs {
        let i = cu8[2 * k];
        let q = cu8[2 * k + 1];
        p.push(u8iq_to_power(i, q));
    }
    // Normalize to ~[0..1]
    if let Some(maxv) = p.iter().copied().max_by(|a, b| a.partial_cmp(b).unwrap()) {
        if maxv > 0.0 {
            for v in &mut p {
                *v /= maxv;
            }
        }
    }
    p
}

/// Get sum (energy) over a fractional window [start_us, end_us) starting at `base_idx` in samples.
/// Uses simple rectangular integration on the power series.
fn window_energy(power: &[f32], base_idx: usize, start_us: f32, end_us: f32, s_per_us: f32) -> f32 {
    let start = (base_idx as f32 + start_us * s_per_us).round() as isize;
    let end = (base_idx as f32 + end_us * s_per_us).round() as isize;
    let lo = start.max(0) as usize;
    let hi = end.max(lo as isize) as usize;
    if hi <= lo || hi > power.len() {
        return 0.0;
    }
    power[lo..hi].iter().sum()
}

/// Check for Mode-S preamble at index `i0` (start of preamble).
/// Preamble pulses at 0, 0.5, 1.0, 3.5 µs must be "high";
/// gaps near 2.0, 2.5, 3.0 µs must be "low".
fn is_preamble(power: &[f32], i0: usize, s_per_us: f32) -> bool {
    // Measure a local reference peak over the first ~4.5 us to get a relative scale.
    let ref_energy = window_energy(power, i0, 0.0, 4.5, s_per_us);
    if ref_energy == 0.0 {
        return false;
    }
    // Short pulse windows around expected pulse centers (0.2 us wide)
    // and gap windows (also ~0.2 us) to test low energy.
    let pulse = |us: f32| window_energy(power, i0, us - 0.1, us + 0.1, s_per_us);
    let gap = |us: f32| window_energy(power, i0, us - 0.1, us + 0.1, s_per_us);

    let p0 = pulse(0.0);
    let p1 = pulse(0.5);
    let p2 = pulse(1.0);
    let p3 = pulse(3.5);

    let g0 = gap(2.0);
    let g1 = gap(2.5);
    let g2 = gap(3.0);

    // Normalize by max pulse to be scale-invariant
    let pmax = p0.max(p1).max(p2).max(p3).max(1e-6);

    let high_ok = (p0 / pmax) > PREAMBLE_HIGH
        && (p1 / pmax) > PREAMBLE_HIGH
        && (p2 / pmax) > PREAMBLE_HIGH
        && (p3 / pmax) > PREAMBLE_HIGH;

    // Gaps should be meaningfully lower than peaks
    let low_ok = (g0 / pmax) < PREAMBLE_LOW && (g1 / pmax) < PREAMBLE_LOW && (g2 / pmax) < PREAMBLE_LOW;

    high_ok && low_ok
}

/// After a preamble starting at i0, extract MSG_BITS bits starting 8 µs later.
/// For each bit window (1 µs), compare energy in first half vs second half.
fn extract_bits(power: &[f32], i0: usize, s_per_us: f32, n_bits: usize) -> Option<Vec<u8>> {
    let start_bits = i0 + (8.0 * s_per_us) as usize; // message begins 8 µs after preamble start
    let mut bits = Vec::with_capacity(n_bits);
    for b in 0..n_bits {
        let bit_start = start_bits as f32 + (b as f32) * s_per_us; // start of this 1 µs bit (in samples)
        // First half [0..0.5us), second half [0.5..1.0us)
        let e_first = window_energy(power, bit_start as usize, 0.0, HALF_US, s_per_us);
        let e_second = window_energy(power, bit_start as usize, HALF_US, 1.0, s_per_us);
        if e_first == 0.0 && e_second == 0.0 {
            return None; // ran off the end
        }
        let bit = if e_first > e_second { 1u8 } else { 0u8 };
        bits.push(bit);
    }
    Some(bits)
}

fn bits_to_bytes_hex(bits: &[u8]) -> String {
    let mut out = String::new();
    for chunk in bits.chunks(8) {
        let mut byte: u8 = 0;
        for (i, b) in chunk.iter().enumerate() {
            // Mode S is big-endian per byte (MSB first)
            byte |= (*b & 1) << (7 - i);
        }
        out.push_str(&format!("{:02X}", byte));
    }
    out
}

// Basic Mode S field decoding for all possible FR (Format) and FRN (Format Number) values.
// This is a minimal decoder for demonstration; for production, use a full Mode S library.
fn decode_mode_s(bytes: &[u8]) -> Result<Option<AdsbMessage>, Box<dyn Error>> {
    if bytes.len() < 14 {
        return Err(format!("Frame too short for Mode S decode: got {} bytes", bytes.len()).into());
    }
    let df = bytes[0] >> 3; // Downlink Format (DF)
    let ca = bytes[0] & 0x07; // Capability (CA)
    let icao = format!("{:02X}{:02X}{:02X}", bytes[1], bytes[2], bytes[3]); // ICAO address (3 bytes)

    let mut tc = None;
    let mut payload_vec = Vec::new();

    match df {
        0 => {
            // Short Air-Air Surveillance
            payload_vec.push("Short Air-Air Surveillance".to_string());
        }
        4 => {
            // Surveillance, Altitude Reply
            let alt = format!("{:02X}{:02X}", bytes[4], bytes[5]);
            payload_vec.push(format!("Altitude Reply: {}", alt));
        }
        5 | 21 => {
            // Surveillance, Identity Reply or All-call Reply
            let id = format!("{:02X}{:02X}", bytes[4], bytes[5]);
            payload_vec.push(format!("Identity Reply: {}", id));
        }
        11 => {
            // All-call Reply (no identity)
            payload_vec.push("All-call Reply (no identity)".to_string());
        }
        16 => {
            // Long Air-Air Surveillance
            payload_vec.push("Long Air-Air Surveillance".to_string());
        }
        17 | 18 => {
            // Extended squitter (ADS-B)
            tc = Some(bytes[4] >> 3);
            match tc.unwrap() {
                1..=4 => {
                    let chars = &bytes[5..11];
                    let chars_hex: String = chars.iter().map(|b| format!("{:02X}", b)).collect();
                    payload_vec.push(format!("Aircraft identification: {}", chars_hex));
                }
                5..=8 | 9..=18 => {
                    let pos = &bytes[5..11];
                    let pos_hex: String = pos.iter().map(|b| format!("{:02X}", b)).collect();
                    payload_vec.push(format!("Position message: {}", pos_hex));
                }
                19 => {
                    let vel = &bytes[5..11];
                    let vel_hex: String = vel.iter().map(|b| format!("{:02X}", b)).collect();
                    payload_vec.push(format!("Airborne velocity: {}", vel_hex));
                }
                _ => {
                    payload_vec.push("Other ADS-B message".to_string());
                }
            }
        }
        20 => {
            // Comm-B, altitude reply
            payload_vec.push("Comm-B, altitude reply".to_string());
        }
        24 => {
            // Comm-D (ELM)
            payload_vec.push("Comm-D (ELM)".to_string());
        }
        _ => {
            payload_vec.push(format!("Unknown or unsupported DF: {}", df));
        }
    }
    let payload = if payload_vec.is_empty() { None } else { Some(payload_vec.join("; ")) };

    Ok(Some(AdsbMessage {
        df,
        ca,
        icao,
        tc,
        payload,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_u8iq_to_power_centered() {
        // Centered values should yield zero power
        assert_eq!(u8iq_to_power(127, 127), 0.0);
    }

    #[test]
    fn test_u8iq_to_power_max() {
        // Max values should yield positive power
        let p = u8iq_to_power(255, 255);
        assert!(p > 0.0);
    }

    #[test]
    fn test_bits_to_bytes_hex() {
        // 8 bits, MSB first, should yield one byte in hex
        let bits = vec![1,0,1,0,1,0,1,0]; // 0b10101010 = 0xAA
        assert_eq!(bits_to_bytes_hex(&bits), "AA");
    }

    #[test]
    fn test_bits_to_bytes_hex_multiple_bytes() {
        let bits = vec![
            1,1,1,1,0,0,0,0, // 0xF0
            0,1,0,1,0,1,0,1  // 0x55
        ];
        assert_eq!(bits_to_bytes_hex(&bits), "F055");
    }

    #[test]
    fn test_window_energy_basic() {
        let power = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let s_per_us = 1.0;
        // window from index 1 to 3 (exclusive)
        let e = window_energy(&power, 0, 1.0, 3.0, s_per_us);
        assert_eq!(e, 2.0 + 3.0);
    }

    #[test]
    fn test_iq_power_series_normalization() {
        let cu8 = vec![255, 255, 0, 0, 127, 127, 255, 0];
        let p = iq_power_series(&cu8);
        let max = p.iter().cloned().fold(f32::MIN, f32::max);
        assert!((max - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_extract_bits_simple() {
        // Simulate a power series with clear first-half/second-half difference
        let mut power = vec![0.0; 100];
        let s_per_us = 2.0; // 2 samples per us
        let i0 = 0;
        // Preamble: fill with high values to pass is_preamble
        for i in 0..10 { power[i] = 1.0; }
        // Message: alternate high/low in first/second half of each bit
        let n_bits = 8;
        let start_bits = i0 + (8.0 * s_per_us) as usize;
        for b in 0..n_bits {
            let bit_start = start_bits + (b as usize * s_per_us as usize);
            // First half high, second half low
            power[bit_start] = 2.0;
            power[bit_start + 1] = 0.1;
        }
        let bits = extract_bits(&power, i0, s_per_us, n_bits).unwrap();
        assert_eq!(bits, vec![1; n_bits]);
    }

    #[test]
    fn test_is_preamble_false_on_zero() {
        let power = vec![0.0; 50];
        let s_per_us = 2.0;
        assert!(!is_preamble(&power, 0, s_per_us));
    }

    #[test]
    fn test_decode_mode_s_short_frame() {
        let bytes = vec![0x00, 0x01, 0x02];
        let result = decode_mode_s(&bytes);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("Frame too short"));
    }

    #[test]
    fn test_decode_mode_s_df5() {
        // DF 5 (0x28), which is 0b00101000, so DF = 5, CA = 0
        let bytes = vec![0x28, 0x01, 0x02, 0x03, 0x04, 0x05, 0,0,0,0,0,0,0,0];
        let result = decode_mode_s(&bytes).unwrap();
        let msg = result.unwrap();
        assert_eq!(msg.df, 5); // 0x28 >> 3 == 5
        assert_eq!(msg.icao, "010203");
        assert!(msg.payload.as_ref().unwrap().contains("Identity Reply"));
    }

    #[test]
    fn test_decode_mode_s_df17_adsb_ident() {
        // DF 17, TC 4 (identification)
        let bytes = vec![0x88, 0xAA, 0xBB, 0xCC, 0x20, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0,0,0];
        let result = decode_mode_s(&bytes).unwrap();
        let msg = result.unwrap();
        assert_eq!(msg.df, 17);
        assert_eq!(msg.icao, "AABBCC");
        assert_eq!(msg.tc, Some(4));
        assert!(msg.payload.as_ref().unwrap().contains("Aircraft identification"));
    }

    #[test]
    fn test_decode_mode_s_df4_altitude() {
        // DF 4, Altitude Reply
        let bytes = vec![0x20, 0x01, 0x02, 0x03, 0x12, 0x34, 0,0,0,0,0,0,0,0];
        let result = decode_mode_s(&bytes).unwrap();
        let msg = result.unwrap();
        assert_eq!(msg.df, 4);
        assert!(msg.payload.as_ref().unwrap().contains("Altitude Reply"));
    }

    #[test]
    fn test_decode_mode_s_df0_short_air_air() {
        // DF 0, Short Air-Air Surveillance
        let bytes = vec![0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0,0,0,0,0,0,0,0];
        let result: Option<AdsbMessage> = decode_mode_s(&bytes).unwrap();
        let msg = result.unwrap();
        assert_eq!(msg.df, 0);
        assert!(msg.payload.as_ref().unwrap().contains("Short Air-Air Surveillance"));
    }
}

/// --- Main ----------------------------------------------------------------

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() > 1 {
        eprintln!("Usage: {} [filename]", args[0]);
        std::process::exit(1);
    }
    let cu8 : Vec<u8>;
    if args.len() == 1 {
        // If a filename is provided, load it instead of capturing
        cu8 = load_cu8(&args[1]);
    } else {
        // Otherwise, capture from RTL-SDR
        let capture_path: String = format!("capture.{:?}.cu8", SystemTime::now());
        cu8 = generate_capture(&capture_path, 100).expect("Capture failed");
    }
    if cu8.is_empty() {
        eprintln!("No data captured; exiting");
        return;
    }
    let power = iq_power_series(&cu8);
    let s_per_us = SAMPLE_RATE_HZ / 1_000_000.0;

    let mut found = 0usize;
    let mut i = 0usize;
    while (i + (12.0 * s_per_us) as usize + (MSG_BITS as f32 * s_per_us) as usize) < power.len() {
        if is_preamble(&power, i, s_per_us) {
            if let Some(bits) = extract_bits(&power, i, s_per_us, MSG_BITS) {
                let hex = bits_to_bytes_hex(&bits);
                println!("MSG @sample {}: {}", i, hex);
                let result = decode_mode_s(&bits);
                println!("Decoded: {:?}", result);
                found += 1;
                // Skip ahead by the whole frame to avoid re-detecting inside it
                i += ((8.0 + MSG_BITS as f32) * s_per_us) as usize;
            }
        }
        i += 1;
    }

    eprintln!("Found {} candidate {}-bit frames", found, MSG_BITS);
}

