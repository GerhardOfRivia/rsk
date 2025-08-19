// Cargo.toml
// [dependencies]
// rtlsdr = "0.1.4"

use rtlsdr::{get_device_count, open, RTLSDRError};
use std::fs::File;
use std::io::Write;

fn main() -> Result<(), RTLSDRError> {
    let count = get_device_count();
    if count == 0 {
        eprintln!("No RTL-SDR devices found");
        return Ok(());
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
    let mut file = File::create("capture.cu8")
        .expect("failed to create output file");

    // Capture N chunks and write them
    let chunks = 50;          // how many blocks to capture
    let block_size = 16 * 16384; // samples per block
    for n in 0..chunks {
        let buf = dev.read_sync(block_size)?;
        file.write_all(&buf).expect("write failed");
        println!("Captured block {} ({} bytes)", n + 1, buf.len());
    }

    println!("Capture complete -> capture.cu8");
    Ok(())
}
