# RTL IQ READER

So, the IQ > ADS-B byte pipeline is:
IQ samples > magnitude > preamble detection > pulse timing > bits > bytes > CRC check.

For education purposes, simple and readable

This self-contained Rust program that writes a .cu8 file (interleaved unsigned 8-bit I/Q), computes amplitude, detects the ADS-B preamble, and then extracts 112-bit Mode-S messages (long frames) by sampling each 1 µs bit using first/second-half energy (PPM). It prints each decoded frame as hex bytes.

[TOC]

## What ADS-B looks like

- Frequency: 1090 MHz
- Modulation: Pulse Position Modulation (PPM)
- Symbol rate: 1 Mbps (each bit = 1 µs)
- Messages: 56 or 112 bits (short or long)
- Preamble: fixed pattern of 8 pulses, 8 µs long

So, the task is to turn amplitude blips in your IQ into a sequence of 1s and 0s.

## Steps to decode

### Get the baseband amplitude

Compute magnitude of each IQ pair: $mag = \sqrt{I^2 + Q^2}$

- This reduces the IQ stream to a real-valued “power vs. time” signal.
- ADS-B pulses show up as spikes.

### Detect the preamble

Look for the known 8-µs preamble:

- Pulses at 0, 0.5, 1.0, 3.5 µs
- Gaps elsewhere

This syncs you to the start of a frame.

### Sample the bits

After preamble, each bit is 1 µs long:

- A pulse in the first half (0–0.5 µs) = bit 1
- A pulse in the second half (0.5–1 µs) = bit 0

Collect 56 or 112 of these to get the raw ADS-B frame bits.

### Pack bits into bytes

- Group 8 bits → 1 byte.
- For example, the 112-bit Mode-S long message = 14 bytes.

### CRC check & message decode

- Validate with the ADS-B CRC polynomial.
- Extract DF (downlink format), ICAO hex address, payload, etc.
