/// Equivalent to rtl_test — detect and probe RTL-SDR devices.
use rs_rtl::{DeviceDescriptors, DeviceId, RtlSdr};

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| "info".into()),
        )
        .init();

    println!("rs-rtl device test\n");

    // List devices
    match DeviceDescriptors::new() {
        Ok(devices) => {
            println!("Found {} RTL-SDR device(s):\n", devices.len());
            for (i, dev) in devices.iter().enumerate() {
                println!(
                    "  #{i}: {vendor:04x}:{product:04x} bus={bus} addr={addr}",
                    vendor = dev.vendor_id,
                    product = dev.product_id,
                    bus = dev.bus,
                    addr = dev.address,
                );
                if let Some(ref mfr) = dev.manufacturer {
                    println!("       Manufacturer: {mfr}");
                }
                if let Some(ref prod) = dev.product {
                    println!("       Product:      {prod}");
                }
                if let Some(ref serial) = dev.serial {
                    println!("       Serial:       {serial}");
                }
                println!();
            }
        }
        Err(e) => {
            eprintln!("Failed to list devices: {e}");
            std::process::exit(1);
        }
    }

    // Try to open device 0
    println!("Opening device #0...");
    match RtlSdr::open(DeviceId::Index(0)) {
        Ok(sdr) => {
            println!("  Tuner:       {:?}", sdr.tuner_type());
            println!(
                "  Gains (dB):  {:?}",
                sdr.gains()
                    .iter()
                    .map(|g| *g as f32 / 10.0)
                    .collect::<Vec<_>>()
            );
            println!("\nDevice opened and initialized successfully.");
        }
        Err(e) => {
            eprintln!("Failed to open device: {e}");
            std::process::exit(1);
        }
    }
}
