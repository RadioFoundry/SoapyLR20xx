// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
//! Lists the bridges that answer discovery: `cargo run --example discover [address]`.

use std::time::Duration;

fn main() -> std::io::Result<()> {
    let wait = Duration::from_millis(500);
    let found = match std::env::args().nth(1) {
        Some(a) => lr20xx_link::discover_at(a.parse().expect("an IPv4 address"), wait)?,
        None => lr20xx_link::discover(wait)?,
    };
    for (addr, h) in &found {
        println!(
            "{} {} board {} ROM {}.{} firmware {}",
            addr.ip(),
            h.serial,
            h.board,
            h.rom_major,
            h.rom_minor,
            h.firmware
        );
    }
    if found.is_empty() {
        println!("none answered");
    }
    Ok(())
}
