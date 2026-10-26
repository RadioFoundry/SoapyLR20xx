// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
//! Builds liblr20xx (lib/) and the link encoding (proto/) with the POSIX port, and on Linux the spidev / GPIO HAL.
//! The ESP-IDF files (port/esp_idf.c, hal/esp_idf.c) are left out. lib and proto are links to the repository's
//! folders; `cargo package` copies what they point to.

use std::path::PathBuf;

fn main() {
    let root = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let lib = root.join("lib");
    let proto = root.join("proto");
    let linux = std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("linux");

    let mut b = cc::Build::new();
    b.std("c11").include(lib.join("include")).include(proto.join("include")).warnings(false);
    let mut srcs: Vec<PathBuf> = std::fs::read_dir(lib.join("src"))
        .expect("lib/src")
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "c"))
        .collect();
    srcs.sort();
    srcs.push(lib.join("port/posix.c"));
    srcs.push(proto.join("src/lr20xx_link.c"));
    if linux {
        srcs.push(lib.join("hal/linux.c"));
        b.define("LR20XX_HAL_LINUX", "1");
    }
    for s in &srcs {
        println!("cargo:rerun-if-changed={}", s.display());
        b.file(s);
    }
    println!("cargo:rerun-if-changed={}", lib.join("include").display());
    println!("cargo:rerun-if-changed={}", proto.join("include").display());
    b.compile("lr20xx");
    if !std::env::var("CARGO_CFG_TARGET_OS").unwrap_or_default().contains("macos") {
        println!("cargo:rustc-link-lib=m");
    }
    println!("cargo:include={}", lib.join("include").display());
}
