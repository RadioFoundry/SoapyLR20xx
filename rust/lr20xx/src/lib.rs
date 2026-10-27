// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
//! A Semtech LR20xx (LR2021, LR2022, LR2012) as an SDR, from Rust: a safe API over liblr20xx, the C core in the
//! repo's lib/. The chip logic is the C's alone; this crate binds it. Std hosts only (the C's clock is
//! lib/port/posix.c).
//!
//! - [`Device`]: an open chip over a [`Hal`] (your SPI and pins), or on Linux over spidev (`Device::open_linux`).
//! - [`IqSession`]: raw I/Q receive and sample transmit (lr20xx_iq.h, docs/iq.md).
//! - [`Board`], [`Rom`]: what's around the chip, and which ROMs the raw paths are known on.
//! - [`cmd`]: command frames from the C builders.

pub mod board;
pub mod cmd;
pub mod device;
#[cfg(feature = "embedded-hal")]
pub mod eh;
pub mod error;
pub mod hal;
pub mod iq;

pub use board::{Board, Rom};
#[cfg(target_os = "linux")]
pub use device::LinuxConfig;
pub use device::{Device, Status};
pub use error::{err_name, Error, Result};
pub use hal::Hal;
pub use iq::{
    gain_db, plan, rates, IqConfig, IqFormat, IqMode, IqSession, IqStats, Plan, TxConfig, TxMode, TxStats, GAP,
};

/// The raw bindings, for what the safe API doesn't cover.
pub use lr20xx_sys as sys;

/// The library's clock, us: what [`iq::TxConfig::at_us`] and the receive callback's times are on.
pub fn now_us() -> i64 {
    unsafe { sys::lr20xx_now_us() }
}
