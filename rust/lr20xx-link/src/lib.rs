// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
//! The SoapyLR20xx bridge link in pure Rust: how a host drives an LR20xx on a microcontroller (the repo's
//! firmware/) over TCP or a USB serial port. The encoding matches proto/include/lr20xx_link.h byte for byte (the
//! C is the reference; the lr20xx crate's tests compare the two). docs/link.md describes the protocol.

pub mod client;
pub mod codec;

pub use client::{discover, discover_at, Client, Error, Result, RxBlock};
pub use codec::*;
