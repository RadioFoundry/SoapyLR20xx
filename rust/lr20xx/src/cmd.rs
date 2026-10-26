// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
//! Command frames from the C builders (lr20xx_cmd.h and the other encoding headers), as byte vectors to pass to
//! [`Device::write`](crate::Device::write). A few common ones are wrapped; [`build`] calls any builder.

use lr20xx_sys as sys;

/// The largest frame a builder writes (the raw route is 13 bytes; CalibFE with several frequencies more).
pub const FRAME_MAX: usize = 64;

/// A frame from a builder: `f` gets a buffer of FRAME_MAX bytes and returns the length written.
///
/// # Safety
/// `f` must not write past FRAME_MAX bytes (the builders in the C headers write at most that).
pub unsafe fn build(f: impl FnOnce(*mut u8) -> usize) -> Vec<u8> {
    let mut b = [0u8; FRAME_MAX];
    let n = f(b.as_mut_ptr()).min(FRAME_MAX);
    b[..n].to_vec()
}

/// An opcode alone (GetVersion, ClearRxFifo...).
pub fn op(opcode: u16) -> Vec<u8> {
    unsafe { build(|b| sys::lr20xx_cmd(b, opcode)) }
}

/// An opcode and one byte (SetStandby, SetPacketType...).
pub fn op_u8(opcode: u16, a: u8) -> Vec<u8> {
    unsafe { build(|b| sys::lr20xx_cmd_u8(b, opcode, a)) }
}

pub fn set_rf_frequency(hz: u32) -> Vec<u8> {
    unsafe { build(|b| sys::lr20xx_cmd_set_rf_frequency(b, hz)) }
}

/// SetRx; 0xFFFFFF is continuous.
pub fn set_rx(timeout: u32) -> Vec<u8> {
    unsafe { build(|b| sys::lr20xx_cmd_set_rx(b, timeout)) }
}

pub fn set_tx(timeout: u32) -> Vec<u8> {
    unsafe { build(|b| sys::lr20xx_cmd_set_tx(b, timeout)) }
}

/// SetAgcGainManual: a step 1 to 13, 0 the AGC.
pub fn set_gain(step: u8) -> Vec<u8> {
    unsafe { build(|b| sys::lr20xx_cmd_set_gain(b, step)) }
}

pub fn write_reg32(addr: u32, value: u32) -> Vec<u8> {
    unsafe { build(|b| sys::lr20xx_cmd_write_reg32(b, addr, value)) }
}

pub fn read_reg32(addr: u32, words: u8) -> Vec<u8> {
    unsafe { build(|b| sys::lr20xx_cmd_read_reg32(b, addr, words)) }
}

/// The frequency word for hz, corrected for a crystal ppm off.
pub fn freq_corrected(hz: f64, ppm: f64) -> u32 {
    unsafe { sys::lr20xx_freq_corrected(hz, ppm) }
}

pub use sys::{
    LR20XX_CLEAR_RX_FIFO, LR20XX_GET_STATUS, LR20XX_GET_VERSION, LR20XX_SET_PACKET_TYPE, LR20XX_SET_STANDBY,
};
