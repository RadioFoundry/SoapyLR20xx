// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
//! Boards (lib/include/lr20xx_board.h) and the ROM table (lr20xx_chips.h).

use std::ffi::{c_char, CString};

use lr20xx_sys as sys;

use crate::error::{c_str, c_text, Error, Result};

enum Inner {
    Static(&'static sys::lr20xx_board_t),
    // boxed: the board's name points into the buffer
    Owned(Box<sys::lr20xx_board_buf_t>),
}

/// What a board puts around the chip: clock, RF switch, IRQ DIO, range.
pub struct Board(Inner);

// The static boards are constant data; an owned one is ours alone.
unsafe impl Send for Board {}
unsafe impl Sync for Board {}

impl Board {
    /// A board by name ("generic", "w12").
    pub fn find(name: &str) -> Option<Board> {
        let c = CString::new(name).ok()?;
        let p = unsafe { sys::lr20xx_board_find(c.as_ptr()) };
        (!p.is_null()).then(|| Board(Inner::Static(unsafe { &*p })))
    }

    /// A bare chip: no TCXO, no RF switch, no IRQ DIO.
    pub fn generic() -> Board {
        Board(Inner::Static(unsafe { &*std::ptr::addr_of!(sys::lr20xx_board_generic) }))
    }

    /// A board from text ("w12", "generic tcxo=3 irq=7", ...; lr20xx_board_parse).
    pub fn parse(text: &str) -> Result<Board> {
        let c = CString::new(text).map_err(|_| Error::new(Error::INVALID_ARG, "text with a NUL"))?;
        let mut buf: Box<sys::lr20xx_board_buf_t> = Box::new(unsafe { std::mem::zeroed() });
        let mut err = [0 as c_char; 128];
        let r = unsafe { sys::lr20xx_board_parse(c.as_ptr(), &mut *buf, err.as_mut_ptr(), err.len()) };
        if r < 0 {
            return Err(Error::new(Error::INVALID_ARG, c_text(&err)));
        }
        // the name was set to point into this buffer; it moved into the box before the call, so it's stable
        Ok(Board(Inner::Owned(buf)))
    }

    pub fn raw(&self) -> &sys::lr20xx_board_t {
        match &self.0 {
            Inner::Static(b) => b,
            Inner::Owned(b) => &b.b,
        }
    }

    pub fn as_ptr(&self) -> *const sys::lr20xx_board_t {
        self.raw()
    }

    pub fn name(&self) -> String {
        c_str(self.raw().name)
    }

    /// What the front end passes, Hz.
    pub fn freq_range(&self) -> (f64, f64) {
        (self.raw().freq_min, self.raw().freq_max)
    }

    pub fn tx_power_max(&self) -> f64 {
        self.raw().tx_power_max
    }

    pub fn irq_dio(&self) -> Option<u8> {
        let d = self.raw().irq_dio;
        (d >= 0).then_some(d as u8)
    }
}

/// A ROM (firmware) version the raw paths are known on.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Rom {
    pub major: u8,
    pub minor: u8,
    pub part: String,
    pub iq_rx: bool,
    pub iq_tx: bool,
    pub tested: String,
}

impl Rom {
    fn from_raw(r: &sys::lr20xx_rom_t) -> Rom {
        Rom {
            major: r.major,
            minor: r.minor,
            part: c_str(r.part),
            iq_rx: r.iq_rx,
            iq_tx: r.iq_tx,
            tested: c_str(r.tested),
        }
    }

    /// The row for a version, None for one not run yet.
    pub fn find(major: u8, minor: u8) -> Option<Rom> {
        let p = unsafe { sys::lr20xx_rom_find(major, minor) };
        (!p.is_null()).then(|| Rom::from_raw(unsafe { &*p }))
    }

    /// Every row.
    pub fn all() -> Vec<Rom> {
        let mut n = 0;
        let p = unsafe { sys::lr20xx_roms(&mut n) };
        if p.is_null() || n <= 0 {
            return Vec::new();
        }
        unsafe { std::slice::from_raw_parts(p, n as usize) }.iter().map(Rom::from_raw).collect()
    }
}
