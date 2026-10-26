// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
//! The chip's pins from Rust: a [`Hal`] bridged into the C library's lr20xx_hal_t.

use std::ffi::{c_int, c_void};
use std::panic::{catch_unwind, AssertUnwindSafe};

use lr20xx_sys as sys;

use crate::error::{Error, Result};

/// The chip's pins (lib/include/lr20xx_hal.h): an SPI bus with the chip's NSS, BUSY, NRESET, optionally a DIO
/// wired as an IRQ.
pub trait Hal {
    /// One frame, NSS low for all of it: `tx` out, and the first `rx.len()` bytes clocked in at the same time into
    /// `rx` (None: nothing wanted). `rx` is never longer than `tx`.
    fn xfer(&mut self, tx: &[u8], rx: Option<&mut [u8]>) -> Result<()>;
    /// BUSY's level: true while the chip works.
    fn busy(&mut self) -> Result<bool>;
    /// Drive NRESET (false: in reset).
    fn nreset(&mut self, high: bool) -> Result<()>;
    /// Wait until the IRQ pin is high: Some(true) at once if it is, Some(false) after timeout_ms; None if there's
    /// no IRQ pin.
    fn wait_irq(&mut self, _timeout_ms: u32) -> Option<bool> {
        None
    }
    /// Whether there's an IRQ pin (wait_irq answers).
    fn has_irq(&self) -> bool {
        false
    }
    /// The IRQ pin's level now, None without one.
    fn irq(&mut self) -> Option<bool> {
        None
    }
    /// Change the SPI clock; Err(NOT_SUPPORTED) if it's fixed.
    fn set_clock(&mut self, _hz: u32, _input_delay_ns: u32) -> Result<()> {
        Err(Error::new(Error::NOT_SUPPORTED, "fixed SPI clock"))
    }
}

type Boxed = Box<dyn Hal + Send>;

fn hal<'a>(ctx: *mut c_void) -> &'a mut Boxed {
    unsafe { &mut *(ctx as *mut Boxed) }
}

/// A Rust panic must not unwind into C: it becomes LR20XX_FAIL.
fn guard<T>(fallback: T, f: impl FnOnce() -> T) -> T {
    catch_unwind(AssertUnwindSafe(f)).unwrap_or(fallback)
}

fn code(r: Result<()>) -> sys::lr20xx_err_t {
    match r {
        Ok(()) => 0,
        Err(e) => e.code(),
    }
}

unsafe extern "C" fn t_xfer(
    ctx: *mut c_void,
    tx: *const u8,
    rx: *mut u8,
    len: usize,
    rx_len: usize,
) -> sys::lr20xx_err_t {
    guard(sys::LR20XX_FAIL, || {
        let tx = unsafe { std::slice::from_raw_parts(tx, len) };
        let rx = if rx_len > 0 && !rx.is_null() {
            Some(unsafe { std::slice::from_raw_parts_mut(rx, rx_len) })
        } else {
            None
        };
        code(hal(ctx).xfer(tx, rx))
    })
}

unsafe extern "C" fn t_busy(ctx: *mut c_void) -> c_int {
    guard(-1, || match hal(ctx).busy() {
        Ok(b) => b as c_int,
        Err(_) => -1,
    })
}

unsafe extern "C" fn t_nreset(ctx: *mut c_void, level: c_int) -> sys::lr20xx_err_t {
    guard(sys::LR20XX_FAIL, || code(hal(ctx).nreset(level != 0)))
}

unsafe extern "C" fn t_wait_irq(ctx: *mut c_void, timeout_ms: c_int) -> bool {
    guard(false, || hal(ctx).wait_irq(timeout_ms.max(0) as u32).unwrap_or(false))
}

unsafe extern "C" fn t_irq(ctx: *mut c_void) -> c_int {
    guard(-1, || match hal(ctx).irq() {
        Some(b) => b as c_int,
        None => -1,
    })
}

unsafe extern "C" fn t_set_clock(ctx: *mut c_void, hz: c_int, delay: c_int) -> sys::lr20xx_err_t {
    guard(sys::LR20XX_FAIL, || code(hal(ctx).set_clock(hz.max(0) as u32, delay.max(0) as u32)))
}

unsafe extern "C" fn t_close(ctx: *mut c_void) {
    let _ = catch_unwind(|| drop(unsafe { Box::from_raw(ctx as *mut Boxed) }));
}

/// The HAL as the C library takes it. The box is freed by the HAL's close, which lr20xx_open calls on a failure
/// and lr20xx_close on close.
pub(crate) fn into_raw<H: Hal + Send + 'static>(h: H) -> sys::lr20xx_hal_t {
    let irq = h.has_irq();
    let ctx = Box::into_raw(Box::new(Box::new(h) as Boxed)) as *mut c_void;
    sys::lr20xx_hal_t {
        ctx,
        xfer: Some(t_xfer),
        busy: Some(t_busy),
        nreset: Some(t_nreset),
        wait_irq: if irq { Some(t_wait_irq) } else { None },
        irq: if irq { Some(t_irq) } else { None },
        set_clock: Some(t_set_clock),
        close: Some(t_close),
    }
}
