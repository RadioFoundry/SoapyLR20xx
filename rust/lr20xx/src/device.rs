// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
//! An open chip (lib/include/lr20xx.h).

use std::ptr;

use lr20xx_sys as sys;

use crate::error::{c_str, check, Error, Result};
use crate::hal::{into_raw, Hal};

/// A command's status word (DS table 6-38).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Status {
    /// LR20XX_CMD_*: 0 FAIL, 1 PERR, 2 OK, 3 DATA.
    pub cmd: u8,
    pub irq: bool,
    pub reset: u8,
    pub mode: u8,
}

impl From<sys::lr20xx_status_t> for Status {
    fn from(s: sys::lr20xx_status_t) -> Self {
        Status { cmd: s.cmd, irq: s.irq, reset: s.reset, mode: s.mode }
    }
}

/// An open LR20xx. Closed (held in reset, the HAL released) on drop.
pub struct Device {
    raw: *mut sys::lr20xx_t,
}

// The handle is plain memory and the HAL is Send; one owner at a time.
unsafe impl Send for Device {}

impl Device {
    /// Take a HAL, reset the chip and wait for it.
    pub fn open<H: Hal + Send + 'static>(hal: H) -> Result<Self> {
        let cfg = sys::lr20xx_config_t { hal: into_raw(hal), mem: ptr::null_mut() };
        let mut raw = ptr::null_mut();
        let e = unsafe { sys::lr20xx_open(&cfg, &mut raw) };
        check(e, || "open".into())?;
        Ok(Device { raw })
    }

    /// The chip on Linux's spidev and GPIO character device (lib/hal/linux.c).
    #[cfg(target_os = "linux")]
    pub fn open_linux(cfg: &LinuxConfig) -> Result<Self> {
        use std::ffi::CString;
        let bad = |_| Error::new(Error::INVALID_ARG, "a path with a NUL");
        let spidev = CString::new(cfg.spidev.as_str()).map_err(bad)?;
        let chip = CString::new(cfg.gpiochip.as_str()).map_err(bad)?;
        let c = sys::lr20xx_linux_cfg_t {
            spidev: spidev.as_ptr(),
            speed_hz: cfg.speed_hz as i32,
            gpiochip: chip.as_ptr(),
            busy: cfg.busy as i32,
            nreset: cfg.nreset as i32,
            irq: cfg.irq.map_or(-1, |v| v as i32),
            pa_en: cfg.pa_en.map_or(-1, |v| v as i32),
        };
        let mut hal: sys::lr20xx_hal_t = unsafe { std::mem::zeroed() };
        let mut err = [0 as std::ffi::c_char; 160];
        let e = unsafe { sys::lr20xx_hal_linux_open(&c, &mut hal, err.as_mut_ptr(), err.len()) };
        check(e, || crate::error::c_text(&err))?;
        let cfg = sys::lr20xx_config_t { hal, mem: ptr::null_mut() };
        let mut raw = ptr::null_mut();
        check(unsafe { sys::lr20xx_open(&cfg, &mut raw) }, || "open".into())?;
        Ok(Device { raw })
    }

    pub fn as_ptr(&self) -> *mut sys::lr20xx_t {
        self.raw
    }

    fn check(&self, e: sys::lr20xx_err_t) -> Result<()> {
        check(e, || self.last_error())
    }

    /// What failed last, as the driver says it.
    pub fn last_error(&self) -> String {
        c_str(unsafe { sys::lr20xx_last_error(self.raw) })
    }

    /// Commands that came back FAIL or PERR, and BUSY timeouts.
    pub fn failures(&self) -> u32 {
        unsafe { sys::lr20xx_failures(self.raw) }
    }

    pub fn reset(&mut self) -> Result<()> {
        self.check(unsafe { sys::lr20xx_reset(self.raw) })
    }

    pub fn wake(&mut self) -> Result<()> {
        self.check(unsafe { sys::lr20xx_wake(self.raw) })
    }

    /// A write command (a frame from the `cmd` builders), its status checked.
    pub fn write(&mut self, frame: &[u8]) -> Result<()> {
        self.check(unsafe { sys::lr20xx_write(self.raw, frame.as_ptr(), frame.len()) })
    }

    /// A write command's frame alone, no status read.
    pub fn send(&mut self, frame: &[u8]) -> Result<()> {
        self.check(unsafe { sys::lr20xx_send(self.raw, frame.as_ptr(), frame.len()) })
    }

    /// A read command: the frame, then n bytes of answer.
    pub fn read(&mut self, frame: &[u8], n: usize) -> Result<Vec<u8>> {
        let mut out = vec![0; n];
        self.check(unsafe { sys::lr20xx_read(self.raw, frame.as_ptr(), frame.len(), out.as_mut_ptr(), n) })?;
        Ok(out)
    }

    /// n bytes of the RX FIFO (1024 at most).
    pub fn read_fifo(&mut self, out: &mut [u8]) -> Result<()> {
        self.check(unsafe { sys::lr20xx_read_fifo(self.raw, out.as_mut_ptr(), out.len()) })
    }

    pub fn write_tx_fifo(&mut self, data: &[u8]) -> Result<()> {
        self.check(unsafe { sys::lr20xx_write_tx_fifo(self.raw, data.as_ptr(), data.len()) })
    }

    pub fn status(&mut self) -> Result<Status> {
        let mut s: sys::lr20xx_status_t = unsafe { std::mem::zeroed() };
        self.check(unsafe { sys::lr20xx_get_status(self.raw, &mut s) })?;
        Ok(s.into())
    }

    /// The firmware (ROM) version: (major, minor). The LR2021's is 1.24 (0x01, 0x18).
    pub fn version(&mut self) -> Result<(u8, u8)> {
        let (mut a, mut b) = (0, 0);
        self.check(unsafe { sys::lr20xx_version(self.raw, &mut a, &mut b) })?;
        Ok((a, b))
    }

    pub fn errors(&mut self) -> Result<u16> {
        let mut v = 0;
        self.check(unsafe { sys::lr20xx_errors(self.raw, &mut v) })?;
        Ok(v)
    }

    pub fn rx_fifo_level(&mut self) -> Result<u16> {
        let mut v = 0;
        self.check(unsafe { sys::lr20xx_rx_fifo_level(self.raw, &mut v) })?;
        Ok(v)
    }

    pub fn tx_fifo_level(&mut self) -> Result<u16> {
        let mut v = 0;
        self.check(unsafe { sys::lr20xx_tx_fifo_level(self.raw, &mut v) })?;
        Ok(v)
    }

    /// The FIFO IRQ flags (rx, tx), cleared.
    pub fn fifo_flags(&mut self) -> Result<(u8, u8)> {
        let (mut r, mut t) = (0, 0);
        self.check(unsafe { sys::lr20xx_fifo_flags(self.raw, &mut r, &mut t) })?;
        Ok((r, t))
    }

    /// The FIFO IRQ flags (rx, tx), left set.
    pub fn fifo_flags_peek(&mut self) -> Result<(u8, u8)> {
        let (mut r, mut t) = (0, 0);
        self.check(unsafe { sys::lr20xx_fifo_flags_peek(self.raw, &mut r, &mut t) })?;
        Ok((r, t))
    }

    pub fn read32(&mut self, addr: u32) -> Result<u32> {
        let mut v = 0;
        self.check(unsafe { sys::lr20xx_read32(self.raw, addr, &mut v) })?;
        Ok(v)
    }

    /// 1 to 64 words from addr on.
    pub fn read_mem(&mut self, addr: u32, n: usize) -> Result<Vec<u32>> {
        if !(1..=64).contains(&n) {
            return Err(Error::new(Error::INVALID_SIZE, "1 to 64 words"));
        }
        let mut w = vec![0u32; n];
        self.check(unsafe { sys::lr20xx_read_mem(self.raw, addr, w.as_mut_ptr(), n as i32) })?;
        Ok(w)
    }

    /// The pending IRQs.
    pub fn irq(&mut self) -> Result<u32> {
        let mut v = 0;
        self.check(unsafe { sys::lr20xx_irq(self.raw, &mut v) })?;
        Ok(v)
    }

    pub fn has_irq(&self) -> bool {
        unsafe { sys::lr20xx_has_irq(self.raw) }
    }

    pub fn wait_irq(&mut self, timeout_ms: u32) -> bool {
        unsafe { sys::lr20xx_wait_irq(self.raw, timeout_ms.min(i32::MAX as u32) as i32) }
    }

    pub fn set_clock(&mut self, hz: u32, input_delay_ns: u32) -> Result<()> {
        self.check(unsafe { sys::lr20xx_set_clock(self.raw, hz as i32, input_delay_ns as i32) })
    }
}

impl Drop for Device {
    fn drop(&mut self) {
        unsafe { sys::lr20xx_close(self.raw) }
    }
}

/// Where the chip is on Linux (lib/include/lr20xx_hal_linux.h).
#[cfg(target_os = "linux")]
#[derive(Clone, Debug)]
pub struct LinuxConfig {
    /// "/dev/spidev0.0"
    pub spidev: String,
    /// 0: 16 MHz.
    pub speed_hz: u32,
    /// "/dev/gpiochip0"
    pub gpiochip: String,
    pub busy: u32,
    pub nreset: u32,
    pub irq: Option<u32>,
    pub pa_en: Option<u32>,
}
