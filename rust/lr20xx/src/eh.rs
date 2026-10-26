// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
//! An adapter from embedded-hal 1.0 to the chip's [`Hal`]: an SpiDevice (NSS its chip select), BUSY and an
//! optional IRQ as InputPins, NRESET as an OutputPin. For std hosts (the library's clock is lib/port/posix.c);
//! a no_std target needs a port of its own.

use std::time::{Duration, Instant};

use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::SpiDevice;

use crate::error::{Error, Result};
use crate::hal::Hal;

/// No IRQ pin.
pub struct NoPin;

impl embedded_hal::digital::ErrorType for NoPin {
    type Error = core::convert::Infallible;
}

impl InputPin for NoPin {
    fn is_high(&mut self) -> core::result::Result<bool, Self::Error> {
        Ok(false)
    }
    fn is_low(&mut self) -> core::result::Result<bool, Self::Error> {
        Ok(true)
    }
}

pub struct EhHal<S, B, R, I = NoPin> {
    pub spi: S,
    pub busy: B,
    pub nreset: R,
    pub irq: Option<I>,
}

impl<S, B, R> EhHal<S, B, R, NoPin> {
    pub fn new(spi: S, busy: B, nreset: R) -> Self {
        EhHal { spi, busy, nreset, irq: None }
    }
}

impl<S, B, R, I> EhHal<S, B, R, I> {
    pub fn with_irq(spi: S, busy: B, nreset: R, irq: I) -> Self {
        EhHal { spi, busy, nreset, irq: Some(irq) }
    }
}

fn fail<E: core::fmt::Debug>(what: &'static str) -> impl FnOnce(E) -> Error {
    move |e| Error::new(Error::FAIL, format!("{what}: {e:?}"))
}

impl<S: SpiDevice, B: InputPin, R: OutputPin, I: InputPin> Hal for EhHal<S, B, R, I> {
    fn xfer(&mut self, tx: &[u8], rx: Option<&mut [u8]>) -> Result<()> {
        match rx {
            // SpiDevice::transfer keeps the first read.len() bytes clocked in and discards the rest
            Some(r) => self.spi.transfer(r, tx).map_err(fail("SPI")),
            None => self.spi.write(tx).map_err(fail("SPI")),
        }
    }
    fn busy(&mut self) -> Result<bool> {
        self.busy.is_high().map_err(fail("BUSY"))
    }
    fn nreset(&mut self, high: bool) -> Result<()> {
        if high { self.nreset.set_high() } else { self.nreset.set_low() }.map_err(fail("NRESET"))
    }
    fn has_irq(&self) -> bool {
        self.irq.is_some()
    }
    fn irq(&mut self) -> Option<bool> {
        self.irq.as_mut().and_then(|p| p.is_high().ok())
    }
    fn wait_irq(&mut self, timeout_ms: u32) -> Option<bool> {
        let p = self.irq.as_mut()?;
        let end = Instant::now() + Duration::from_millis(timeout_ms as u64);
        loop {
            if p.is_high().unwrap_or(false) {
                return Some(true);
            }
            if Instant::now() >= end {
                return Some(false);
            }
            std::thread::sleep(Duration::from_micros(50));
        }
    }
}
