// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
//! The chip as an SDR (lib/include/lr20xx_iq.h): raw I/Q receive sessions and sample transmit.

use std::ffi::c_void;
use std::marker::PhantomData;
use std::panic::{catch_unwind, AssertUnwindSafe};

use lr20xx_sys as sys;

use crate::board::{Board, Rom};
use crate::device::Device;
use crate::error::{c_text, check, Error, Result};

/// Receive modes (the table in lr20xx_iq.h).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IqMode {
    /// FSK16 up to 260 kS/s, then CONV8.
    Auto,
    /// 125 or 250 kS/s, cs16.
    Lora,
    /// An FSK rx_bw code's rate, cs16.
    Fsk16,
    /// Any rate, exact, through the chip's rate converter, cs8 (6 bits).
    Conv8,
}

impl IqMode {
    fn raw(self) -> sys::lr20xx_iq_mode_t {
        match self {
            IqMode::Auto => sys::LR20XX_IQ_AUTO,
            IqMode::Lora => sys::LR20XX_IQ_LORA,
            IqMode::Fsk16 => sys::LR20XX_IQ_FSK16,
            IqMode::Conv8 => sys::LR20XX_IQ_CONV8,
        }
    }
    fn from_raw(m: sys::lr20xx_iq_mode_t) -> IqMode {
        match m {
            sys::LR20XX_IQ_LORA => IqMode::Lora,
            sys::LR20XX_IQ_FSK16 => IqMode::Fsk16,
            sys::LR20XX_IQ_CONV8 => IqMode::Conv8,
            _ => IqMode::Auto,
        }
    }
}

/// Sample format: I then Q, little-endian.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IqFormat {
    Cs16,
    Cs8,
}

/// What a rate comes out as in a mode.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plan {
    pub mode: IqMode,
    pub format: IqFormat,
    pub sample_bytes: usize,
    pub rate_num: u32,
    pub rate_den: u32,
    pub rate: f64,
    pub passband_hz: f64,
    pub chip_rate: f64,
}

impl From<&sys::lr20xx_iq_plan_t> for Plan {
    fn from(p: &sys::lr20xx_iq_plan_t) -> Self {
        Plan {
            mode: IqMode::from_raw(p.mode),
            format: if p.format == sys::LR20XX_IQ_CS8 { IqFormat::Cs8 } else { IqFormat::Cs16 },
            sample_bytes: p.sample_bytes.max(0) as usize,
            rate_num: p.rate_num,
            rate_den: p.rate_den,
            rate: p.rate,
            passband_hz: p.passband_hz,
            chip_rate: p.chip_rate,
        }
    }
}

/// The plan for a rate in a mode.
pub fn plan(rate: f64, mode: IqMode) -> Result<Plan> {
    let mut p: sys::lr20xx_iq_plan_t = unsafe { std::mem::zeroed() };
    check(unsafe { sys::lr20xx_iq_plan(rate, mode.raw(), &mut p) }, || format!("no plan for {rate} S/s in {mode:?}"))?;
    Ok((&p).into())
}

/// A mode's rates: a list (LORA, FSK16), and for CONV8 a range.
pub fn rates(mode: IqMode) -> (Vec<f64>, Option<(f64, f64)>) {
    let mut v = vec![0f64; 256];
    let (mut lo, mut hi) = (0f64, 0f64);
    let n = unsafe { sys::lr20xx_iq_rates(mode.raw(), v.as_mut_ptr(), v.len() as i32, &mut lo, &mut hi) };
    v.truncate(n.max(0) as usize);
    let range = (mode == IqMode::Conv8 || mode == IqMode::Auto) && hi > 0.0;
    (v, range.then_some((lo, hi)))
}

/// The front end's gain at manual steps 1 to 13, dB.
pub fn gain_db() -> [f64; 13] {
    unsafe { sys::lr20xx_iq_gain_db }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IqConfig {
    pub rate: f64,
    pub mode: IqMode,
    pub center_hz: f64,
    pub ppm: f64,
    /// 1 to 13, a manual step; 0 the chip's AGC.
    pub gain: i32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct IqStats {
    pub samples: u64,
    pub losses: u32,
    pub restarts: u32,
    pub errors: u32,
    pub reads: u32,
    pub irq_late: u32,
}

/// The sink's flags: samples were lost just ahead of these.
pub const GAP: u32 = sys::LR20XX_IQ_GAP;

/// Transmit modes (lr20xx_tx.h).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TxMode {
    Cw,
    /// int8, count x fdev / 64 Hz.
    Freq,
    /// int8, count x rate / 128 Hz.
    PhaseStep,
    /// int8 I, Q: the phase only.
    IqPhase,
    /// int8 I, Q: phase and magnitude.
    IqPolar,
}

impl TxMode {
    fn raw(self) -> sys::lr20xx_tx_mode_t {
        match self {
            TxMode::Cw => sys::LR20XX_TX_CW,
            TxMode::Freq => sys::LR20XX_TX_FREQ,
            TxMode::PhaseStep => sys::LR20XX_TX_PHASE_STEP,
            TxMode::IqPhase => sys::LR20XX_TX_IQ_PHASE,
            TxMode::IqPolar => sys::LR20XX_TX_IQ_POLAR,
        }
    }
    /// FIFO bytes a sample.
    pub fn sample_bytes(self) -> usize {
        unsafe { sys::lr20xx_tx_sample_bytes(self.raw()) }.max(0) as usize
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TxConfig {
    pub mode: TxMode,
    /// 0: the receive center.
    pub center_hz: f64,
    pub ppm: f64,
    pub rate: u32,
    pub fdev_hz: u32,
    pub power_dbm: f64,
    /// The length in samples (0: unknown).
    pub samples: u64,
    /// 0: the default.
    pub max_ms: i32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TxStats {
    pub runs: i32,
    pub sent: usize,
    pub refills: u32,
    pub tx_flags: u8,
    pub early: bool,
    pub timed_out: bool,
    pub key_us: i64,
    pub end_us: i64,
}

/// An SDR on a device: the device is borrowed for the session's life.
pub struct IqSession<'d> {
    iq: Box<sys::lr20xx_iq_t>,
    _board: Board,
    _dev: PhantomData<&'d mut Device>,
}

unsafe impl Send for IqSession<'_> {}

unsafe extern "C" fn sink_tramp<F: FnMut(&[u8], u32, i64)>(
    ctx: *mut c_void,
    data: *const u8,
    n: usize,
    flags: u32,
    t_us: i64,
) {
    let (f, bytes) = unsafe { &mut *(ctx as *mut (F, usize)) };
    let s = unsafe { std::slice::from_raw_parts(data, n * *bytes) };
    let _ = catch_unwind(AssertUnwindSafe(|| f(s, flags, t_us)));
}

unsafe extern "C" fn pull_tramp<F: FnMut(&mut [u8]) -> usize>(ctx: *mut c_void, buf: *mut u8, max: usize) -> usize {
    let f = unsafe { &mut *(ctx as *mut F) };
    let b = unsafe { std::slice::from_raw_parts_mut(buf, max) };
    catch_unwind(AssertUnwindSafe(|| f(b).min(max))).unwrap_or(0)
}

impl<'d> IqSession<'d> {
    /// Take an open chip as an SDR: its ROM checked (allow_untested lets one past the table), no patch running,
    /// the chip set up.
    pub fn new(dev: &'d mut Device, board: Board, allow_untested: bool) -> Result<Self> {
        let mut iq: Box<sys::lr20xx_iq_t> = Box::new(unsafe { std::mem::zeroed() });
        let e = unsafe { sys::lr20xx_iq_init(&mut *iq, dev.as_ptr(), board.as_ptr(), allow_untested) };
        check(e, || c_text(&iq.error))?;
        Ok(IqSession { iq, _board: board, _dev: PhantomData })
    }

    fn err(&self, e: i32) -> Result<()> {
        check(e, || c_text(&self.iq.error))
    }

    /// The ROM's row, None for one allowed past the table.
    pub fn rom(&self) -> Option<Rom> {
        Rom::find(self.iq.rom_major, self.iq.rom_minor).filter(|_| !self.iq.rom.is_null())
    }

    pub fn rom_version(&self) -> (u8, u8) {
        (self.iq.rom_major, self.iq.rom_minor)
    }

    /// Start receiving (a session ends first).
    pub fn start(&mut self, cfg: &IqConfig) -> Result<Plan> {
        let c = sys::lr20xx_iq_cfg_t {
            rate: cfg.rate,
            mode: cfg.mode.raw(),
            center_hz: cfg.center_hz,
            ppm: cfg.ppm,
            gain: cfg.gain,
        };
        let e = unsafe { sys::lr20xx_iq_start(&mut *self.iq, &c) };
        self.err(e)?;
        Ok(self.plan())
    }

    /// The session's plan.
    pub fn plan(&self) -> Plan {
        (&self.iq.plan).into()
    }

    pub fn in_session(&self) -> bool {
        self.iq.in_session
    }

    pub fn set_gain(&mut self, gain: i32) -> Result<()> {
        let e = unsafe { sys::lr20xx_iq_set_gain(&mut *self.iq, gain) };
        self.err(e)
    }

    /// One reader step: up to wait_ms for samples, then `f(samples, flags, t_us)` with whole samples
    /// (plan().sample_bytes each). Returns the samples delivered.
    pub fn poll<F: FnMut(&[u8], u32, i64)>(&mut self, wait_ms: i32, f: F) -> Result<usize> {
        let mut ctx = (f, self.iq.plan.sample_bytes.max(1) as usize);
        let n = unsafe {
            sys::lr20xx_iq_poll(
                &mut *self.iq,
                Some(sink_tramp::<F>),
                &mut ctx as *mut (F, usize) as *mut c_void,
                wait_ms,
            )
        };
        if n < 0 {
            return Err(Error::new(n, c_text(&self.iq.error)));
        }
        Ok(n as usize)
    }

    /// End the session (standby, reset, set up again).
    pub fn stop(&mut self) {
        unsafe { sys::lr20xx_iq_stop(&mut *self.iq) }
    }

    pub fn stats(&self) -> IqStats {
        let s = &self.iq.stats;
        IqStats {
            samples: s.samples,
            losses: s.losses,
            restarts: s.restarts,
            errors: s.errors,
            reads: s.reads,
            irq_late: s.irq_late,
        }
    }

    pub fn last_error(&self) -> String {
        c_text(&self.iq.error)
    }

    /// One transmission (half duplex: a session pauses and resumes). `pull` fills its buffer with FIFO bytes in the
    /// mode's format and returns how many; 0 at the end.
    pub fn transmit<F: FnMut(&mut [u8]) -> usize>(&mut self, tx: &TxConfig, mut pull: F) -> Result<TxStats> {
        let t = sys::lr20xx_iq_tx_t {
            mode: tx.mode.raw(),
            center_hz: tx.center_hz,
            ppm: tx.ppm,
            rate: tx.rate,
            fdev_hz: tx.fdev_hz,
            power_dbm: tx.power_dbm,
            samples: tx.samples,
            max_ms: tx.max_ms,
            streamed: false,
            stop: std::ptr::null(),
        };
        let mut s: sys::lr20xx_tx_stream_stats_t = unsafe { std::mem::zeroed() };
        let e = unsafe {
            sys::lr20xx_iq_transmit(
                &mut *self.iq,
                &t,
                Some(pull_tramp::<F>),
                &mut pull as *mut F as *mut c_void,
                &mut s,
            )
        };
        self.err(e)?;
        Ok(TxStats {
            runs: s.runs,
            sent: s.sent,
            refills: s.refills,
            tx_flags: s.tx_flags,
            early: s.early,
            timed_out: s.timed_out,
            key_us: s.key_us,
            end_us: s.end_us,
        })
    }
}

impl Drop for IqSession<'_> {
    fn drop(&mut self) {
        if self.iq.in_session {
            self.stop();
        }
    }
}
