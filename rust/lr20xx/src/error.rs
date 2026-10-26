// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
//! Errors: the C library's lr20xx_err_t, with the driver's text where there is some.

use std::ffi::CStr;
use std::fmt;

use lr20xx_sys as sys;

/// An lr20xx_err_t (ESP-IDF's values) and what failed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Error {
    code: i32,
    detail: String,
}

impl Error {
    pub fn new(code: i32, detail: impl Into<String>) -> Self {
        Error { code, detail: detail.into() }
    }
    /// The C error code.
    pub fn code(&self) -> i32 {
        self.code
    }
    /// Its name, as lr20xx_err_name gives it ("LR20XX_ERR_TIMEOUT").
    pub fn name(&self) -> &'static str {
        err_name(self.code)
    }
    pub fn detail(&self) -> &str {
        &self.detail
    }

    pub const FAIL: i32 = sys::LR20XX_FAIL;
    pub const NO_MEM: i32 = sys::LR20XX_ERR_NO_MEM as i32;
    pub const INVALID_ARG: i32 = sys::LR20XX_ERR_INVALID_ARG as i32;
    pub const INVALID_STATE: i32 = sys::LR20XX_ERR_INVALID_STATE as i32;
    pub const INVALID_SIZE: i32 = sys::LR20XX_ERR_INVALID_SIZE as i32;
    pub const NOT_FOUND: i32 = sys::LR20XX_ERR_NOT_FOUND as i32;
    pub const NOT_SUPPORTED: i32 = sys::LR20XX_ERR_NOT_SUPPORTED as i32;
    pub const TIMEOUT: i32 = sys::LR20XX_ERR_TIMEOUT as i32;
    pub const INVALID_RESPONSE: i32 = sys::LR20XX_ERR_INVALID_RESPONSE as i32;
    pub const INVALID_CRC: i32 = sys::LR20XX_ERR_INVALID_CRC as i32;
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.detail.is_empty() {
            write!(f, "{}", self.name())
        } else {
            write!(f, "{}: {}", self.name(), self.detail)
        }
    }
}

impl std::error::Error for Error {}

pub type Result<T> = std::result::Result<T, Error>;

pub fn err_name(code: i32) -> &'static str {
    // lr20xx_err_name returns static strings
    unsafe { CStr::from_ptr(sys::lr20xx_err_name(code)) }.to_str().unwrap_or("?")
}

/// A C string field (NUL-terminated within the array) as a String.
pub(crate) fn c_text(buf: &[std::ffi::c_char]) -> String {
    let bytes: Vec<u8> = buf.iter().take_while(|&&c| c != 0).map(|&c| c.to_ne_bytes()[0]).collect(); // c_char: i8 or u8 by target
    String::from_utf8_lossy(&bytes).into_owned()
}

/// A C string pointer as a String ("" for NULL).
pub(crate) fn c_str(p: *const std::ffi::c_char) -> String {
    if p.is_null() {
        String::new()
    } else {
        unsafe { CStr::from_ptr(p) }.to_string_lossy().into_owned()
    }
}

pub(crate) fn check(code: i32, detail: impl FnOnce() -> String) -> Result<()> {
    if code == 0 {
        Ok(())
    } else {
        Err(Error::new(code, detail()))
    }
}
