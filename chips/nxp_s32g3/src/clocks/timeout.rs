// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2026.

//! Wall-clock bounded polling using the ARM Cortex-M DWT cycle counter.
//!
//! During early clock initialization the only timer guaranteed available is the
//! ARM DWT cycle counter (CYCCNT). This module provides a minimal API that
//! ensures no polling loop can spin longer than a specified timeout, preventing
//! hangs if hardware fails to respond.
//!
//! The initial M7 clock source is FIRC (48 MHz). All timeout conversions
//! assume this frequency. After the clock tree is configured the DWT counter
//! still increments at the actual core clock frequency, but we no longer use
//! these helpers (the kernel's alarm subsystem takes over).

/// Initial M7 clock frequency (FIRC) — used to convert ms → cycles.
const FIRC_HZ: u32 = 48_000_000;

/// DEMCR register — bit 24 (TRCENA) enables DWT/ITM.
const DEMCR: *mut u32 = 0xE000_EDFC as *mut u32;
/// DWT Control Register — bit 0 (CYCCNTENA) enables the cycle counter.
const DWT_CTRL: *mut u32 = 0xE000_1000 as *mut u32;
/// DWT Cycle Count Register.
const DWT_CYCCNT: *const u32 = 0xE000_1004 as *const u32;
/// DWT Lock Access Register — must write 0xC5ACCE55 to unlock DWT writes.
const DWT_LAR: *mut u32 = 0xE000_1FB0 as *mut u32;

/// Ensure the DWT cycle counter is enabled. Idempotent.
pub fn dwt_enable() {
    // ### Safety
    // These are standard ARM Cortex-M debug registers at architecturally fixed
    // addresses. Writing the enable bits is idempotent and has no side-effects
    // beyond enabling the counter. The LAR unlock sequence is required on
    // Cortex-M7 to allow software writes to DWT_CTRL.
    unsafe {
        // Enable trace subsystem (TRCENA in DEMCR)
        let demcr = core::ptr::read_volatile(DEMCR);
        core::ptr::write_volatile(DEMCR, demcr | (1 << 24));
        // Unlock DWT (CoreSight LAR)
        core::ptr::write_volatile(DWT_LAR, 0xC5AC_CE55);
        // Enable cycle counter
        let ctrl = core::ptr::read_volatile(DWT_CTRL);
        core::ptr::write_volatile(DWT_CTRL, ctrl | 1);
    }
}

/// Read the current DWT cycle count.
#[inline(always)]
pub fn dwt_read() -> u32 {
    // ### Safety
    // DWT_CYCCNT is a read-only hardware counter at a fixed ARM address.
    unsafe { core::ptr::read_volatile(DWT_CYCCNT) }
}

/// Return the number of cycles elapsed since `start` (handles wrap).
#[inline(always)]
pub fn dwt_elapsed_since(start: u32) -> u32 {
    dwt_read().wrapping_sub(start)
}

/// Convert milliseconds to DWT cycle count at FIRC frequency.
#[inline(always)]
pub const fn ms_to_cycles(ms: u32) -> u32 {
    (FIRC_HZ / 1000) * ms
}
