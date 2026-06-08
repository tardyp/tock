// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2026.

//! DFS (Digital Frequency Synthesizer) driver for NXP S32G3.
//!
//! The S32G3 contains two DFS blocks (RM §24.2.7.4):
//!
//! - **CORE_DFS** — fed by CORE_PLL VCO, 6 output ports (CORE_DFS1..6_CLK)
//! - **PERIPH_DFS** — fed by PERIPH_PLL VCO, 6 output ports (PERIPH_DFS1..6_CLK)
//!
//! Each DFS port produces a divided clock from the PLL's VCO:
//!
//! ```text
//! f_DFS_port = f_VCO / (2 × (MFI + MFN / 36))
//! ```
//!
//! where MFI is an integer divider and MFN is a fractional divider (0..35).
//!
//! # Features
//!
//! - Multiple-phase divider outputs, controlled independently
//! - Phase dividers have independent resets
//! - CORE_DFS supports 6 outputs
//! - PERIPH_DFS supports 6 outputs
//!
//! See RM "DFS" chapter for register details.

use core::cell::Cell;

use kernel::platform::chip::ClockInterface;
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, register_structs, ReadOnly, ReadWrite};
use kernel::utilities::StaticRef;
use kernel::{debug, ErrorCode};

use super::timeout::{dwt_elapsed_since, dwt_enable, dwt_read, ms_to_cycles};

// ---------------------------------------------------------------------------
// DFS Base Addresses
// ---------------------------------------------------------------------------

/// CORE_DFS base address.
pub const CORE_DFS_BASE_ADDR: u32 = 0x4005_4000;
/// PERIPH_DFS base address.
pub const PERIPH_DFS_BASE_ADDR: u32 = 0x4005_8000;

/// Number of output ports per DFS.
pub const DFS_PORT_COUNT: usize = 6;

/// DFS port lock timeout (ms). Typically < 10 µs, allow 5 ms.
const DFS_LOCK_TIMEOUT_MS: u32 = 5;

// ---------------------------------------------------------------------------
// Register Definitions
// ---------------------------------------------------------------------------

register_structs! {
    /// DFS register block (one per DFS instance, RM §26.3.1).
    pub DfsRegisters {
        (0x000 => _reserved0),
        /// DFS Port Status Register
        (0x00C => pub portsr: ReadOnly<u32, DFS_PORTSR::Register>),
        /// DFS Port Loss-of-Lock Status Register
        (0x010 => pub portlolsr: ReadWrite<u32>),
        /// DFS Port Reset Register
        (0x014 => pub portreset: ReadWrite<u32, DFS_PORTRESET::Register>),
        /// DFS Control Register
        (0x018 => pub ctrl: ReadWrite<u32, DFS_CTRL::Register>),
        /// DFS Port Divider registers (DVPORT0..5) at offsets 0x01C + n*4
        (0x01C => pub dvport: [ReadWrite<u32, DFS_DVPORT::Register>; DFS_PORT_COUNT]),
        (0x034 => @END),
    }
}

register_bitfields![u32,
    /// DFS Control Register
    DFS_CTRL [
        /// DFS Reset: 1 = DFS in reset
        DFS_RESET OFFSET(1) NUMBITS(1) []
    ],

    /// DFS Port Status Register — each bit indicates lock for that port
    DFS_PORTSR [
        /// Port n locked (bit n corresponds to port n)
        PORTSTAT OFFSET(0) NUMBITS(6) []
    ],

    /// DFS Port Reset Register — each bit holds a port in reset
    DFS_PORTRESET [
        /// Port n reset (1 = held in reset, 0 = operational)
        PORTRESET OFFSET(0) NUMBITS(6) []
    ],

    /// DFS Port Divider Register (per port)
    DFS_DVPORT [
        /// Integer part of the divider (MFI). Valid: 1..255.
        MFI OFFSET(8) NUMBITS(8) [],
        /// Fractional part of the divider (MFN). Valid: 0..35.
        MFN OFFSET(0) NUMBITS(6) []
    ]
];

// ---------------------------------------------------------------------------
// DFS Instance Identifier
// ---------------------------------------------------------------------------

/// Identifies which DFS block this driver controls.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum DfsInstance {
    Core,
    Periph,
}

impl DfsInstance {
    /// Short uppercase name for log messages.
    pub fn name(self) -> &'static str {
        match self {
            DfsInstance::Core => "CORE_DFS",
            DfsInstance::Periph => "PERIPH_DFS",
        }
    }
}

// ---------------------------------------------------------------------------
// DFS Port Configuration
// ---------------------------------------------------------------------------

/// Configuration for a single DFS output port.
///
/// Output frequency: `f_port = f_vco / (2 × (mfi + mfn / 36))`
#[derive(Copy, Clone, Debug)]
pub struct DfsPortConfig {
    /// Port index (0..5).
    pub port: usize,
    /// Integer divider (MFI). Valid: 1..255.
    pub mfi: u8,
    /// Fractional divider (MFN). Valid: 0..35.
    pub mfn: u8,
}

// ---------------------------------------------------------------------------
// DFS Driver
// ---------------------------------------------------------------------------

/// DFS driver for a single S32G3 DFS block.
pub struct Dfs {
    registers: StaticRef<DfsRegisters>,
    instance: DfsInstance,
    /// VCO frequency from the parent PLL, in Hz. Must be set by the clocks
    /// layer after PLL configuration.
    vco_freq_hz: Cell<u32>,
}

impl Dfs {
    /// Create a DFS driver for the given instance.
    pub const fn new(instance: DfsInstance) -> Self {
        let base = match instance {
            DfsInstance::Core => CORE_DFS_BASE_ADDR,
            DfsInstance::Periph => PERIPH_DFS_BASE_ADDR,
        };
        Self {
            registers: unsafe { StaticRef::new(base as *const DfsRegisters) },
            instance,
            vco_freq_hz: Cell::new(0),
        }
    }

    /// Get the DFS instance identifier.
    pub fn instance(&self) -> DfsInstance {
        self.instance
    }

    /// Set the source VCO frequency (must be called after parent PLL is configured).
    pub fn set_vco_frequency_hz(&self, freq_hz: u32) {
        self.vco_freq_hz.set(freq_hz);
    }

    /// Take the DFS out of reset (enable the block).
    pub fn enable_dfs(&self) {
        let regs = &*self.registers;
        regs.ctrl.modify(DFS_CTRL::DFS_RESET::CLEAR);
    }

    /// Put the DFS into reset (disable the block).
    pub fn disable_dfs(&self) {
        let regs = &*self.registers;
        regs.ctrl.modify(DFS_CTRL::DFS_RESET::SET);
    }

    /// Check if the DFS block is out of reset.
    pub fn is_enabled_dfs(&self) -> bool {
        let regs = &*self.registers;
        !regs.ctrl.is_set(DFS_CTRL::DFS_RESET)
    }

    /// Configure and enable a DFS output port.
    ///
    /// The DFS block must already be out of reset.
    ///
    /// # Errors
    /// - [`ErrorCode::INVAL`]: port index or divider values out of range
    /// - [`ErrorCode::BUSY`]: port did not lock in time
    pub fn configure_port(&self, config: DfsPortConfig) -> Result<(), ErrorCode> {
        let name = self.instance.name();
        if config.port >= DFS_PORT_COUNT {
            return Err(ErrorCode::INVAL);
        }
        if config.mfi == 0 || config.mfn > 35 {
            return Err(ErrorCode::INVAL);
        }

        let regs = &*self.registers;
        let port_mask = 1u32 << config.port;

        // Already locked — almost certainly configured by BootROM/ATF. Skip
        // because tearing down CORE_DFS port 0 kills XBAR_2X_CLK, which feeds
        // the M7 system bus on S32G3 (instructions cease to fetch → silent
        // hang). We trust the live divider until callers explicitly disable.
        if (regs.portsr.get() & port_mask) != 0 {
            return Ok(());
        }

        // Hold port in reset while configuring.
        regs.portreset.modify(
            DFS_PORTRESET::PORTRESET.val(regs.portreset.read(DFS_PORTRESET::PORTRESET) | port_mask),
        );

        // Write divider values.
        regs.dvport[config.port]
            .write(DFS_DVPORT::MFI.val(config.mfi as u32) + DFS_DVPORT::MFN.val(config.mfn as u32));

        // Release port from reset.
        regs.portreset.modify(
            DFS_PORTRESET::PORTRESET
                .val(regs.portreset.read(DFS_PORTRESET::PORTRESET) & !port_mask),
        );

        // Wait for port to lock (wall-clock bounded).
        dwt_enable();
        let timeout = ms_to_cycles(DFS_LOCK_TIMEOUT_MS);
        let start = dwt_read();
        loop {
            let portsr = regs.portsr.get();
            let elapsed = dwt_elapsed_since(start);
            if (portsr & port_mask) != 0 {
                return Ok(());
            }
            if elapsed >= timeout {
                debug!(
                    "[{}] port {} TIMEOUT after {} cycles ({} ms)",
                    name, config.port, elapsed, DFS_LOCK_TIMEOUT_MS
                );
                return Err(ErrorCode::BUSY);
            }
        }
    }

    /// Disable (hold in reset) a DFS output port.
    pub fn disable_port(&self, port: usize) {
        if port >= DFS_PORT_COUNT {
            return;
        }
        let regs = &*self.registers;
        let port_mask = 1u32 << port;
        regs.portreset.modify(
            DFS_PORTRESET::PORTRESET.val(regs.portreset.read(DFS_PORTRESET::PORTRESET) | port_mask),
        );
    }

    /// Check if a port is locked (producing a valid clock).
    pub fn is_port_locked(&self, port: usize) -> bool {
        if port >= DFS_PORT_COUNT {
            return false;
        }
        let regs = &*self.registers;
        (regs.portsr.read(DFS_PORTSR::PORTSTAT) & (1u32 << port)) != 0
    }

    /// Get the output frequency of a DFS port in Hz.
    ///
    /// Returns `None` if the port is not locked or VCO is not configured.
    pub fn get_port_frequency_hz(&self, port: usize) -> Option<u32> {
        if port >= DFS_PORT_COUNT || !self.is_port_locked(port) {
            return None;
        }
        let vco = self.vco_freq_hz.get();
        if vco == 0 {
            return None;
        }

        let regs = &*self.registers;
        let dvport = regs.dvport[port].extract();
        let mfi = dvport.read(DFS_DVPORT::MFI);
        let mfn = dvport.read(DFS_DVPORT::MFN);

        if mfi == 0 {
            return None;
        }

        // f_port = f_vco / (2 × (mfi + mfn/36))
        // = f_vco × 36 / (2 × (mfi × 36 + mfn))
        // = f_vco × 18 / (mfi × 36 + mfn)
        let divisor = mfi * 36 + mfn;
        if divisor == 0 {
            return None;
        }
        let freq = (vco as u64) * 18 / (divisor as u64);
        Some(freq as u32)
    }
}

impl ClockInterface for Dfs {
    fn is_enabled(&self) -> bool {
        self.is_enabled_dfs()
    }

    fn enable(&self) {
        self.enable_dfs();
    }

    fn disable(&self) {
        self.disable_dfs();
    }
}
