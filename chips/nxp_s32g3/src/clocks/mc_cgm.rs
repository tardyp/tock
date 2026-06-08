// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2026.

//! MC_CGM (Clock Generation Module) driver for NXP S32G3.
//!
//! The MC_CGM modules contain clock mux selectors and dividers that route
//! PLL/oscillator outputs to peripheral and core clocks. RM §24.3.
//!
//! The S32G3 has multiple MC_CGM instances:
//!
//! - **MC_CGM_0** — Main peripheral clock muxes (XBAR, PER, CAN, LIN, SPI, QSPI, …)
//! - **MC_CGM_1** — A53 core clock
//! - **MC_CGM_2** — PFE clocks
//! - **MC_CGM_5** — DDR clock
//! - **MC_CGM_6** — GMAC clocks
//!
//! Each mux has:
//! - A Clock Select Control register (CSC) — selects the source
//! - A Clock Select Status register (CSS) — confirms the active source
//! - Zero or more Divider Control registers (DC_n) — enables and configures dividers
//!
//! # Clock Source Mapping (RM §24.3.1, Table 78)
//!
//! The `CgmClockSource` enum maps the clock selector index to the corresponding
//! source clock signal.
//!
//! # Key MC_CGM_0 Mux Assignments (RM §24.3.2.1, Table 79)
//!
//! | Mux | Selector Output | Sources |
//! |-----|-----------------|---------|
//! | 0   | XBAR_2X_CLK    | CORE_DFS1_CLK, FIRC_CLK |
//! | 3   | PER_CLK        | PERIPH_PLL_PHI1_CLK, FIRC_CLK |
//! | 7   | CAN_PE_CLK     | PERIPH_PLL_PHI2_CLK, FXOSC_CLK, FIRC_CLK |
//! | 8   | LIN_BAUD_CLK   | PERIPH_PLL_PHI3_CLK, FXOSC_CLK, FIRC_CLK |
//! | 12  | QSPI_2X_CLK   | PERIPH_DFS1_CLK, FIRC_CLK |
//! | 14  | USDHC_CLK      | PERIPH_DFS3_CLK, FIRC_CLK |
//! | 16  | SPI_CLK        | PERIPH_PLL_PHI7_CLK, FIRC_CLK |

use kernel::utilities::registers::interfaces::{Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, register_structs, ReadOnly, ReadWrite};
use kernel::utilities::StaticRef;
use kernel::{debug, ErrorCode};

use super::timeout::{dwt_elapsed_since, dwt_enable, dwt_read, ms_to_cycles};

// ---------------------------------------------------------------------------
// MC_CGM Base Addresses
// ---------------------------------------------------------------------------

/// MC_CGM_0 base address (RM §25.x).
pub const MC_CGM_0_BASE_ADDR: u32 = 0x4003_0000;
/// MC_CGM_1 base address.
pub const MC_CGM_1_BASE_ADDR: u32 = 0x4003_4000;
/// MC_CGM_2 base address.
pub const MC_CGM_2_BASE_ADDR: u32 = 0x4401_8000;
/// MC_CGM_5 base address.
pub const MC_CGM_5_BASE_ADDR: u32 = 0x4006_8000;
/// MC_CGM_6 base address.
pub const MC_CGM_6_BASE_ADDR: u32 = 0x4053_C000;

/// Maximum mux count per MC_CGM instance.
const MAX_MUX_COUNT: usize = 17;

/// Maximum dividers per mux.
const MAX_DIV_PER_MUX: usize = 2;

/// MC_CGM mux switch / divider update timeout (ms).
/// Mux switches typically complete in < 1 µs, allow 5 ms.
const CGM_TIMEOUT_MS: u32 = 5;

// ---------------------------------------------------------------------------
// Register Definitions
// ---------------------------------------------------------------------------

register_structs! {
    /// Per-mux register set (CSC + CSS + up to 2 dividers + DIV_UPD_STAT).
    ///
    /// Each mux occupies a 0x40-byte slot in the MC_CGM address space.
    /// Offset within MC_CGM: 0x300 + mux_index * 0x40.
    /// Layout per RM §25.x.x: CSC=0x00, CSS=0x04, DC_0=0x08, DC_1=0x0C,
    /// DIV_UPD_STAT=0x3C.
    pub CgmMuxRegisters {
        /// Clock Select Control: selects the clock source
        (0x00 => pub csc: ReadWrite<u32, MUX_CSC::Register>),
        /// Clock Select Status: shows the current active source
        (0x04 => pub css: ReadOnly<u32, MUX_CSS::Register>),
        /// Divider Control 0
        (0x08 => pub dc0: ReadWrite<u32, MUX_DC::Register>),
        /// Divider Control 1
        (0x0C => pub dc1: ReadWrite<u32, MUX_DC::Register>),
        (0x10 => _pad0),
        /// Divider Update Status (RM: offset 0x3C within mux slot)
        (0x3C => pub div_upd_stat: ReadOnly<u32, MUX_DIV_UPD_STAT::Register>),
        (0x40 => @END),
    }
}

register_structs! {
    /// MC_CGM register block. The mux array starts at offset 0x300.
    pub McCgmRegisters {
        (0x000 => _reserved_head),
        /// Per-mux register arrays (mux 0..16 for MC_CGM_0)
        (0x300 => pub mux: [CgmMuxRegisters; MAX_MUX_COUNT]),
        (0x300 + MAX_MUX_COUNT * 0x40 => @END),
    }
}

register_bitfields![u32,
    /// Mux Clock Select Control Register
    MUX_CSC [
        /// Clock source selector index (see CgmClockSource / RM Table 78)
        SELCTL OFFSET(24) NUMBITS(6) [],
        /// Clock switch request trigger (write 1 to initiate switch)
        CLK_SW OFFSET(2) NUMBITS(1) [],
        /// Safe Clock Select: force FIRC as source (for failover)
        SAFE_SW OFFSET(3) NUMBITS(1) [],
        /// Rampup/Rampdown enable (for PCFS)
        RAMPUP OFFSET(0) NUMBITS(1) [],
        RAMPDOWN OFFSET(1) NUMBITS(1) []
    ],

    /// Mux Clock Select Status Register
    MUX_CSS [
        /// Currently active clock source selector index
        SELSTAT OFFSET(24) NUMBITS(6) [],
        /// Clock switch in progress
        CLK_SW OFFSET(2) NUMBITS(1) [],
        /// Safe clock is active
        SAFE_SW OFFSET(3) NUMBITS(1) [],
        /// Switch was completed successfully
        SWIP OFFSET(16) NUMBITS(1) [],
        /// Switch trigger status
        SWTRG OFFSET(17) NUMBITS(3) []
    ],

    /// Mux Divider Control Register
    MUX_DC [
        /// Divider Enable
        DE  OFFSET(31) NUMBITS(1) [],
        /// Divider value. Actual division = DIV + 1.
        DIV OFFSET(16) NUMBITS(10) []
    ],

    /// Mux Divider Update Status
    MUX_DIV_UPD_STAT [
        /// Divider update in progress
        DIV_UPD_STAT OFFSET(0) NUMBITS(1) []
    ]
];

// ---------------------------------------------------------------------------
// Clock Source Enumeration (RM §24.3.1, Table 78)
// ---------------------------------------------------------------------------

/// Clock source index values for MC_CGM mux selectors.
///
/// These are the selector indices written to MUX_CSC[SELCTL] and read from
/// MUX_CSS[SELSTAT]. Only sources relevant to the main application clocks are
/// enumerated. See RM Table 78 for the complete mapping.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum CgmClockSource {
    /// FIRC_CLK (48 MHz)
    Firc = 0,
    /// SIRC_CLK (32 kHz)
    Sirc = 1,
    /// FXOSC_CLK (20–40 MHz)
    Fxosc = 2,
    /// CORE_PLL PHI0
    CorePllPhi0 = 4,
    /// CORE_PLL PHI1
    CorePllPhi1 = 5,
    /// CORE_DFS1_CLK (CORE_DFS port 0)
    CoreDfs1 = 12,
    /// CORE_DFS2_CLK (CORE_DFS port 1)
    CoreDfs2 = 13,
    /// CORE_DFS3_CLK (CORE_DFS port 2)
    CoreDfs3 = 14,
    /// CORE_DFS4_CLK (CORE_DFS port 3)
    CoreDfs4 = 15,
    /// CORE_DFS5_CLK (CORE_DFS port 4)
    CoreDfs5 = 16,
    /// CORE_DFS6_CLK (CORE_DFS port 5)
    CoreDfs6 = 17,
    /// PERIPH_PLL PHI0
    PeriphPllPhi0 = 18,
    /// PERIPH_PLL PHI1
    PeriphPllPhi1 = 19,
    /// PERIPH_PLL PHI2
    PeriphPllPhi2 = 20,
    /// PERIPH_PLL PHI3
    PeriphPllPhi3 = 21,
    /// PERIPH_PLL PHI4
    PeriphPllPhi4 = 22,
    /// PERIPH_PLL PHI5
    PeriphPllPhi5 = 23,
    /// PERIPH_PLL PHI6
    PeriphPllPhi6 = 24,
    /// PERIPH_PLL PHI7
    PeriphPllPhi7 = 25,
    /// PERIPH_DFS1_CLK (PERIPH_DFS port 0)
    PeriphDfs1 = 26,
    /// PERIPH_DFS2_CLK (PERIPH_DFS port 1)
    PeriphDfs2 = 27,
    /// PERIPH_DFS3_CLK (PERIPH_DFS port 2)
    PeriphDfs3 = 28,
    /// PERIPH_DFS4_CLK (PERIPH_DFS port 3)
    PeriphDfs4 = 29,
    /// PERIPH_DFS5_CLK (PERIPH_DFS port 4)
    PeriphDfs5 = 30,
    /// PERIPH_DFS6_CLK (PERIPH_DFS port 5)
    PeriphDfs6 = 31,
    /// ACCEL_PLL PHI0
    AccelPllPhi0 = 32,
    /// ACCEL_PLL PHI1
    AccelPllPhi1 = 33,
    /// DDR_PLL PHI0
    DdrPllPhi0 = 36,
}

// ---------------------------------------------------------------------------
// MC_CGM Instance Identifier
// ---------------------------------------------------------------------------

/// Identifies which MC_CGM instance.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum CgmInstance {
    Cgm0,
    Cgm1,
    Cgm2,
    Cgm5,
    Cgm6,
}

impl CgmInstance {
    /// Number of muxes available in this MC_CGM instance.
    fn num_muxes(self) -> usize {
        match self {
            CgmInstance::Cgm0 => 17, // mux 0..16
            CgmInstance::Cgm1 => 1,  // mux 0 only
            CgmInstance::Cgm2 => 10, // mux 0..9
            CgmInstance::Cgm5 => 1,  // mux 0 only
            CgmInstance::Cgm6 => 4,  // mux 0..3
        }
    }

    /// Short uppercase name for log messages.
    pub fn name(self) -> &'static str {
        match self {
            CgmInstance::Cgm0 => "CGM0",
            CgmInstance::Cgm1 => "CGM1",
            CgmInstance::Cgm2 => "CGM2",
            CgmInstance::Cgm5 => "CGM5",
            CgmInstance::Cgm6 => "CGM6",
        }
    }
}

// ---------------------------------------------------------------------------
// MC_CGM Driver
// ---------------------------------------------------------------------------

/// MC_CGM (Clock Generation Module) driver for a single instance.
///
/// Provides clock mux selection and divider configuration for routing PLLs
/// and oscillators to peripherals/cores.
pub struct McCgm {
    registers: StaticRef<McCgmRegisters>,
    instance: CgmInstance,
}

impl McCgm {
    /// Create a driver for the specified MC_CGM instance.
    pub const fn new(instance: CgmInstance) -> Self {
        let base = match instance {
            CgmInstance::Cgm0 => MC_CGM_0_BASE_ADDR,
            CgmInstance::Cgm1 => MC_CGM_1_BASE_ADDR,
            CgmInstance::Cgm2 => MC_CGM_2_BASE_ADDR,
            CgmInstance::Cgm5 => MC_CGM_5_BASE_ADDR,
            CgmInstance::Cgm6 => MC_CGM_6_BASE_ADDR,
        };
        Self {
            registers: unsafe { StaticRef::new(base as *const McCgmRegisters) },
            instance,
        }
    }

    /// Get the MC_CGM instance identifier.
    pub fn instance(&self) -> CgmInstance {
        self.instance
    }

    /// Select the clock source for a mux.
    ///
    /// Performs a glitchless clock switch (RM §24.1.1: "Glitchless clock
    /// switching").
    ///
    /// # Parameters
    /// - `mux`: mux index within this MC_CGM
    /// - `source`: desired clock source
    ///
    /// # Errors
    /// - [`ErrorCode::INVAL`]: mux index out of range
    /// - [`ErrorCode::BUSY`]: clock switch did not complete in time
    pub fn set_mux_source(&self, mux: usize, source: CgmClockSource) -> Result<(), ErrorCode> {
        let name = self.instance.name();
        if mux >= self.instance.num_muxes() {
            return Err(ErrorCode::INVAL);
        }

        let regs = &*self.registers;
        let mux_regs = &regs.mux[mux];

        // If already on the desired source and not switching, leave the mux
        // alone — needed for the muxes that feed the M7 system bus
        // (e.g. MC_CGM_0 mux 0 → XBAR_2X_CLK). Toggling CLK_SW on a live
        // bus-feeder glitches the bus and silently wedges the M7.
        let css = mux_regs.css.get();
        let cur_selstat = (css >> 24) & 0x3F;
        let swip = (css >> 16) & 0x1;
        if cur_selstat == source as u32 && swip == 0 {
            return Ok(());
        }

        // Write the source selector and trigger the switch.
        mux_regs
            .csc
            .write(MUX_CSC::SELCTL.val(source as u32) + MUX_CSC::CLK_SW::SET);

        // Wait for the switch to complete (wall-clock bounded).
        dwt_enable();
        let timeout = ms_to_cycles(CGM_TIMEOUT_MS);
        let start = dwt_read();
        loop {
            let css = mux_regs.css.get();
            let elapsed = dwt_elapsed_since(start);
            let selstat = (css >> 24) & 0x3F;
            let swip = (css >> 16) & 0x1;
            if selstat == source as u32 && swip == 0 {
                return Ok(());
            }
            if elapsed >= timeout {
                debug!(
                    "[{} mux{}] TIMEOUT after {} cycles ({} ms) (CSS=0x{:08x})",
                    name, mux, elapsed, CGM_TIMEOUT_MS, css
                );
                return Err(ErrorCode::BUSY);
            }
        }
    }

    /// Get the currently active clock source for a mux.
    ///
    /// Returns `None` if the mux index is out of range.
    pub fn get_mux_source(&self, mux: usize) -> Option<u8> {
        if mux >= self.instance.num_muxes() {
            return None;
        }
        let regs = &*self.registers;
        Some(regs.mux[mux].css.read(MUX_CSS::SELSTAT) as u8)
    }

    /// Enable and configure a clock divider on a mux.
    ///
    /// # Parameters
    /// - `mux`: mux index
    /// - `div_index`: divider index (0 or 1)
    /// - `div_value`: divider value (actual division = div_value + 1)
    ///
    /// # Errors
    /// - [`ErrorCode::INVAL`]: mux or divider index out of range
    /// - [`ErrorCode::BUSY`]: divider update did not complete in time
    pub fn set_mux_divider(
        &self,
        mux: usize,
        div_index: usize,
        div_value: u16,
    ) -> Result<(), ErrorCode> {
        let name = self.instance.name();
        if mux >= self.instance.num_muxes() || div_index >= MAX_DIV_PER_MUX {
            return Err(ErrorCode::INVAL);
        }

        let regs = &*self.registers;
        let mux_regs = &regs.mux[mux];

        let dc_reg = if div_index == 0 {
            &mux_regs.dc0
        } else {
            &mux_regs.dc1
        };

        dc_reg.write(MUX_DC::DE::SET + MUX_DC::DIV.val(div_value as u32));

        dwt_enable();
        let timeout = ms_to_cycles(CGM_TIMEOUT_MS);
        let start = dwt_read();
        loop {
            let div_upd = mux_regs.div_upd_stat.get();
            let elapsed = dwt_elapsed_since(start);
            if (div_upd & 0x1) == 0 {
                return Ok(());
            }
            if elapsed >= timeout {
                debug!(
                    "[{} mux{}] divider update TIMEOUT after {} cycles (DIV_UPD_STAT=0x{:08x})",
                    name, mux, elapsed, div_upd
                );
                return Err(ErrorCode::BUSY);
            }
        }
    }

    /// Disable a clock divider on a mux.
    pub fn disable_mux_divider(&self, mux: usize, div_index: usize) {
        if mux >= self.instance.num_muxes() || div_index >= MAX_DIV_PER_MUX {
            return;
        }

        let regs = &*self.registers;
        let mux_regs = &regs.mux[mux];

        let dc_reg = if div_index == 0 {
            &mux_regs.dc0
        } else {
            &mux_regs.dc1
        };

        dc_reg.write(MUX_DC::DE::CLEAR + MUX_DC::DIV.val(0));
    }

    /// Force a mux to the safe clock (FIRC).
    ///
    /// Used during reset domain management (RM §24.4).
    pub fn force_safe_clock(&self, mux: usize) -> Result<(), ErrorCode> {
        self.set_mux_source(mux, CgmClockSource::Firc)
    }
}
