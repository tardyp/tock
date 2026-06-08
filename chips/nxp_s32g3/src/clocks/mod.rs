// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2026.

//! S32G3 Clock Framework
//!
//! This module implements the full clock hierarchy for the NXP S32G3 SoC as
//! documented in the Reference Manual Chapter 24.
//!
//! # Clock Hierarchy
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────────────┐
//! │ Clock Sources                                                       │
//! │   FIRC (48 MHz)   SIRC (32 kHz)   FXOSC (20–40 MHz)               │
//! └──────────┬────────────┬──────────────────┬──────────────────────────┘
//!            │            │                  │
//!            ▼            │                  ▼
//! ┌──────────────────────────────────────────────────────────────────────┐
//! │ PLLs (ref: FIRC or FXOSC)                                           │
//! │   CORE_PLL (FM) ─── CORE_DFS (6 outputs)                           │
//! │   PERIPH_PLL ─────── PERIPH_DFS (6 outputs)                        │
//! │   DDR_PLL (FM)                                                      │
//! │   ACCEL_PLL (FM)                                                    │
//! └──────────┬──────────────────────────────────────────────────────────┘
//!            │
//!            ▼
//! ┌──────────────────────────────────────────────────────────────────────┐
//! │ MC_CGM_0 Muxes (clock selectors + dividers)                         │
//! │   Mux0 → XBAR_2X_CLK → /div → LBIST_CLK, DAPB_CLK                │
//! │   Mux3 → PER_CLK                                                   │
//! │   Mux7 → CAN_PE_CLK                                                │
//! │   Mux8 → LIN_BAUD_CLK (LINFlexD)                                  │
//! │   Mux12 → QSPI_2X_CLK                                             │
//! │   Mux14 → USDHC_CLK                                                │
//! │   Mux16 → SPI_CLK                                                  │
//! │                                                                      │
//! │ MC_CGM_1: Mux0 → A53_CORE_CLK                                      │
//! │ MC_CGM_5: Mux0 → DDR_CLK                                           │
//! └──────────────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Design
//!
//! Following the Tock clock interface pattern (see `kernel::platform::chip::ClockInterface`),
//! each clock source (FIRC, SIRC, FXOSC, PLL, DFS) is a separate struct with
//! `enable()` / `disable()` / `is_enabled()` / `get_frequency_hz()` methods.
//!
//! The top-level [`Clocks`] struct owns references to all clock sources and the
//! MC_CGM mux configuration, providing a unified entry point for clock setup.
//!
//! # Usage
//!
//! ```rust,ignore
//! let clocks = &peripherals.clocks;
//! // FIRC is always on after reset
//! assert!(clocks.firc.is_enabled());
//!
//! // Enable FXOSC (40 MHz crystal on the PCU board)
//! clocks.fxosc.set_frequency_mhz(40);
//! clocks.fxosc.enable().unwrap();
//!
//! // Configure PERIPH_PLL at 800 MHz VCO
//! clocks.periph_pll.configure(PllConfig { rdiv: 1, mfi: 40, mfn: 0 }).unwrap();
//! clocks.periph_pll.enable().unwrap();
//!
//! // Route LIN_BAUD_CLK via MC_CGM_0 mux 8
//! clocks.mc_cgm0.set_mux_source(8, CgmClockSource::PeriphPllPhi3).unwrap();
//! ```

pub mod clocks;
pub mod dfs;
pub mod firc;
pub mod fxosc;
pub mod mc_cgm;
pub mod pll;
pub mod sirc;
pub mod timeout;

pub use clocks::Clocks;
