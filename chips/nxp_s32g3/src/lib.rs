// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2026.

#![no_std]

pub mod chip;
pub mod clocks;
pub mod linflexd;
pub mod mc_me;
pub mod mscm;
pub mod siul2;
pub mod stm;

use cortexm7::{initialize_ram_jump_to_main, unhandled_interrupt};
use cortexm7::{CortexM7, CortexMVariant};

// The S32G XMODEM and NOR/HSE handoffs branch directly to the image base.
// Keep executable Thumb code in `.multiboot`, which the Tock linker places
// before `.vectors`, so byte 0 is a real entry point instead of vector data.
// The trampoline initializes the ECC-protected DTCM stack window with word
// stores before moving MSP into it; the padding keeps `BASE_VECTORS` 1024-byte
// aligned for VTOR on Cortex-M7.
#[cfg(all(target_arch = "arm", target_os = "none"))]
core::arch::global_asm!(
    r#"
    .section .multiboot, "ax"
    .syntax unified
    .cpu cortex-m7
    .thumb

    .global nxp_s32g3_boot_entry
    .type nxp_s32g3_boot_entry, %function
    .thumb_func
nxp_s32g3_boot_entry:
    movw r0, #:lower16:_sstack
    movt r0, #:upper16:_sstack
    movw r1, #:lower16:_estack
    movt r1, #:upper16:_estack
    movs r2, #0
1:
    cmp  r0, r1
    bhs  2f
    str  r2, [r0], #4
    b    1b
2:
    mov  sp, r1
    b    initialize_ram_jump_to_main
    .size nxp_s32g3_boot_entry, . - nxp_s32g3_boot_entry

    .balign 1024, 0
"#,
);

extern "C" {
    fn _estack();
}

#[cfg_attr(
    all(target_arch = "arm", target_os = "none"),
    link_section = ".vectors"
)]
#[cfg_attr(all(target_arch = "arm", target_os = "none"), used)]
pub static BASE_VECTORS: [unsafe extern "C" fn(); 16] = [
    _estack,                      //  0 — initial stack pointer
    initialize_ram_jump_to_main,  //  1 — Reset
    unhandled_interrupt,          //  2 — NMI
    CortexM7::HARD_FAULT_HANDLER, //  3 — HardFault
    unhandled_interrupt,          //  4 — MemManage
    unhandled_interrupt,          //  5 — BusFault
    unhandled_interrupt,          //  6 — UsageFault
    unhandled_interrupt,          //  7 — reserved
    unhandled_interrupt,          //  8 — reserved
    unhandled_interrupt,          //  9 — reserved
    unhandled_interrupt,          // 10 — reserved
    CortexM7::SVC_HANDLER,        // 11 — SVCall
    unhandled_interrupt,          // 12 — reserved
    unhandled_interrupt,          // 13 — reserved
    unhandled_interrupt,          // 14 — PendSV
    CortexM7::SYSTICK_HANDLER,
];

#[cfg_attr(all(target_arch = "arm", target_os = "none"), link_section = ".irqs")]
#[cfg_attr(all(target_arch = "arm", target_os = "none"), used)]
pub static IRQS: [unsafe extern "C" fn(); mscm::NUM_EXTERNAL_IRQS] =
    [CortexM7::GENERIC_ISR; mscm::NUM_EXTERNAL_IRQS];

pub unsafe fn init() {
    crate::trace_sync!("init: start");
    cortexm7::nvic::disable_all();
    cortexm7::nvic::clear_all_pending();
    let vector_table: *const [unsafe extern "C" fn(); 16] = core::ptr::addr_of!(BASE_VECTORS);
    let vector_table: *const () = vector_table.cast();
    cortexm7::scb::set_vector_table_offset(vector_table);
    cortexm7::nvic::enable_all();
    crate::trace_sync!("init: end");
}
