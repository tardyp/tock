/* Licensed under the Apache License, Version 2.0 or the MIT License. */
/* SPDX-License-Identifier: Apache-2.0 OR MIT                         */
/* Copyright Tock Contributors 2026.                                  */

/* Memory layout for NXP S32G3 Main SoC M7_0.
 *
 * XMODEM mode deposits the payload at 0x34000000.
 * NOR/HSE mode deposits the same payload layout at 0x34200000.
 * The mode difference is only the executable SRAM base address; LMA == VMA
 * for all sections and writable data stays in DTCM.
 */

NXP_S32G3_IMAGE_BASE = DEFINED(NXP_S32G3_IMAGE_BASE) ? NXP_S32G3_IMAGE_BASE : 0x34000000;

MEMORY
{
  rom  (rx)  : ORIGIN = NXP_S32G3_IMAGE_BASE, LENGTH = 0x0003F000  /* 252 KiB — kernel code + rodata */
  prog (rx)  : ORIGIN = NXP_S32G3_IMAGE_BASE + 0x0003F000, LENGTH = 0x00008000  /* 32 KiB — TBF app area */
  ram  (rwx) : ORIGIN = 0x20000000, LENGTH = 0x0000E000  /* 56 KiB — DTCM: stack, BSS, data */
}
