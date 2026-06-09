/* Licensed under the Apache License, Version 2.0 or the MIT License. */
/* SPDX-License-Identifier: Apache-2.0 OR MIT                         */
/* Copyright Tock Contributors 2026.                                  */

/* NOR/HSE variant of layout.ld.
 *
 * This keeps the same section layout as XMODEM and only overrides the SRAM
 * execution base used by chip_layout.ld.
 */

NXP_S32G3_IMAGE_BASE = 0x34200000;

INCLUDE ./chip_layout.ld
INCLUDE tock_kernel_layout.ld
