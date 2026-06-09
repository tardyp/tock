// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2026.

fn main() {
    tock_build_scripts::default::rustflags_check();
    tock_build_scripts::default::include_tock_kernel_layout();
    tock_build_scripts::default::add_board_dir_to_linker_search_path();

    println!("cargo:rerun-if-env-changed=NXP_S32G3_BOOT_MODE");
    let linker_script = match std::env::var("NXP_S32G3_BOOT_MODE").as_deref() {
        Ok("nor") => "layout_nor.ld",
        Ok("xmodem") | Err(_) => "layout.ld",
        Ok(other) => panic!("unsupported NXP_S32G3_BOOT_MODE={other}; expected `xmodem` or `nor`"),
    };

    tock_build_scripts::default::set_and_track_linker_script(linker_script);
}
