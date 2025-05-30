// BSD 3-Clause License
//
// Copyright (c) 2024, Woven by Toyota.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

use std::env;
use std::error::Error;
use std::path::PathBuf;

fn main() -> Result<(), Box<dyn Error>> {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=src/api/api.h");
    println!("cargo:rerun-if-changed=src/api/mod.rs");
    println!("cargo:rerun-if-changed=src/api/rules/aliases.h");
    println!("cargo:rerun-if-changed=src/api/rules/rules.h");
    println!("cargo:rerun-if-changed=src/api/rules/mod.rs");
    println!("cargo:rerun-if-changed=src/common/common.h");
    println!("cargo:rerun-if-changed=src/common/mod.rs");
    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=src/math/math.h");
    println!("cargo:rerun-if-changed=src/math/mod.rs");
    println!("cargo:rerun-if-changed=src/plugin/mod.rs");
    println!("cargo:rerun-if-changed=src/plugin/plugin.h");
    println!("cargo:rerun-if-changed=src/utility/mod.rs");
    println!("cargo:rerun-if-changed=src/utility/utility.h");

    let maliput_bin_path =
        PathBuf::from(env::var("DEP_MALIPUT_SDK_MALIPUT_BIN_PATH").expect("DEP_MALIPUT_SDK_MALIPUT_BIN_PATH not set"));

    println!("cargo:rustc-link-search=native={}", maliput_bin_path.display());

    // Link to all the libraries in the bazel-bin folder.
    // The order of these is important! Otherwise, we get undefined reference errors.
    println!("cargo:rustc-link-lib=math");
    println!("cargo:rustc-link-lib=common");
    println!("cargo:rustc-link-lib=drake");
    println!("cargo:rustc-link-lib=geometry_base");
    println!("cargo:rustc-link-lib=plugin");
    println!("cargo:rustc-link-lib=utility");
    println!("cargo:rustc-link-lib=base");
    println!("cargo:rustc-link-lib=api");

    cxx_build::bridges([
        "src/math/mod.rs",
        "src/utility/mod.rs",
        "src/api/rules/mod.rs",
        "src/api/mod.rs",
        "src/plugin/mod.rs",
        "src/common/mod.rs",
    ])
    .flag_if_supported("-std=c++17")
    .include("src")
    .compile("maliput-sys");

    let maliput_malidrive_plugin_path = PathBuf::from(
        env::var("DEP_MALIPUT_SDK_MALIPUT_MALIDRIVE_PLUGIN_PATH")
            .expect("DEP_MALIPUT_SDK_MALIPUT_MALIDRIVE_PLUGIN_PATH not set"),
    );

    // Environment variables are available from within binaries and tests in the crate.
    println!(
        "cargo:rustc-env=MALIPUT_PLUGIN_PATH={}",
        maliput_malidrive_plugin_path.display()
    );

    Ok(())
}
