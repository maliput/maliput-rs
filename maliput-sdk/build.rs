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
use std::fs;
use std::path::PathBuf;

fn main() -> Result<(), Box<dyn Error>> {
    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=BUILD.bazel");
    println!("cargo:rerun-if-changed=MODULE.bazel");
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let bazel_output_base_dir = out_dir.join("bazel_output_base");
    fs::create_dir_all(&bazel_output_base_dir)?;

    // Bazel build
    let code = std::process::Command::new("bazel")
        .arg(format!("--output_base={}", bazel_output_base_dir.display()))
        .arg("build")
        .arg(format!("--symlink_prefix={}", bazel_output_base_dir.join("bazel-").display()))
        .arg("//...")
        .status()
        .expect("Failed to generate build script");
    if code.code() != Some(0) {
        panic!("Failed to generate build script");
    }
    let bazel_bin_dir = bazel_output_base_dir.join("bazel-bin");


    // ************* Maliput Files ************* //

    // TODO(francocipollone): Get version from MODULE.bazel configuration.
    let maliput_version = "1.2.0";
    let maliput_bin_path = bazel_bin_dir.join("external").join(format!("maliput~{}", maliput_version));

    //---Header files---
    let virtual_includes_path = maliput_bin_path.join("_virtual_includes");
    let mut virtual_includes = Vec::new();
    for entry in fs::read_dir(virtual_includes_path)? {
        let entry = entry?;
        let path = entry.path();
        if path.is_dir() {
            virtual_includes.push(path);
        }
    }

    // Add all the virtual includes to CXXBRIDGE_DIR.
    for (i, path) in virtual_includes.iter().enumerate() {
        println!("cargo:CXXBRIDGE_DIR{}={}", i, path.display());
    }

    // Environment variable to pass down to this crate:
    println!("cargo:rustc-env=MALIPUT_BIN_PATH={}", maliput_bin_path.display());

    // Environment variable to pass down to dependent crates:
    // See: https://doc.rust-lang.org/cargo/reference/build-scripts.html#the-links-manifest-key
    println!("cargo:root={}", out_dir.display()); //> Accessed as MALIPUT_SDK_ROOT
    println!("cargo:maliput_bin_path={}", maliput_bin_path.display()); //> Accessed as MALIPUT_SDK_MALIPUT_BIN_PATH

    Ok(())

}
