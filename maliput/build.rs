// BSD 3-Clause License
//
// Copyright (c) 2025, Woven by Toyota.
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

    // This is a workaround to correctly have the .so files linked to the final binary.
    // Cargo doesn’t propagate the crate’s link arguments to higher-level crates.
    // We set two arguments: rpath and link-search. The rpath is intended for the final binary to link to the library, and link-search allows Cargo to locate the library.
    //
    // See: https://github.com/rust-lang/cargo/issues/5077
    let maliput_sdk_out_root = PathBuf::from(env::var("DEP_MALIPUT_SDK_ROOT").expect("DEP_MALIPUT_SDK_ROOT not set"));
    let maliput_sdk_so_folder = maliput_sdk_out_root.join("bazel_output_base").join("bazel-bin");
    println!("cargo:rustc-link-search=native={}", maliput_sdk_so_folder.display());
    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", maliput_sdk_so_folder.display());
    Ok(())
}
