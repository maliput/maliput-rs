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

/// Gets the bazel library version.
///
/// Executes "bazel mod explain" and looks for a line with `library_name` to retrieve its version.
/// This function will fail when either:
/// - The bazel execution fails.
/// - The library cannot be found.
/// - The version cannot be retrieved.
///
/// ```rust
/// let maliput_library_name: str = "maliput";
/// println!("Maliput library version is: {}", get_bazel_library_version(maliput_library_name));
/// ```
/// ### Arguments
///
/// * `library_name` - The name of the bazel module.
///
/// ### Returns
/// A String containing the bazel library version.
fn get_bazel_library_version(library_name: &str) -> String {
    // "bazel mod explain" simply lists the packages and their versions imported as bazel modules.
    let output: std::process::Output = std::process::Command::new("bazel")
        .arg("mod")
        .arg("explain")
        .output()
        .expect("Failed to execute 'bazel mod explain'");
    // Read from the stdout the output and look for the matching library name.
    let process_output: String = String::from_utf8_lossy(&output.stdout).to_string();
    let library_token: String = library_name.to_owned() + "@";
    let mut process_output_lines: std::str::Lines = process_output.lines();
    let line_index: usize = process_output_lines
        .position(|line| line.contains(&library_token))
        .expect("Couldn't find the library");
    let library_version: &str = process_output
        .lines()
        .nth(line_index)
        .expect("Failed to retrieve the line from the process output.")
        .split('@')
        .last()
        .expect("Failed to retrieve library version.")
        .trim(); // Remove trailing spaces.
    String::from(library_version)
}

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
        .arg(format!(
            "--symlink_prefix={}",
            bazel_output_base_dir.join("bazel-").display()
        ))
        .arg("//...")
        .status()
        .expect("Failed to generate build script");
    if code.code() != Some(0) {
        panic!("Failed to generate build script");
    }
    let bazel_bin_dir = bazel_output_base_dir.join("bazel-bin");

    let maliput_version = get_bazel_library_version("maliput");
    println!("Detected maliput version: <{}>", maliput_version);
    let maliput_bin_path = bazel_bin_dir
        .join("external")
        .join(format!("maliput~{}", maliput_version));
    let maliput_malidrive_version = get_bazel_library_version("maliput_malidrive");
    println!("Detected maliput_malidrive version: <{}>", maliput_malidrive_version);
    let maliput_malidrive_bin_path = bazel_bin_dir
        .join("external")
        .join(format!("maliput_malidrive~{}", maliput_malidrive_version));

    // ************* maliput header files ************* //

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

    // ************* maliput_malidrive header files ************* //
    // TODO(francocipollone): For consistency we should also add include paths for maliput_malidrive.

    // ************* maliput_malidrive resource files ************* //
    let maliput_malidrive_resource_path = bazel_bin_dir
        .join("libmaliput_sdk.so.runfiles")
        .join(String::from("maliput_malidrive~") + &maliput_malidrive_version)
        .join("resources");

    // ************* crate output env vars ************* //

    // Environment variable to pass down to this crate:
    println!("cargo:rustc-env=MALIPUT_BIN_PATH={}", maliput_bin_path.display());
    println!(
        "cargo:rustc-env=MALIPUT_MALIDRIVE_BIN_PATH={}",
        maliput_malidrive_bin_path.display()
    );
    println!(
        "cargo:rustc-env=MALIPUT_MALIDRIVE_PLUGIN_PATH={}",
        maliput_malidrive_bin_path.join("maliput_plugins").display()
    );
    println!(
        "cargo:rustc-env=MALIPUT_MALIDRIVE_RESOURCE_PATH={}",
        maliput_malidrive_resource_path.display()
    );

    // Environment variable to pass down to dependent crates:
    // See: https://doc.rust-lang.org/cargo/reference/build-scripts.html#the-links-manifest-key
    println!("cargo:root={}", out_dir.display()); //> Accessed as MALIPUT_SDK_ROOT
    println!("cargo:maliput_bin_path={}", maliput_bin_path.display()); //> Accessed as MALIPUT_SDK_MALIPUT_BIN_PATH
    println!(
        "cargo:maliput_malidrive_bin_path={}",
        maliput_malidrive_bin_path.display()
    ); //> Accessed as MALIPUT_SDK_MALIPUT_MALIDRIVE_BIN_PATH
    println!(
        "cargo:maliput_malidrive_plugin_path={}",
        maliput_malidrive_bin_path.join("maliput_plugins").display()
    ); //> Accessed as MALIPUT_SDK_MALIPUT_MALIDRIVE_PLUGIN_PATH

    Ok(())
}
