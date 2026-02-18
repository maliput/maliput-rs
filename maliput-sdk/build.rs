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
use std::path::{Path, PathBuf};
use std::process::{Command, Output};

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
    let output: Output = Command::new("bazel")
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
    let mut library_version: &str = process_output
        .lines()
        .nth(line_index)
        .expect("Failed to retrieve the line from the process output.")
        .split('@')
        .next_back()
        .expect("Failed to retrieve library version.")
        .trim(); // Remove trailing spaces.
    if library_version == "_" {
        // Support the case when "local_path_override" is used for the modules.
        library_version = "override";
    }
    String::from(library_version)
}

/// Resolve Bazel output path for an external dependency that may be versioned with `~version` or suffixed with `+`.
///
/// # Arguments
/// - `bazel_bin_dir`: Path to `bazel-bin` (e.g., from `$(bazel info bazel-bin)`).
/// - `subdir`: Subdirectory like "external" or "lib" where the package is located.
/// - `crate_name`: Name of the external Bazel dependency.
/// - `version`: Version string, used for `~version` fallback.
/// - `postfix_path`: Optional extra subpath to append after the package name (e.g., "resources").
///
/// # Returns
/// A full path to the resolved location.
///
/// # Panics
/// Panics if neither format is found.
fn resolve_bazel_package_path(
    bazel_bin_dir: &Path,
    subdir: &str,
    crate_name: &str,
    version: &str,
    postfix_path: Option<&str>,
) -> PathBuf {
    let base_path = bazel_bin_dir.join(subdir);

    let try_path = |suffix: String| {
        let mut path = base_path.join(suffix);
        if let Some(postfix) = postfix_path {
            path = path.join(postfix);
        }
        path
    };

    let plus_path = try_path(format!("{}+", crate_name));
    if plus_path.exists() {
        return plus_path;
    }

    let tilde_path = try_path(format!("{}~{}", crate_name, version));
    if tilde_path.exists() {
        return tilde_path;
    }

    panic!(
        "Could not resolve Bazel path for `{}` using either '+' or '~{}'",
        crate_name, version
    );
}

fn main() -> Result<(), Box<dyn Error>> {
    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=.bazelrc");
    println!("cargo:rerun-if-changed=BUILD.bazel");
    println!("cargo:rerun-if-changed=MODULE.bazel");
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let bazel_output_base_dir = out_dir.join("bazel_output_base");
    fs::create_dir_all(&bazel_output_base_dir)?;

    // Propagate feature flags as cfg for use in lib.rs
    let malidrive_enabled = env::var("CARGO_FEATURE_MALIPUT_MALIDRIVE").is_ok();
    let geopackage_enabled = env::var("CARGO_FEATURE_MALIPUT_GEOPACKAGE").is_ok();

    // Forward number of jobs used in cargo build execution to the bazel build.
    // NUM_JOBS env var is set by cargo to the number of jobs used in the build.
    let jobs = env::var("NUM_JOBS").unwrap().to_string();
    // Bazel build
    let code = Command::new("bazel")
        .arg(format!("--output_base={}", bazel_output_base_dir.display()))
        .arg("build")
        .arg(format!(
            "--symlink_prefix={}",
            bazel_output_base_dir.join("bazel-").display()
        ))
        .arg("--macos_minimum_os=10.15")
        .arg(format!("--jobs={}", jobs))
        .arg("//...")
        .status()
        .expect("Failed to generate build script");
    if code.code() != Some(0) {
        panic!("Failed to generate build script");
    }
    let bazel_bin_dir = bazel_output_base_dir.join("bazel-bin");

    let maliput_version = get_bazel_library_version("maliput");
    println!("cargo:info=maliput version: <{}>", maliput_version);
    let maliput_bin_path = resolve_bazel_package_path(&bazel_bin_dir, "external", "maliput", &maliput_version, None);

    // ---- maliput_malidrive (conditional on feature) ----
    let maliput_malidrive_bin_path = if malidrive_enabled {
        let maliput_malidrive_version = get_bazel_library_version("maliput_malidrive");
        println!("cargo:info=maliput_malidrive version: <{}>", maliput_malidrive_version);
        let path = resolve_bazel_package_path(
            &bazel_bin_dir,
            "external",
            "maliput_malidrive",
            &maliput_malidrive_version,
            None,
        );
        println!("cargo:info=maliput_malidrive_bin_path: {:?}", path);
        Some((path, maliput_malidrive_version))
    } else {
        None
    };

    // ---- maliput_geopackage (conditional on feature) ----
    let maliput_geopackage_bin_path = if geopackage_enabled {
        let maliput_geopackage_version = get_bazel_library_version("maliput_geopackage");
        println!(
            "cargo:info=maliput_geopackage version: <{}>",
            maliput_geopackage_version
        );
        let path = resolve_bazel_package_path(
            &bazel_bin_dir,
            "external",
            "maliput_geopackage",
            &maliput_geopackage_version,
            None,
        );
        println!("cargo:info=maliput_geopackage_bin_path: {:?}", path);
        Some(path)
    } else {
        None
    };
    // ************* maliput header files ************* //

    //---Header files---
    let virtual_includes_path = maliput_bin_path.join("_virtual_includes");
    println!("{:?}", virtual_includes_path);
    let mut virtual_includes = Vec::new();
    for entry in fs::read_dir(virtual_includes_path)? {
        println!("{:?}", entry);
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
    if let Some((ref malidrive_path, ref malidrive_version)) = maliput_malidrive_bin_path {
        // Set library file name based on OS
        let lib_dir = if cfg!(target_os = "macos") {
            "libmaliput_malidrive_dummy.dylib.runfiles"
        } else if cfg!(target_os = "linux") {
            "libmaliput_malidrive_dummy.so.runfiles"
        } else {
            "maliput_malidrive_dummy.dll.runfiles"
        };

        let maliput_malidrive_resource_path = resolve_bazel_package_path(
            &bazel_bin_dir,
            lib_dir,
            "maliput_malidrive",
            malidrive_version,
            Some("resources"),
        );

        println!(
            "cargo:rustc-env=MALIPUT_MALIDRIVE_BIN_PATH={}",
            malidrive_path.display()
        );
        println!(
            "cargo:rustc-env=MALIPUT_MALIDRIVE_PLUGIN_PATH={}",
            malidrive_path.join("maliput_plugins").display()
        );
        println!(
            "cargo:rustc-env=MALIPUT_MALIDRIVE_RESOURCE_PATH={}",
            maliput_malidrive_resource_path.display()
        );
        println!("cargo:maliput_malidrive_bin_path={}", malidrive_path.display()); //> Accessed as DEP_MALIPUT_SDK_MALIPUT_MALIDRIVE_BIN_PATH
        println!(
            "cargo:maliput_malidrive_plugin_path={}",
            malidrive_path.join("maliput_plugins").display()
        ); //> Accessed as DEP_MALIPUT_SDK_MALIPUT_MALIDRIVE_PLUGIN_PATH
    }

    // ************* maliput_geopackage env vars ************* //
    if let Some(ref geopackage_path) = maliput_geopackage_bin_path {
        println!(
            "cargo:rustc-env=MALIPUT_GEOPACKAGE_BIN_PATH={}",
            geopackage_path.display()
        );
        println!(
            "cargo:rustc-env=MALIPUT_GEOPACKAGE_PLUGIN_PATH={}",
            geopackage_path.join("maliput_plugins").display()
        );
        println!("cargo:maliput_geopackage_bin_path={}", geopackage_path.display()); //> Accessed as DEP_MALIPUT_SDK_MALIPUT_GEOPACKAGE_BIN_PATH
        println!(
            "cargo:maliput_geopackage_plugin_path={}",
            geopackage_path.join("maliput_plugins").display()
        ); //> Accessed as DEP_MALIPUT_SDK_MALIPUT_GEOPACKAGE_PLUGIN_PATH
    }

    // ************* crate output env vars ************* //

    // Environment variable to pass down to this crate:
    println!("cargo:rustc-env=MALIPUT_BIN_PATH={}", maliput_bin_path.display());

    // Environment variable to pass down to dependent crates:
    // See: https://doc.rust-lang.org/cargo/reference/build-scripts.html#the-links-manifest-key
    println!("cargo:root={}", out_dir.display()); //> Accessed as DEP_MALIPUT_SDK_ROOT
    println!("cargo:bin_path={}", bazel_bin_dir.display()); //> Accessed as DEP_MALIPUT_SDK_BIN_PATH
    println!("cargo:maliput_bin_path={}", maliput_bin_path.display()); //> Accessed as DEP_MALIPUT_SDK_MALIPUT_BIN_PATH

    Ok(())
}
