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
#![allow(rustdoc::bare_urls)]
#![doc = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/README.md"))]

use std::path::PathBuf;

/// Returns a map of libraries here vendored and the directories to search for the binaries.
pub fn sdk_libraries() -> Vec<(String, PathBuf)> {
    vec![
        ("maliput".to_string(), PathBuf::from(env!("MALIPUT_BIN_PATH"))),
        (
            "maliput_malidrive".to_string(),
            PathBuf::from(env!("MALIPUT_MALIDRIVE_BIN_PATH")),
        ),
        (
            "maliput_geopackage".to_string(),
            PathBuf::from(env!("MALIPUT_GEOPACKAGE_BIN_PATH")),
        ),
    ]
}

/// Returns a map of resources here vendored and the directories to search for the resources.
pub fn sdk_resources() -> Vec<(String, PathBuf)> {
    vec![(
        "maliput_malidrive".to_string(),
        PathBuf::from(env!("MALIPUT_MALIDRIVE_RESOURCE_PATH")),
    )]
}

/// Returns the path to the maliput_malidrive plugin.
pub fn get_maliput_malidrive_plugin_path() -> PathBuf {
    PathBuf::from(env!("MALIPUT_MALIDRIVE_PLUGIN_PATH"))
}

/// Returns the path to the maliput_geopackage plugin.
pub fn get_maliput_geopackage_plugin_path() -> PathBuf {
    PathBuf::from(env!("MALIPUT_GEOPACKAGE_PLUGIN_PATH"))
}
