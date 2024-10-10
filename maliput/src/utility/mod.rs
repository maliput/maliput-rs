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

use crate::api::RoadNetwork;
use std::error::Error;
use std::fs::{create_dir, read_to_string, remove_file};
use std::path::{Path, PathBuf};

pub type Features = maliput_sys::utility::ffi::Features;

/// Generates a Wavefront and a Material file from the `road_network`. These are written under the
/// `dirpath` directory as `fileroot.obj` and `fileroot.mtl`.
///
/// Fails if `dirpath` doesn't exist, or if the files can't be created.
pub fn generate_obj_file(road_network: &RoadNetwork, dirpath: &String, fileroot: &String, features: &Features) {
    unsafe {
        maliput_sys::utility::ffi::Utility_GenerateObjFile(
            road_network.rn.as_ref().map_or(std::ptr::null(), |ref_data| {
                ref_data as *const maliput_sys::utility::ffi::RoadNetwork
            }),
            dirpath,
            fileroot,
            features,
        );
    }
}

pub fn get_obj_description_from_road_network(
    road_network: &RoadNetwork,
    features: &Features,
) -> Result<String, Box<dyn Error>> {
    let output_directory = std::env::temp_dir().join("maliput");
    if !output_directory.exists() {
        let _ = create_dir(&output_directory);
    }
    let output_directory = path_to_string(output_directory)?;
    let file_name = String::from("road_network");
    generate_obj_file(road_network, &output_directory, &file_name, &features);
    let output_directory = Path::new(&output_directory);
    let obj_full_path = output_directory.join(create_file_name(file_name.as_str(), "obj"));
    let obj_description = read_to_string(&obj_full_path)?;
    remove_file(obj_full_path)?;
    let mtl_full_path = output_directory.join(create_file_name(file_name.as_str(), "mtl"));
    remove_file(mtl_full_path)?;
    Ok(obj_description)
}

/// Converts a `PathBuf` to a `String` object.
fn path_to_string(path: PathBuf) -> Result<String, &'static str> {
    Ok(path
        .as_os_str()
        .to_str()
        .ok_or("Failed to get output directory.")?
        .to_string())
}

/// Returns a `String` concatenating the `extension` to the `file_name`.
fn create_file_name(file_name: &str, extension: &str) -> String {
    format!("{}.{}", file_name, extension)
}
