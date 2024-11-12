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
use std::path::Path;

pub type ObjFeatures = maliput_sys::utility::ffi::Features;

/// Generates a Wavefront and a Material file from the `road_network`.
///
/// # Arguments
///
/// * `road_network` - The road network to generate the Wavefront file from.
/// * `dirpath` - The directory where the files will be created.
/// * `fileroot` - The base name of the files. This means without the extension.
/// * `obj_features` - Customization features for the Wavefront file.
///
/// # Details
/// These are written under the `dirpath` directory as `fileroot.obj` and `fileroot.mtl`.
/// In case `dirpath` doesn't exist, or if the files can't be created, a runtime error will occur.
pub fn generate_obj_file(
    road_network: &RoadNetwork,
    dirpath: impl AsRef<Path>,
    fileroot: impl AsRef<Path>,
    obj_features: &ObjFeatures,
) -> Result<(), Box<dyn Error>> {
    let dirpath = to_string(dirpath)?;
    let fileroot = to_string(fileroot)?;
    let raw_rn = road_network.rn.as_ref();
    if let Some(raw_rn) = raw_rn {
        unsafe {
            maliput_sys::utility::ffi::Utility_GenerateObjFile(raw_rn, &dirpath, &fileroot, obj_features);
        }
        Ok(())
    } else {
        Result::Err(Box::from("RoadNetwork is empty."))
    }
}

/// Obtain a Wavefront formatted String that describes `road_network`'s geometry.
///
/// # Arguments
///
/// * `road_network` - The road network to get the Wavefront description from.
/// * `obj_features` - Customization features for the Wavefront meshes.
///
/// # Returns
///
/// * A String containing the Wavefront description.
/// * A dynamic error if there was an issue processing the road network.
pub fn get_obj_description_from_road_network(
    road_network: &RoadNetwork,
    obj_features: &ObjFeatures,
) -> Result<String, Box<dyn Error>> {
    let output_directory = std::env::temp_dir().join("maliput");
    if !output_directory.exists() {
        let _ = create_dir(&output_directory);
    }
    let output_directory = to_string(output_directory)?;
    let file_name = String::from("road_network");
    generate_obj_file(road_network, &output_directory, &file_name, obj_features)?;
    let output_directory = Path::new(&output_directory);
    let obj_full_path = output_directory.join(create_file_name(file_name.as_str(), "obj"));
    let obj_description = read_to_string(&obj_full_path)?;
    remove_file(obj_full_path)?;
    let mtl_full_path = output_directory.join(create_file_name(file_name.as_str(), "mtl"));
    remove_file(mtl_full_path)?;
    Ok(obj_description)
}

/// Converts a `impl AsRef<Path>` to a `String` object.
fn to_string(path: impl AsRef<Path>) -> Result<String, &'static str> {
    Ok(path
        .as_ref()
        .as_os_str()
        .to_str()
        .ok_or("Failed to get output directory.")?
        .to_string())
}

/// Returns a `String` concatenating the `extension` to the `file_name`.
fn create_file_name(file_name: &str, extension: &str) -> String {
    format!("{}.{}", file_name, extension)
}
