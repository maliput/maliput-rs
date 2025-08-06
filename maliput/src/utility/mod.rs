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
use crate::common::MaliputError;
use std::error::Error;
use std::fs::{create_dir_all, read_to_string, remove_file};
use std::path::{Path, PathBuf};

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
/// # Returns
/// A `PathBuf` object containing the path to the generated Wavefront file.
///
/// # Details
/// These are written under the `dirpath` directory as `fileroot.obj` and `fileroot.mtl`.
/// In case `dirpath` doesn't exist, or if the files can't be created, an error is returned.
pub fn generate_obj_file(
    road_network: &RoadNetwork,
    dirpath: impl AsRef<Path>,
    fileroot: impl AsRef<str>,
    obj_features: &ObjFeatures,
) -> Result<PathBuf, MaliputError> {
    // Saves the complete path to the generated Wavefront file.
    let future_obj_file_path = dirpath.as_ref().join(fileroot.as_ref().to_string() + ".obj");
    let dirpath = to_string(dirpath).map_err(|e| MaliputError::ObjCreationError(e.to_string()))?;
    // Creates dirpath if does not exist.
    create_dir_all(&dirpath).map_err(|e| MaliputError::ObjCreationError(e.to_string()))?;
    let raw_rn = road_network.rn.as_ref();
    if let Some(raw_rn) = raw_rn {
        unsafe {
            maliput_sys::utility::ffi::Utility_GenerateObjFile(
                raw_rn,
                &dirpath,
                &fileroot.as_ref().to_string(),
                obj_features,
            )
            .map_err(|e| MaliputError::ObjCreationError(e.to_string()))?;
        }
        // Verify if the file was created.
        if future_obj_file_path.is_file() && future_obj_file_path.with_extension("mtl").is_file() {
            Ok(future_obj_file_path)
        } else {
            Result::Err(MaliputError::ObjCreationError(String::from(
                "Failed to generate the Wavefront files.",
            )))
        }
    } else {
        Result::Err(MaliputError::ObjCreationError(String::from("RoadNetwork is empty.")))
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
    create_dir_all(&output_directory)?;
    let file_name = String::from("road_network");
    let path_to_obj_file = generate_obj_file(road_network, &output_directory, &file_name, obj_features)?;
    let obj_description = read_to_string(&path_to_obj_file)?;
    remove_file(path_to_obj_file.clone())?;
    let mtl_full_path = path_to_obj_file.with_extension("mtl");
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
