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

use maliput_sys::api::ffi::RoadNetwork;
use std::error::Error;
use std::fs::{read_to_string, remove_file};
use std::path::Path;

type Features = maliput_sys::utility::ffi::Features;

// Generate a mesh obj file from the road network.
pub fn generate_obj_file(road_network: &RoadNetwork, dirpath: &String, fileroot: &String, features: &Features) {
    unsafe {
        maliput_sys::utility::ffi::Utility_GenerateObjFile(road_network, dirpath, fileroot, features);
    }
}

pub fn get_obj_description_from_road_network(
    road_network: &RoadNetwork,
    output_directory: String,
) -> Result<String, Box<dyn Error>> {
    let file_name = String::from("road_network");
    let features = Features {
        max_grid_unit: 1.0,
        min_grid_resolution: 5.0,
        draw_stripes: true,
        draw_arrows: true,
        draw_lane_haze: true,
        draw_branch_points: true,
        draw_elevation_bounds: true,
        off_grid_mesh_generation: false,
        simplify_mesh_threshold: 0.,
        stripe_width: 0.25,
        stripe_elevation: 0.05,
        arrow_elevation: 0.05,
        lane_haze_elevation: 0.02,
        branch_point_elevation: 0.5,
        branch_point_height: 0.5,
        origin: [0.; 3],
        highlighted_segments: Vec::new(),
    };
    generate_obj_file(road_network, &output_directory, &file_name, &features);
    let full_path = Path::new(&output_directory);
    let full_path = full_path.join(file_name + ".obj");
    let obj_description = read_to_string(&full_path)?;
    let _ = remove_file(full_path);
    Ok(obj_description)
}
