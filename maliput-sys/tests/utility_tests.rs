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

#[cfg(test)]
mod utility_test {
    mod generate_obj_file_test {
        use cxx::UniquePtr;
        use maliput_sys::api::ffi::RoadNetwork;
        use maliput_sys::plugin::ffi::CreateRoadNetwork;
        use maliput_sys::utility::ffi::Features;
        use maliput_sys::utility::ffi::Utility_GenerateObjFile;
        use std::fs::{create_dir, metadata, remove_dir_all};
        use std::path::PathBuf;

        fn setup_road_network() -> UniquePtr<RoadNetwork> {
            std::env::set_var("MALIPUT_PLUGIN_PATH", maliput_sdk::get_maliput_malidrive_plugin_path());
            let road_network_loader_id = String::from("maliput_malidrive");
            let package_location = env!("CARGO_MANIFEST_DIR");
            let xodr_path = format!("opendrive_file:{}/tests/resources/ArcLane.xodr", package_location);
            let road_network_properties = vec![xodr_path];

            CreateRoadNetwork(&road_network_loader_id, &road_network_properties)
        }

        #[test]
        fn generate_obj_file() {
            let obj_features = Features {
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
            let road_network = setup_road_network();
            let out_dir = std::env::temp_dir().join("maliput");
            if out_dir.exists() {
                let _ = remove_dir_all(&out_dir);
            }
            let _ = create_dir(&out_dir);

            let out_dir = out_dir.as_os_str().to_str().unwrap().to_string();
            let file_root = String::from("road_network");
            unsafe {
                Utility_GenerateObjFile(road_network.as_ref().unwrap(), &out_dir, &file_root, &obj_features);
            }
            let file_path = PathBuf::from(&out_dir).join(file_root + ".obj");
            let metadata = metadata(file_path).unwrap();

            // Check that the file is not empty.
            assert_ne!(metadata.len(), 0);

            let _ = remove_dir_all(out_dir);
        }
    }
}
