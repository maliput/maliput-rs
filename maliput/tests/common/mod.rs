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

use maliput::api::RoadNetwork;
use std::collections::HashMap;

pub fn create_t_shape_road_network() -> RoadNetwork {
    // Set MALIPUT_PLUGIN_PATH
    let maliput_malidrive_plugin_path = maliput_sdk::get_maliput_malidrive_plugin_path();
    std::env::set_var("MALIPUT_PLUGIN_PATH", maliput_malidrive_plugin_path);

    // Get location of odr resources
    let package_location = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let xodr_path = format!("{}/data/xodr/TShapeRoad.xodr", package_location);

    let road_network_properties = HashMap::from([
        ("road_geometry_id", "my_rg_from_rust"),
        ("opendrive_file", xodr_path.as_str()),
        ("linear_tolerance", "0.01"),
    ]);
    RoadNetwork::new("maliput_malidrive", &road_network_properties)
}

#[allow(dead_code)]
pub fn assert_inertial_position_equality(
    left: &maliput::api::InertialPosition,
    right: &maliput::api::InertialPosition,
    tolerance: f64,
) {
    assert!((left.x() - right.x()).abs() < tolerance);
    assert!((left.y() - right.y()).abs() < tolerance);
    assert!((left.z() - right.z()).abs() < tolerance);
}

#[allow(dead_code)]
pub fn assert_lane_position_equality(
    left: &maliput::api::LanePosition,
    right: &maliput::api::LanePosition,
    tolerance: f64,
) {
    assert!((left.s() - right.s()).abs() < tolerance);
    assert!((left.r() - right.r()).abs() < tolerance);
    assert!((left.h() - right.h()).abs() < tolerance);
}
