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

use maliput::{api::RoadNetwork, ResourceManager};
use std::collections::HashMap;

#[allow(dead_code)]
pub fn create_t_shape_road_network(omit_non_drivable_lanes: bool) -> RoadNetwork {
    // Get location of odr resources
    let package_location = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let xodr_path = format!("{}/data/xodr/TShapeRoad.xodr", package_location);

    let omit_non_drivable_lanes = match omit_non_drivable_lanes {
        false => "false",
        true => "true",
    };

    let road_network_properties = HashMap::from([
        ("road_geometry_id", "my_rg_from_rust"),
        ("opendrive_file", xodr_path.as_str()),
        ("linear_tolerance", "0.01"),
        ("omit_nondrivable_lanes", omit_non_drivable_lanes),
    ]);
    let rn_res = RoadNetwork::new("maliput_malidrive", &road_network_properties);
    assert!(
        rn_res.is_ok(),
        "Expected RoadNetwork to be created successfully with TShapeRoad.xodr"
    );
    rn_res.unwrap()
}

#[allow(dead_code)]
pub fn create_arc_lane_road_network() -> RoadNetwork {
    // Get location of odr resources
    let package_location = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let xodr_path = format!("{}/data/xodr/ArcLane.xodr", package_location);

    let road_network_properties = HashMap::from([
        ("road_geometry_id", "my_rg_from_rust"),
        ("opendrive_file", xodr_path.as_str()),
        ("linear_tolerance", "0.01"),
    ]);
    let rn_res = RoadNetwork::new("maliput_malidrive", &road_network_properties);
    assert!(
        rn_res.is_ok(),
        "Expected RoadNetwork to be created successfully with ArcLane.xodr"
    );
    rn_res.unwrap()
}

#[allow(dead_code)]
pub fn create_town_01_road_network() -> RoadNetwork {
    // Get location of odr resources
    let package_location = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let xodr_path = format!("{}/data/xodr/Town01.xodr", package_location);

    let road_network_properties = HashMap::from([
        ("road_geometry_id", "my_rg_from_rust"),
        ("opendrive_file", xodr_path.as_str()),
        ("linear_tolerance", "0.01"),
    ]);
    let rn_res = RoadNetwork::new("maliput_malidrive", &road_network_properties);
    assert!(
        rn_res.is_ok(),
        "Expected RoadNetwork to be created successfully with Town01.xodr"
    );
    rn_res.unwrap()
}

#[allow(dead_code)]
pub fn create_t_shape_road_network_with_books() -> RoadNetwork {
    let rm = ResourceManager::new();
    let t_shape_xodr_path = rm
        .get_resource_path_by_name("maliput_malidrive", "TShapeRoad.xodr")
        .unwrap();
    let t_shape_books_path = rm
        .get_resource_path_by_name("maliput_malidrive", "TShapeRoad.yaml")
        .unwrap();

    let road_network_properties = HashMap::from([
        ("road_geometry_id", "my_rg_from_rust"),
        ("opendrive_file", t_shape_xodr_path.to_str().unwrap()),
        ("road_rule_book", t_shape_books_path.to_str().unwrap()),
        ("traffic_light_book", t_shape_books_path.to_str().unwrap()),
        ("phase_ring_book", t_shape_books_path.to_str().unwrap()),
        ("intersection_book", t_shape_books_path.to_str().unwrap()),
        ("linear_tolerance", "0.01"),
    ]);
    let rn_res = RoadNetwork::new("maliput_malidrive", &road_network_properties);
    assert!(
        rn_res.is_ok(),
        "Expected RoadNetwork to be created successfully with TShapeRoad.xodr and books"
    );
    rn_res.unwrap()
}

#[allow(dead_code)]
pub fn create_loop_road_pedestrian_crosswalk_road_network_with_books() -> RoadNetwork {
    let rm = ResourceManager::new();
    let loop_road_pedestrian_crosswalk_xodr_path = rm
        .get_resource_path_by_name("maliput_malidrive", "LoopRoadPedestrianCrosswalk.xodr")
        .unwrap();
    let loop_road_pedestrian_crosswalk_books_path = rm
        .get_resource_path_by_name("maliput_malidrive", "LoopRoadPedestrianCrosswalk.yaml")
        .unwrap();

    let road_network_properties = HashMap::from([
        ("road_geometry_id", "my_rg_from_rust"),
        (
            "opendrive_file",
            loop_road_pedestrian_crosswalk_xodr_path.to_str().unwrap(),
        ),
        (
            "road_rule_book",
            loop_road_pedestrian_crosswalk_books_path.to_str().unwrap(),
        ),
        (
            "rule_registry",
            loop_road_pedestrian_crosswalk_books_path.to_str().unwrap(),
        ),
        (
            "traffic_light_book",
            loop_road_pedestrian_crosswalk_books_path.to_str().unwrap(),
        ),
        (
            "phase_ring_book",
            loop_road_pedestrian_crosswalk_books_path.to_str().unwrap(),
        ),
        (
            "intersection_book",
            loop_road_pedestrian_crosswalk_books_path.to_str().unwrap(),
        ),
        ("linear_tolerance", "0.01"),
    ]);
    let rn_res = RoadNetwork::new("maliput_malidrive", &road_network_properties);
    assert!(
        rn_res.is_ok(),
        "Expected RoadNetwork to be created successfully with LoopRoadPedestrianCrosswalk.xodr and books"
    );
    rn_res.unwrap()
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
