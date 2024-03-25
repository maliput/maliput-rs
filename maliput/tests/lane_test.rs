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
mod common;

#[test]
fn lane_api() {
    let tolerance = 1e-10;
    let inertial_pos = maliput::api::InertialPosition::new(5.0, 1.75, 0.0);
    let expected_lane_id = String::from("0_0_1");
    let road_network = common::create_t_shape_road_network();
    let road_geometry = road_network.road_geometry();

    let road_position_result = road_geometry.to_road_position(&inertial_pos);
    assert_eq!(road_position_result.road_position.lane().id(), expected_lane_id);
    let lane = road_position_result.road_position.lane();
    let index = lane.index();
    assert_eq!(index, 1);
    let contains = lane.contains(&road_position_result.road_position.pos());
    assert!(contains);
    let lane_bounds = lane.lane_bounds(0.0);
    assert_eq!(lane_bounds.min(), -1.75);
    assert_eq!(lane_bounds.max(), 1.75);
    let segment_bounds = lane.segment_bounds(0.0);
    assert_eq!(segment_bounds.min(), -5.25);
    assert_eq!(segment_bounds.max(), 1.75);
    let elevation_bounds = lane.elevation_bounds(0.0, 0.0);
    assert_eq!(elevation_bounds.min(), 0.0);
    assert_eq!(elevation_bounds.max(), 5.0);
    let orientation = lane.get_orientation(&road_position_result.road_position.pos());
    assert_eq!(orientation.roll(), 0.0);
    assert_eq!(orientation.pitch(), 0.0);
    assert_eq!(orientation.yaw(), 0.0);
    let ret_inertial_position = lane.to_inertial_position(&road_position_result.road_position.pos());
    assert!((ret_inertial_position.x() - inertial_pos.x()).abs() < tolerance);
    assert!((ret_inertial_position.y() - inertial_pos.y()).abs() < tolerance);
    assert!((ret_inertial_position.z() - inertial_pos.z()).abs() < tolerance);
    let left_lane = lane.to_left();
    let right_lane = lane.to_right();
    // In TShapeRoad map there is no left lane from current lane.
    assert!(left_lane.is_none());
    // In TShapeRoad map there is a right lane from current lane.
    assert!(right_lane.is_some());
    assert_eq!(right_lane.unwrap().id(), "0_0_-1");
}
