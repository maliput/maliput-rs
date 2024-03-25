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
fn linear_tolerance() {
    let road_network = common::create_t_shape_road_network();
    let road_geometry = road_network.road_geometry();
    assert_eq!(road_geometry.linear_tolerance(), 0.01);
}

#[test]
fn to_road_position() {
    let expected_nearest_position = maliput::api::InertialPosition::new(5.0, 1.75, 0.0);
    let expected_lane_position = maliput::api::LanePosition::new(5.0, 0.0, 0.0);
    let expected_lane_id = String::from("0_0_1");
    let road_network = common::create_t_shape_road_network();
    let road_geometry = road_network.road_geometry();

    let road_position_result = road_geometry.to_road_position(&expected_nearest_position);
    assert_eq!(road_position_result.road_position.lane().id(), expected_lane_id);
    common::assert_lane_position_equality(
        &road_position_result.road_position.pos(),
        &expected_lane_position,
        road_geometry.linear_tolerance(),
    );
    common::assert_inertial_position_equality(
        &road_position_result.nearest_position,
        &expected_nearest_position,
        road_geometry.linear_tolerance(),
    );
}

#[test]
fn by_index() {
    let road_network = common::create_t_shape_road_network();
    let road_geometry = road_network.road_geometry();
    let lane_id = String::from("0_0_1");
    let lane = road_geometry.get_lane(&lane_id);
    assert_eq!(lane.id(), "0_0_1");

    let lanes = road_geometry.get_lanes();
    assert_eq!(lanes.len(), 12);
    let lanes = road_geometry.get_lanes();
    assert_eq!(lanes.len(), 12);
    let lanes = road_geometry.get_lanes();
    assert_eq!(lanes.len(), 12);
}
