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
fn id() {
    let road_network = common::create_t_shape_road_network();
    let road_geometry = road_network.road_geometry();
    assert_eq!(road_geometry.id(), "my_rg_from_rust");
}

#[test]
fn tolerances() {
    let road_network = common::create_t_shape_road_network();
    let road_geometry = road_network.road_geometry();
    assert_eq!(road_geometry.linear_tolerance(), 0.01);
    assert_eq!(road_geometry.angular_tolerance(), 0.01);
}

#[test]
fn to_road_position() {
    let expected_nearest_position = maliput::api::InertialPosition::new(5.0, 1.75, 0.0);
    let expected_lane_position = maliput::api::LanePosition::new(5.0, 0.0, 0.0);
    let expected_lane_id = String::from("0_0_1");
    let road_network = common::create_t_shape_road_network();
    let road_geometry = road_network.road_geometry();

    let road_position_result = road_geometry.to_road_position(&expected_nearest_position).unwrap();
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
    assert!(lane.is_some());
    assert_eq!(lane.unwrap().id(), "0_0_1");

    let lanes = road_geometry.get_lanes();
    assert_eq!(lanes.len(), 12);
    let lanes = road_geometry.get_lanes();
    assert_eq!(lanes.len(), 12);
    let lanes = road_geometry.get_lanes();
    assert_eq!(lanes.len(), 12);

    let segment_id = String::from("0_0");
    let segment = road_geometry.get_segment(&segment_id).unwrap();
    assert_eq!(segment.id(), "0_0");

    let junction_id = String::from("0_0");
    let junction = road_geometry.get_junction(&junction_id).unwrap();
    assert_eq!(junction.id(), "0_0");
}

#[test]
fn backend_custom_command() {
    let road_network = common::create_arc_lane_road_network();
    let road_geometry = road_network.road_geometry();

    let command = String::from("OpenScenarioLanePositionToMaliputRoadPosition,1,50,-1,0.");
    let result = road_geometry.backend_custom_command(&command).unwrap();
    assert_eq!(result, "1_0_-1,51.250000,0.000000,0.000000");
    let command = String::from("OpenScenarioRoadPositionToMaliputRoadPosition,1,50,0.");
    let result = road_geometry.backend_custom_command(&command).unwrap();
    assert_eq!(result, "1_0_-1,51.250000,1.000000,0.000000");
    let command = String::from("MaliputRoadPositionToOpenScenarioLanePosition,1_0_-1,51.250000,0.000000,0.000000");
    let result = road_geometry.backend_custom_command(&command).unwrap();
    assert_eq!(result, "1,50.000000,-1,0.000000");
    let command = String::from("MaliputRoadPositionToOpenScenarioRoadPosition,1_0_-1,51.250000,1.000000,0.000000");
    let result = road_geometry.backend_custom_command(&command).unwrap();
    assert_eq!(result, "1,50.000000,0.000000");
    let command = String::from("OpenScenarioRelativeRoadPositionToMaliputRoadPosition,1,0.,1.,50.,1.");
    let result = road_geometry.backend_custom_command(&command).unwrap();
    assert_eq!(result, "1_0_1,48.750000,1.000000,0.000000");
    let command = String::from("OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition,1,1,0.,-1,50.,-0.8");
    let result = road_geometry.backend_custom_command(&command).unwrap();
    assert_eq!(result, "1_0_-1,48.750000,-0.800000,0.000000");
    let command = String::from("OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition,1,-1,0.,1,50.,0.8");
    let result = road_geometry.backend_custom_command(&command).unwrap();
    assert_eq!(result, "1_0_1,50.000000,0.800000,0.000000");
    let command = String::from("GetRoadOrientationAtOpenScenarioRoadPosition,1,50.,0.");
    let result = road_geometry.backend_custom_command(&command).unwrap();
    assert_eq!(result, "0.000000,-0.000000,1.250000");
    let invalid_command = String::from("InvalidCommand");
    let result = road_geometry.backend_custom_command(&invalid_command);
    assert!(result.is_err());
}

#[test]
fn geo_reference_info() {
    let road_network = common::create_town_01_road_network();
    let road_geometry = road_network.road_geometry();
    let expected_geo_ref = String::from("+lat_0=4.9000000000000000e+1 +lon_0=8.0000000000000000e+0");
    let actual_geo_ref = road_geometry.geo_reference_info();
    assert_eq!(actual_geo_ref, expected_geo_ref);
}

#[test]
fn geo_reference_info_empty() {
    let road_network = common::create_arc_lane_road_network();
    let road_geometry = road_network.road_geometry();
    let expected_geo_ref = String::from("");
    let actual_geo_ref = road_geometry.geo_reference_info();
    assert_eq!(actual_geo_ref, expected_geo_ref);
}
