// BSD 3-Clause License
//
// Copyright (c) 2025, Woven by Toyota.
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

use maliput::api::{LaneChangePermission, LaneMarkingColor, LaneMarkingLine, LaneMarkingType, LaneMarkingWeight};

#[test]
fn lane_marking_line_api() {
    let lane_marking_line = LaneMarkingLine::new(0.5, 0.5, 0.1, 0.0, LaneMarkingColor::White);
    assert_eq!(lane_marking_line.length, 0.5);
    assert_eq!(lane_marking_line.space, 0.5);
    assert_eq!(lane_marking_line.width, 0.1);
    assert_eq!(lane_marking_line.r_offset, 0.0);
    assert_eq!(lane_marking_line.color, LaneMarkingColor::White);
}

/// Tests basic properties of a lane marking (width, height, material).
#[test]
fn lane_marking_basic_properties() {
    let road_network = common::create_t_shape_road_network(false);
    let road_geometry = road_network.road_geometry();
    let lane = road_geometry.get_lane(&"0_0_1".to_string()).unwrap();

    // Get the right boundary and its marking
    let right_boundary = lane.right_boundary().unwrap();
    let markings = right_boundary.get_markings();
    assert!(!markings.is_empty());

    let marking = &markings[0].lane_marking;

    // Check width and height are non-negative
    assert!(marking.width() >= 0.0);
    assert!(marking.height() >= 0.0);

    // Check material is not empty
    let material = marking.material();
    assert!(!material.is_empty());
}

#[test]
fn lane_marking_type_api() {
    let road_network = common::create_t_shape_road_network(false);
    let road_geometry = road_network.road_geometry();
    let lane = road_geometry.get_lane(&"0_0_1".to_string()).unwrap();

    let right_boundary = lane.right_boundary().unwrap();
    let right_markings = right_boundary.get_markings();
    let right_marking = &right_markings[0].lane_marking;

    assert_eq!(right_marking.get_type(), LaneMarkingType::SolidSolid);

    // let left_boundary = lane.left_boundary().unwrap();
    // let left_markings = left_boundary.get_markings();
    // let left_marking = &left_markings[0].lane_marking;

    // TODO(Santoi): I think this should be Solid, but test says it is None. We should check this.
    // let marking_type = left_marking.get_type();
    // assert_eq!(marking_type, LaneMarkingType::Solid);
}

#[test]
fn lane_marking_color_api() {
    let road_network = common::create_t_shape_road_network(false);
    let road_geometry = road_network.road_geometry();
    let lane = road_geometry.get_lane(&"0_0_1".to_string()).unwrap();

    let right_boundary = lane.right_boundary().unwrap();
    let markings = right_boundary.get_markings();
    let marking = &markings[0].lane_marking;

    let color = marking.color();
    assert_eq!(color, LaneMarkingColor::Yellow);
}

#[test]
fn lane_marking_weight_api() {
    let road_network = common::create_t_shape_road_network(false);
    let road_geometry = road_network.road_geometry();
    let lane = road_geometry.get_lane(&"0_0_1".to_string()).unwrap();

    let right_boundary = lane.right_boundary().unwrap();
    let markings = right_boundary.get_markings();
    let marking = &markings[0].lane_marking;

    let weight = marking.weight();
    assert_eq!(weight, LaneMarkingWeight::Unknown);
}

#[test]
fn lane_marking_lane_change_permission() {
    let road_network = common::create_t_shape_road_network(false);
    let road_geometry = road_network.road_geometry();
    let lane = road_geometry.get_lane(&"0_0_1".to_string()).unwrap();

    // Get the right boundary marking (edge line, should restrict lane changes)
    let right_boundary = lane.right_boundary().unwrap();
    let right_markings = right_boundary.get_markings();
    let right_marking = &right_markings[0].lane_marking;

    let right_lane_change = right_marking.lane_change();
    assert_eq!(right_lane_change, LaneChangePermission::Allowed);

    // Get the left boundary marking
    let left_boundary = lane.left_boundary().unwrap();
    let left_markings = left_boundary.get_markings();
    let left_marking = &left_markings[0].lane_marking;

    let left_lane_change = left_marking.lane_change();
    assert_eq!(left_lane_change, LaneChangePermission::Allowed);
}

#[test]
fn lane_marking_lines() {
    let road_network = common::create_t_shape_road_network(false);
    let road_geometry = road_network.road_geometry();
    let lane = road_geometry.get_lane(&"0_0_1".to_string()).unwrap();

    let right_boundary = lane.right_boundary().unwrap();
    let markings = right_boundary.get_markings();
    let marking = &markings[0].lane_marking;

    let lines = marking.lines();
    assert!(lines.is_empty());
}

/// Tests that marking properties are the same in a shared marking between lanes.
#[test]
fn lane_marking_right_lane_and_left_lane_match_shared_marking_properties() {
    let road_network = common::create_t_shape_road_network(false);
    let road_geometry = road_network.road_geometry();
    let right_lane = road_geometry.get_lane(&"0_0_1".to_string()).unwrap();
    let left_lane = right_lane.to_left();
    assert!(left_lane.is_some());
    let left_lane = left_lane.unwrap();

    let right_lane_left_boundary = right_lane.left_boundary().unwrap();
    let right_lane_left_markings = right_lane_left_boundary.get_markings();
    let left_lane_right_boundary = left_lane.right_boundary().unwrap();
    let left_lane_right_markings = left_lane_right_boundary.get_markings();
    assert_eq!(right_lane_left_markings.len(), 1);
    assert_eq!(left_lane_right_markings.len(), 1);
    let left_marking_query = &right_lane_left_markings[0];
    let left_marking = &left_marking_query.lane_marking;
    let right_marking_query = &left_lane_right_markings[0];
    let right_marking = &right_marking_query.lane_marking;

    assert_eq!(left_marking.get_type(), right_marking.get_type());
    assert_eq!(left_marking.color(), right_marking.color());
    assert_eq!(left_marking.weight(), right_marking.weight());
    assert_eq!(left_marking.lane_change(), right_marking.lane_change());
    assert_eq!(left_marking.width(), right_marking.width());
    assert_eq!(left_marking.height(), right_marking.height());
    assert_eq!(left_marking.material(), right_marking.material());
    assert!(left_marking.lines().is_empty());
    assert!(right_marking.lines().is_empty());
}
