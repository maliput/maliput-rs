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
fn test_intersection_book_api() {
    let road_network = common::create_t_shape_road_network_with_books();
    let intersection_book = road_network.intersection_book();
    let intersections = intersection_book.get_intersections();
    assert!(!intersections.is_empty());
    assert_eq!(intersections.len(), 1);
    assert_eq!(intersections[0].id(), "TIntersection");

    let intersection = intersection_book.get_intersection("TIntersection");
    assert!(intersection.is_some());
    assert_eq!(intersection.unwrap().id(), "TIntersection");

    let intersection = intersection_book.get_intersection("Invalid Intersection");
    assert!(intersection.is_none());
}

#[test]
fn test_intersection_book_find_intersection_api() {
    use maliput::api::{InertialPosition, LanePosition};

    let road_network = common::create_t_shape_road_network_with_books();
    let intersection_book = road_network.intersection_book();
    let intersection = intersection_book.get_intersection("TIntersection").unwrap();

    // Test find_intersection_with_traffic_light()
    let bulb_ids = intersection.bulb_ids();
    assert!(!bulb_ids.is_empty());
    let traffic_light_id = bulb_ids[0].traffic_light_id();
    let found_intersection = intersection_book.find_intersection_with_traffic_light(&traffic_light_id);
    assert!(found_intersection.is_some());
    assert_eq!(found_intersection.unwrap().id(), "TIntersection");
    let found_intersection = intersection_book.find_intersection_with_traffic_light("EastFacing");
    assert!(found_intersection.is_some());
    assert_eq!(found_intersection.unwrap().id(), "TIntersection");
    let found_intersection = intersection_book.find_intersection_with_traffic_light("InvalidId");
    assert!(found_intersection.is_none());

    // Test find_intersection_with_discrete_value_rule()
    let discrete_value_rule_states = intersection.discrete_value_rule_states();
    assert!(!discrete_value_rule_states.is_empty());
    let rule_id = &discrete_value_rule_states[0].rule_id;
    let found_intersection = intersection_book.find_intersection_with_discrete_value_rule(rule_id);
    assert!(found_intersection.is_some());
    assert_eq!(found_intersection.unwrap().id(), "TIntersection");
    let found_intersection =
        intersection_book.find_intersection_with_discrete_value_rule("Right-Of-Way Rule Type/EastApproach");
    assert!(found_intersection.is_some());
    assert_eq!(found_intersection.unwrap().id(), "TIntersection");
    let found_intersection = intersection_book.find_intersection_with_discrete_value_rule("InvalidId");
    assert!(found_intersection.is_none());

    // Test find_intersection_with_inertial_position()
    let road_geometry = road_network.road_geometry();
    let intersection_lanes_s_ranges = intersection.region();
    let lane_s_range = &intersection_lanes_s_ranges[0];
    let lane = road_geometry.get_lane(&lane_s_range.lane_id()).unwrap();
    let s_range = lane_s_range.s_range();
    let lane_pos = LanePosition::new((s_range.s0() + s_range.s1()) / 2., 0., 0.);
    let inertial_pos = lane.to_inertial_position(&lane_pos).unwrap();
    let found_intersection = intersection_book.find_intersection_with_inertial_position(&inertial_pos);
    assert!(found_intersection.is_some());
    assert_eq!(found_intersection.unwrap().id(), "TIntersection");
    let inertial_pos_outside = InertialPosition::new(1e6, 1e6, 1e6);
    let found_intersection = intersection_book.find_intersection_with_inertial_position(&inertial_pos_outside);
    assert!(found_intersection.is_none());
}
