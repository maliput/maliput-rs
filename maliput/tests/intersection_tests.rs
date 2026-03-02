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

use maliput::api::{rules::BulbState, InertialPosition, LanePosition};

#[test]
fn intersection_api() {
    let road_network = common::create_t_shape_road_network_with_books();
    let expected_intersection_id = String::from("TIntersection");

    let road_geometry = road_network.road_geometry();
    assert_eq!(road_geometry.id(), "my_rg_from_rust");

    let book = road_network.intersection_book();
    let intersections = book.get_intersections();
    assert_eq!(intersections.len(), 1);
    intersections.iter().for_each(|intersection| {
        assert_eq!(intersection.id(), expected_intersection_id);
    });

    let intersection = book.get_intersection(&expected_intersection_id);
    assert!(intersection.is_some());
    assert_eq!(intersection.as_ref().expect("").id(), expected_intersection_id);

    let wrong_intersection = book.get_intersection("wrong_id");
    assert!(wrong_intersection.is_none());

    // Test methods of the Intersection struct.
    let intersection = intersection.expect("");

    // Test phase()
    let phase_query = intersection.phase();
    assert_eq!(phase_query.state, "AllGo");
    assert!(phase_query.next.is_some());
    let next_phase = phase_query.next.unwrap();
    assert_eq!(next_phase.next_state, "AllStop");
    assert!(next_phase.duration_until.is_some());
    assert_eq!(next_phase.duration_until.unwrap(), 45.0);

    // Test region()
    let region = intersection.region();
    assert_eq!(region.len(), 2);
    let mut region_lane_ids: Vec<String> = region.iter().map(|r| r.lane_id()).collect();
    region_lane_ids.sort();
    assert_eq!(region_lane_ids, vec!["0_0_-1", "1_0_1"]);

    // Test phase_ring_id()
    assert_eq!(intersection.phase_ring_id(), "TIntersectionPhaseRing");

    // Test bulb_ids()
    let bulb_ids = intersection.bulb_ids();
    assert!(!bulb_ids.is_empty());

    // Test get_bulb_state()
    let bulb_state = intersection.get_bulb_state(&bulb_ids[0]);
    assert!(bulb_state.is_some());
    assert_eq!(bulb_state.unwrap(), BulbState::Off);

    // Test discrete_value_rule_states()
    let discrete_value_rule_states = intersection.discrete_value_rule_states();
    assert!(!discrete_value_rule_states.is_empty());

    // Test includes_traffic_light()
    let traffic_light_id = bulb_ids[0].traffic_light_id();
    assert!(intersection.includes_traffic_light(&traffic_light_id));
    assert!(intersection.includes_traffic_light("EastFacing"));
    assert!(!intersection.includes_traffic_light("InvalidTrafficLightId"));

    // Test includes_discrete_value_rule()
    let rule_id = &discrete_value_rule_states[0].rule_id;
    assert!(intersection.includes_discrete_value_rule(rule_id));
    assert!(intersection.includes_discrete_value_rule("Right-Of-Way Rule Type/EastApproach"));
    assert!(!intersection.includes_discrete_value_rule("InvalidRuleId"));

    // Test includes_inertial_position()
    let road_geometry = road_network.road_geometry();
    let intersection_lanes_s_ranges = intersection.region();
    let lane_s_range = &intersection_lanes_s_ranges[0];
    let lane = road_geometry.get_lane(&lane_s_range.lane_id()).unwrap();
    let s_range = lane_s_range.s_range();
    let lane_pos = LanePosition::new((s_range.s0() + s_range.s1()) / 2., 0., 0.);
    let inertial_pos_inside = lane.to_inertial_position(&lane_pos).unwrap();
    assert!(intersection.includes_inertial_position(&inertial_pos_inside, &road_geometry));

    let inertial_pos_outside = InertialPosition::new(1e6, 1e6, 1e6);
    assert!(!intersection.includes_inertial_position(&inertial_pos_outside, &road_geometry));

    // Test with a position just outside the region but within tolerance.
    // Get a point at the end of the region.
    let lane_pos_end = LanePosition::new(s_range.s1(), 0., 0.);
    let inertial_pos_end = lane.to_inertial_position(&lane_pos_end).unwrap();
    // Create a point slightly outside by moving along the lane's orientation.
    let orientation = lane.get_orientation(&lane_pos_end).unwrap();
    let forward_vector = orientation.apply(&InertialPosition::new(1., 0., 0.));
    let slightly_outside_pos =
        inertial_pos_end.clone() + (forward_vector.clone() * (road_geometry.linear_tolerance() / 2.));
    assert!(intersection.includes_inertial_position(&slightly_outside_pos, &road_geometry));

    // Test with a position just outside the region and outside tolerance.
    let far_outside_pos = inertial_pos_end + (forward_vector * (road_geometry.linear_tolerance() * 2.));
    assert!(!intersection.includes_inertial_position(&far_outside_pos, &road_geometry));
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
