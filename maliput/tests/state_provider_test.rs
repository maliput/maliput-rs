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
use maliput::api::rules::RuleType;
use maliput::api::RoadPosition;

#[test]
fn test_phase_provider() {
    let road_network = common::create_t_shape_road_network_with_books();
    let phase_ring_book = road_network.phase_ring_book();
    assert_eq!(phase_ring_book.get_phase_rings_ids().len(), 1);
    let phase_ring_id = &phase_ring_book.get_phase_rings_ids()[0];
    let phase_provider = road_network.phase_provider();
    let state_provider = phase_provider.get_phase(phase_ring_id);
    assert!(state_provider.is_some());
    let state_provider = phase_provider.get_phase(&"TIntersectionPhaseRing".to_string());
    assert!(state_provider.is_some());
    let state_provider = state_provider.unwrap();
    assert_eq!(state_provider.state, "AllGo".to_string());
    assert!(state_provider.next.is_some());
    let next_state = state_provider.next.unwrap();
    assert_eq!(next_state.next_state, "AllStop".to_string());
    assert!(next_state.duration_until.is_some());
    let duration_until = next_state.duration_until.unwrap();
    assert_eq!(duration_until, 45.);
}

#[test]
fn test_discrete_value_rule_state_provider() {
    let road_network = common::create_loop_road_pedestrian_crosswalk_road_network_with_books();
    let discrete_state_provider = road_network.discrete_value_rule_state_provider();

    let discrete_rule_id = "Right-Of-Way Rule Type/WestToEastSouth".to_string();
    let discrete_state_query = discrete_state_provider.get_state_by_rule_id(&discrete_rule_id);
    assert!(discrete_state_query.is_some());
    let discrete_state_query = discrete_state_query.unwrap();
    assert_eq!(discrete_state_query.state.value(), "Go");
    assert!(discrete_state_query.next.is_some());
    let next_state = discrete_state_query.next.unwrap();
    assert_eq!(next_state.next_state.value(), "Stop");
    assert!(next_state.duration_until.is_some());
    let duration_until = next_state.duration_until.unwrap();
    assert_eq!(duration_until, 30.);

    let road_geometry = road_network.road_geometry();
    let lane = road_geometry.get_lane(&"1_1_-1".to_string()).unwrap();
    let road_position = RoadPosition::new(&lane, &maliput::api::LanePosition::new(0., 0., 0.));
    let discrete_state_query_by_type =
        discrete_state_provider.get_state_by_rule_type(&road_position, RuleType::RightOfWay, 1e-3);
    assert!(discrete_state_query_by_type.is_some());
    let discrete_state_query_by_type = discrete_state_query_by_type.unwrap();
    assert_eq!(discrete_state_query_by_type.state.value(), "Go");
    assert!(discrete_state_query_by_type.next.is_some());
    let next_state = discrete_state_query_by_type.next.unwrap();
    assert_eq!(next_state.next_state.value(), "Stop");
    assert!(next_state.duration_until.is_some());
    let duration_until = next_state.duration_until.unwrap();
    assert_eq!(duration_until, 30.);
}

#[test]
fn test_range_value_rule_state_provider() {
    let road_network = common::create_loop_road_pedestrian_crosswalk_road_network_with_books();
    let range_state_provider = road_network.range_value_rule_state_provider();

    let range_rule_id = "Invalid Rule Type/InvalidID".to_string();
    let range_state_query = range_state_provider.get_state_by_rule_id(&range_rule_id);
    assert!(range_state_query.is_none());

    // A Speed-Limit Rule is created for all roads even if the XODR does not specify them.
    let range_rule_id = "Speed-Limit Rule Type/1_0_1_1".to_string();
    let range_state_query = range_state_provider.get_state_by_rule_id(&range_rule_id);
    assert!(range_state_query.is_some());
    let range_state_query = range_state_query.unwrap();
    assert_eq!(range_state_query.state.min(), 0.);
    // The default max speed is set at 40km/h, which is 11.11111111111111 m/s.
    assert_eq!(range_state_query.state.max(), 11.11111111111111);
    assert!(range_state_query.next.is_none());

    let road_geometry = road_network.road_geometry();
    let lane = road_geometry.get_lane(&"1_1_1".to_string()).unwrap();
    let road_position = RoadPosition::new(&lane, &maliput::api::LanePosition::new(0., 0., 0.));
    let range_state_query_by_type =
        range_state_provider.get_state_by_rule_type(&road_position, RuleType::SpeedLimit, 1e-3);
    assert!(range_state_query_by_type.is_some());
    let range_state_query_by_type = range_state_query_by_type.unwrap();
    assert_eq!(range_state_query_by_type.state.min(), 0.);
    assert_eq!(range_state_query_by_type.state.max(), 11.11111111111111);
    assert!(range_state_query_by_type.next.is_none());
}
