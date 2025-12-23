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

#[test]
fn lane_boundary_id_and_index() {
    let road_network = common::create_t_shape_road_network(false);
    let road_geometry = road_network.road_geometry();
    let lane = road_geometry.get_lane(&"0_0_1".to_string()).unwrap();

    let right_boundary = lane.right_boundary().unwrap();
    let boundary_id = right_boundary.id();

    assert!(!boundary_id.is_empty());

    let index = right_boundary.index();
    assert!(index >= 0);
    assert!(index < lane.segment().num_boundaries());
}

/// Tests accessing lanes adjacent to a boundary (left and right).
#[test]
fn lane_boundary_adjacent_lanes() {
    let road_network = common::create_t_shape_road_network(false);
    let road_geometry = road_network.road_geometry();
    let lane = road_geometry.get_lane(&"0_0_1".to_string()).unwrap();

    // Get the left and right boundaries of the lane
    let left_boundary = lane.left_boundary().unwrap();
    let right_boundary = lane.right_boundary().unwrap();

    let lane_to_right_of_left_boundary = left_boundary.lane_to_right();
    assert!(lane_to_right_of_left_boundary.is_some());

    let lane_to_left_of_right_boundary = right_boundary.lane_to_left();
    assert!(lane_to_left_of_right_boundary.is_some());
}

/// Tests getting lane markings from a boundary at a specific s coordinate.
#[test]
fn lane_boundary_get_marking() {
    let road_network = common::create_t_shape_road_network(false);
    let road_geometry = road_network.road_geometry();
    let lane = road_geometry.get_lane(&"0_0_1".to_string()).unwrap();

    // Get the right boundary
    let right_boundary = lane.right_boundary().unwrap();

    // Query marking at s=0.0
    let marking_query = right_boundary.get_marking(0.0);
    assert!(marking_query.is_some());
    // Query marking at lane length
    let marking_query = right_boundary.get_marking(lane.length());
    assert!(marking_query.is_some());
}

/// Tests getting all markings from a boundary.
#[test]
fn lane_boundary_get_all_markings() {
    let road_network = common::create_t_shape_road_network(false);
    let road_geometry = road_network.road_geometry();
    let lane = road_geometry.get_lane(&"0_0_1".to_string()).unwrap();

    // Get the right boundary
    let right_boundary = lane.right_boundary().unwrap();

    // Get all markings for this boundary
    let markings = right_boundary.get_markings();

    // There should be at least one marking
    assert!(!markings.is_empty());

    // Each marking should have valid s_start and s_end values
    for marking_query in &markings {
        assert!(marking_query.s_start >= 0.0);
        assert!(marking_query.s_end >= marking_query.s_start);
    }
}

/// Tests getting markings within a specific s range.
#[test]
fn lane_boundary_get_markings_by_range() {
    let road_network = common::create_t_shape_road_network(false);
    let road_geometry = road_network.road_geometry();
    let lane = road_geometry.get_lane(&"0_0_1".to_string()).unwrap();

    let right_boundary = lane.right_boundary().unwrap();

    let markings = right_boundary.get_markings_by_range(0.0, 10.0);

    // All returned markings should overlap with [0.0, 10.0]
    for marking_query in &markings {
        assert!(marking_query.s_end > 0.0);
        assert!(marking_query.s_start < 10.0);
    }
}

/// Tests that boundaries are shared between adjacent lanes.
#[test]
fn lane_boundary_shared_between_adjacent_lanes() {
    let road_network = common::create_t_shape_road_network(false);
    let road_geometry = road_network.road_geometry();
    let right_lane = road_geometry.get_lane(&"0_0_1".to_string()).unwrap();

    // Get the left lane
    let left_lane = right_lane.to_left();
    assert!(left_lane.is_some());
    let left_lane = left_lane.unwrap();

    // The right lane's left boundary should be the same as the left lane's right boundary
    let right_lane_left_boundary = right_lane.left_boundary().unwrap();
    let left_lane_right_boundary = left_lane.right_boundary().unwrap();

    // Both boundaries should have the same markings
    let right_lane_markings = right_lane_left_boundary.get_markings();
    let left_lane_markings = left_lane_right_boundary.get_markings();

    assert_eq!(right_lane_markings.len(), left_lane_markings.len());

    // The markings should be identical
    for (right_marking, left_marking) in right_lane_markings.iter().zip(left_lane_markings.iter()) {
        assert_eq!(
            right_marking.lane_marking.get_type(),
            left_marking.lane_marking.get_type()
        );
        assert_eq!(right_marking.lane_marking.color(), left_marking.lane_marking.color());
        assert_eq!(right_marking.lane_marking.width(), left_marking.lane_marking.width());
    }
}

/// Tests boundary segment association.
#[test]
fn lane_boundary_segment_association() {
    let road_network = common::create_t_shape_road_network(false);
    let road_geometry = road_network.road_geometry();
    let lane = road_geometry.get_lane(&"0_0_1".to_string()).unwrap();

    // Get the boundaries
    let left_boundary = lane.left_boundary().unwrap();
    let right_boundary = lane.right_boundary().unwrap();

    // Both boundaries should be associated with a segment
    let left_segment = left_boundary.segment();
    let right_segment = right_boundary.segment();

    assert!(left_segment.is_some());
    assert!(right_segment.is_some());

    // Both boundaries should belong to the same segment
    let left_seg = left_segment.unwrap();
    let right_seg = right_segment.unwrap();

    assert_eq!(left_seg.id(), right_seg.id());
}

/// Tests boundary lane adjacency in a multi-lane segment.
#[test]
fn lane_boundary_multi_lane_adjacency() {
    let road_network = common::create_t_shape_road_network(false);
    let road_geometry = road_network.road_geometry();
    let lane = road_geometry.get_lane(&"0_0_1".to_string()).unwrap();

    // Get boundaries
    let left_boundary = lane.left_boundary().unwrap();
    let right_boundary = lane.right_boundary().unwrap();

    // Left boundary should have a lane to the left (if not the leftmost)
    let lane_to_left = left_boundary.lane_to_left();

    // Right boundary should have a lane to the right (if not the rightmost)
    let lane_to_right = right_boundary.lane_to_right();

    // At least one of these should exist (we have multiple lanes)
    let has_left_lane = lane_to_left.is_some();
    let has_right_lane = lane_to_right.is_some();

    assert!(has_left_lane || has_right_lane);
}

/// Tests marking query s-range validity.
#[test]
fn lane_boundary_marking_query_s_ranges() {
    let road_network = common::create_t_shape_road_network(false);
    let road_geometry = road_network.road_geometry();
    let lane = road_geometry.get_lane(&"0_0_1".to_string()).unwrap();

    // Get the right boundary
    let right_boundary = lane.right_boundary().unwrap();

    // Get all markings
    let markings = right_boundary.get_markings();

    // Validate all s-ranges
    for marking_query in &markings {
        let s_start = marking_query.s_start;
        let s_end = marking_query.s_end;

        // s_start should be non-negative
        assert!(s_start >= 0.0, "s_start should be non-negative");

        // s_end should be greater than or equal to s_start
        assert!(s_end >= s_start, "s_end should be >= s_start");

        // For a valid marking, s_end should be greater than s_start
        assert!(s_end > s_start, "s_end should be > s_start for valid markings");
    }
}
