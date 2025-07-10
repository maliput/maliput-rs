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
fn branch_point_api() {
    let road_network = common::create_t_shape_road_network();
    let road_geometry = road_network.road_geometry();
    let branch_point_id = String::from("2");
    let branch_point = road_geometry.get_branch_point(&branch_point_id);
    assert_eq!(branch_point.id(), branch_point_id);
    assert_eq!(branch_point.road_geometry().id(), road_geometry.id());
    // Testing that the api works. The actual values are not important, they are tested in the
    // cpp tests.
    let lane_end_set = branch_point.get_a_side();
    assert_eq!(lane_end_set.size(), 1);
    let lane_end = lane_end_set.get(0);
    let confluent_branches = branch_point.get_confluent_branches(&lane_end);
    assert_eq!(confluent_branches.size(), 1);
    let ongoing_branches = branch_point.get_ongoing_branches(&lane_end);
    assert_eq!(ongoing_branches.size(), 2);
    let ongoing_branches_id_lane_end = ongoing_branches.to_lane_map();
    assert_eq!(ongoing_branches_id_lane_end.len(), 2);
    assert!(ongoing_branches_id_lane_end.contains_key("5_0_-1"));
    assert!(ongoing_branches_id_lane_end.contains_key("9_0_-1"));
    let lane_end_5 = ongoing_branches_id_lane_end.get("5_0_-1").unwrap();
    let lane_end_9 = ongoing_branches_id_lane_end.get("9_0_-1").unwrap();
    assert!(matches!(lane_end_5, maliput::api::LaneEnd::Start(_)));
    assert!(matches!(lane_end_9, maliput::api::LaneEnd::Start(_)));
    let lane_end_set = branch_point.get_b_side();
    assert_eq!(lane_end_set.size(), 2);

    // Test default branch.
    let default_lane_end = branch_point.get_default_branch(&lane_end);
    assert!(default_lane_end.is_some());
    match default_lane_end.unwrap() {
        maliput::api::LaneEnd::Start(l) | maliput::api::LaneEnd::Finish(l) => assert_eq!(l.id(), "9_0_-1"),
    }

    // Test that the default branch is None when the lane_end point to the other side.
    let the_other_lane_end = match lane_end {
        maliput::api::LaneEnd::Start(l) => maliput::api::LaneEnd::Finish(l),
        maliput::api::LaneEnd::Finish(l) => maliput::api::LaneEnd::Start(l),
    };
    let default_branch = branch_point.get_default_branch(&the_other_lane_end);
    assert!(default_branch.is_none());
}
