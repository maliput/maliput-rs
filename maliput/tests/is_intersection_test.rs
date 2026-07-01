// BSD 3-Clause License
//
// Copyright (c) 2026, Woven by Toyota.
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
fn junction_segment_lane_is_intersection_from_xodr_userdata() {
    const XODR_FIXTURE: &str = include_str!("../data/xodr/TShapeRoadJunctionUserDataIntersection.xodr");
    assert!(XODR_FIXTURE.contains("<userData code=\"junctionType\" value=\"intersection\"/>"));

    let road_network = common::create_junction_userdata_intersection_road_network();
    let road_geometry = road_network.road_geometry();

    for i in 0..road_geometry.num_junctions() {
        let junction = road_geometry.junction(i).unwrap();
        // The Junction with id "3" is the only one that has the userData code="junctionType" value="intersection" in the XODR.
        if junction.id().as_str() == "3" {
            assert_eq!(junction.is_intersection(), Some(true));
        } else {
            assert_eq!(junction.is_intersection(), Some(false));
        }
        for j in 0..junction.num_segments() {
            let segment = junction.segment(j).unwrap();
            assert_eq!(segment.is_intersection(), junction.is_intersection());
            for k in 0..segment.num_lanes() {
                let lane = segment.lane(k).unwrap();
                // Now check that the lane, segment and junction all have the same is_intersection() value.
                assert_eq!(lane.is_intersection(), segment.is_intersection());
                assert_eq!(lane.is_intersection(), junction.is_intersection());
            }
        }
    }
}
