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
fn segment_api() {
    let road_network = common::create_t_shape_road_network();
    let road_geometry = road_network.road_geometry();

    let segment_id = String::from("0_0");
    let segment = road_geometry.get_segment(&segment_id).unwrap();
    assert_eq!(segment.id(), segment_id);
    let junction = segment.junction();
    assert_eq!(junction.id(), String::from("0_0"));
    let num_lanes = segment.num_lanes();
    assert_eq!(num_lanes, 2);
    let lane = segment.lane(0);
    assert_eq!(lane.id(), String::from("0_0_-1"));
    // Testing invalid ids.
    let invalid_segment_id = String::from("invalid_id");
    let invalid_segment = road_geometry.get_segment(&invalid_segment_id);
    assert!(invalid_segment.is_none());
}
