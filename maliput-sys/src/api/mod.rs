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

#[cxx::bridge(namespace = "maliput::api")]
pub mod ffi {
    unsafe extern "C++" {
        include!("api/api.h");

        #[namespace = "maliput::math"]
        type Vector3 = crate::math::ffi::Vector3;

        #[namespace = "maliput::api"]
        type RoadNetwork;
        type RoadGeometry;
        // RoadNetwork bindings definitions.
        fn road_geometry(self: &RoadNetwork) -> *const RoadGeometry;
        // RoadGeometry bindings definitions.
        fn num_junctions(self: &RoadGeometry) -> i32;
        fn linear_tolerance(self: &RoadGeometry) -> f64;
        fn angular_tolerance(self: &RoadGeometry) -> f64;
        fn num_branch_points(self: &RoadGeometry) -> i32;

        // LanePosition bindings definitions.
        type LanePosition;
        fn LanePosition_new(s: f64, r: f64, h: f64) -> UniquePtr<LanePosition>;
        fn s(self: &LanePosition) -> f64;
        fn r(self: &LanePosition) -> f64;
        fn h(self: &LanePosition) -> f64;
        fn set_s(self: Pin<&mut LanePosition>, s: f64);
        fn set_r(self: Pin<&mut LanePosition>, r: f64);
        fn set_h(self: Pin<&mut LanePosition>, h: f64);
        fn srh(self: &LanePosition) -> &Vector3;
        fn set_srh(self: Pin<&mut LanePosition>, srh: &Vector3);
        fn LanePosition_to_str(lane_pos: &LanePosition) -> String;
    }
    impl UniquePtr<RoadNetwork> {}
    impl UniquePtr<LanePosition> {}
}
