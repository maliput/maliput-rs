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

#[cxx::bridge(namespace = "maliput::utility")]
#[allow(clippy::missing_safety_doc)]
pub mod ffi {

    pub struct Features {
        /// Maximum distance between rendered vertices, in either s- or r-dimension,
        /// along a lane's surface
        pub max_grid_unit: f64,
        /// Minimum number of vertices, in either s- or r-dimension, along a lane's
        /// surface.
        pub min_grid_resolution: f64,
        /// Draw stripes along lane_bounds() of each lane?
        pub draw_stripes: bool,
        /// Draw arrows at start/finish of each lane?
        pub draw_arrows: bool,
        /// Draw highlighting swath with lane_bounds() of each lane?
        pub draw_lane_haze: bool,
        /// Draw branching at BranchPoints?
        pub draw_branch_points: bool,
        /// Draw highlighting of elevation_bounds of each lane?
        pub draw_elevation_bounds: bool,
        /// Reduce the amount of vertices from the road by creating
        /// quads big enough which don't violate some tolerance. This could affect
        /// the accuracy of curved roads.
        pub off_grid_mesh_generation: bool,
        /// Tolerance for mesh simplification, or the distance from a vertex to an
        /// edge line or to a face plane at which said vertex is considered redundant
        /// (i.e. it is not necessary to further define those geometrical entities),
        /// in meters. If equal to 0, no mesh simplification will take place. If equal
        /// to the road linear tolerance, mesh simplification will be constrained
        /// enough so as to keep mesh geometrical accuracy. If greater than the road
        /// linear tolerance, mesh size reductions will come at the expense of
        /// geometrical accuracy.
        pub simplify_mesh_threshold: f64,
        /// Absolute width of stripes
        pub stripe_width: f64,
        /// Absolute elevation (h) of stripes above road surface
        pub stripe_elevation: f64,
        /// Absolute elevation (h) of arrows above road surface
        pub arrow_elevation: f64,
        /// Absolute elevation (h) of lane-haze above road surface
        pub lane_haze_elevation: f64,
        /// Absolute elevation (h) of branch-points above road surface
        pub branch_point_elevation: f64,
        /// Height of rendered branch-point arrows
        pub branch_point_height: f64,
        /// Origin of OBJ coordinates relative to `Inertial`-frame
        pub origin: [f64; 3],
        /// ID's of specific segments to be highlighted.  (If non-empty, then the
        /// Segments *not* specified on this list will be rendered as grayed-out.)
        pub highlighted_segments: Vec<String>,
    }
    unsafe extern "C++" {
        include!("cxx_utils/error_handling.h");
        include!("utility/utility.h");

        #[namespace = "maliput::api"]
        type RoadNetwork = crate::api::ffi::RoadNetwork;

        /// Generates an .obj and .mtl files named `fileroot` under the `dirpath` directory.
        ///
        /// # Safety
        ///
        /// The caller must ensure the `road_network` pointer is valid during the method execution.
        unsafe fn Utility_GenerateObjFile(
            road_network: *const RoadNetwork,
            dirpath: &String,
            fileroot: &String,
            features: &Features,
        );
    }
}
