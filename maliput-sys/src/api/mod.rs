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
    /// Shared struct for `Lane` pointers.
    /// This is needed because `*const Lane` can't be used directly in the CxxVector collection.
    struct ConstLanePtr {
        pub lane: *const Lane,
    }

    unsafe extern "C++" {
        include!("api/api.h");

        #[namespace = "maliput::math"]
        type Vector3 = crate::math::ffi::Vector3;
        #[namespace = "maliput::math"]
        type Quaternion = crate::math::ffi::Quaternion;
        #[namespace = "maliput::math"]
        type Matrix3 = crate::math::ffi::Matrix3;
        #[namespace = "maliput::math"]
        type RollPitchYaw = crate::math::ffi::RollPitchYaw;

        #[namespace = "maliput::api"]
        type RoadNetwork;
        type RoadGeometry;
        // RoadNetwork bindings definitions.
        fn road_geometry(self: &RoadNetwork) -> *const RoadGeometry;
        fn RoadGeometry_id(road_geometry: &RoadGeometry) -> String;
        // RoadGeometry bindings definitions.
        fn num_junctions(self: &RoadGeometry) -> i32;
        fn linear_tolerance(self: &RoadGeometry) -> f64;
        fn angular_tolerance(self: &RoadGeometry) -> f64;
        fn num_branch_points(self: &RoadGeometry) -> i32;
        fn RoadGeometry_ToRoadPosition(
            rg: &RoadGeometry,
            inertial_position: &InertialPosition,
        ) -> UniquePtr<RoadPositionResult>;
        fn RoadGeometry_GetLane(rg: &RoadGeometry, lane_id: &String) -> ConstLanePtr;
        fn RoadGeometry_GetLanes(rg: &RoadGeometry) -> &CxxVector<ConstLanePtr>;
        fn RoadGeometry_GetSegment(rg: &RoadGeometry, segment_id: &String) -> *const Segment;
        fn RoadGeometry_GetJunction(rg: &RoadGeometry, junction_id: &String) -> *const Junction;
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

        // InertialPosition bindings definitions
        type InertialPosition;
        fn InertialPosition_new(x: f64, y: f64, z: f64) -> UniquePtr<InertialPosition>;
        fn x(self: &InertialPosition) -> f64;
        fn y(self: &InertialPosition) -> f64;
        fn z(self: &InertialPosition) -> f64;
        fn set_x(self: Pin<&mut InertialPosition>, x: f64);
        fn set_y(self: Pin<&mut InertialPosition>, y: f64);
        fn set_z(self: Pin<&mut InertialPosition>, z: f64);
        fn xyz(self: &InertialPosition) -> &Vector3;
        fn set_xyz(self: Pin<&mut InertialPosition>, xyz: &Vector3);
        fn length(self: &InertialPosition) -> f64;
        fn Distance(self: &InertialPosition, other: &InertialPosition) -> f64;
        fn InertialPosition_to_str(inertial_pos: &InertialPosition) -> String;
        fn InertialPosition_operator_eq(lhs: &InertialPosition, rhs: &InertialPosition) -> bool;
        fn InertialPosition_operator_sum(lhs: &InertialPosition, rhs: &InertialPosition)
            -> UniquePtr<InertialPosition>;
        fn InertialPosition_operator_sub(lhs: &InertialPosition, rhs: &InertialPosition)
            -> UniquePtr<InertialPosition>;
        fn InertialPosition_operator_mul_scalar(lhs: &InertialPosition, scalar: f64) -> UniquePtr<InertialPosition>;

        // Lane bindings definitions
        type Lane;
        fn to_left(self: &Lane) -> *const Lane;
        fn to_right(self: &Lane) -> *const Lane;
        fn index(self: &Lane) -> i32;
        fn length(self: &Lane) -> f64;
        fn Contains(self: &Lane, lane_position: &LanePosition) -> bool;
        fn Lane_id(lane: &Lane) -> String;
        fn Lane_lane_bounds(lane: &Lane, s: f64) -> UniquePtr<RBounds>;
        fn Lane_segment_bounds(lane: &Lane, s: f64) -> UniquePtr<RBounds>;
        fn Lane_elevation_bounds(lane: &Lane, s: f64, r: f64) -> UniquePtr<HBounds>;
        fn Lane_GetOrientation(lane: &Lane, lane_position: &LanePosition) -> UniquePtr<Rotation>;
        fn Lane_ToInertialPosition(lane: &Lane, lane_position: &LanePosition) -> UniquePtr<InertialPosition>;
        fn Lane_ToLanePosition(lane: &Lane, inertial_position: &InertialPosition) -> UniquePtr<LanePositionResult>;
        fn Lane_ToSegmentPosition(lane: &Lane, inertial_position: &InertialPosition) -> UniquePtr<LanePositionResult>;

        // Segment bindings definitions
        type Segment;
        fn num_lanes(self: &Segment) -> i32;
        fn junction(self: &Segment) -> *const Junction;
        fn lane(self: &Segment, index: i32) -> *const Lane;
        fn Segment_id(segment: &Segment) -> String;

        // Junction bindings definitions
        type Junction;
        fn road_geometry(self: &Junction) -> *const RoadGeometry;
        fn num_segments(self: &Junction) -> i32;
        fn segment(self: &Junction, index: i32) -> *const Segment;
        fn Junction_id(junction: &Junction) -> String;

        // RoadPosition bindings definitions
        type RoadPosition;
        /// # Safety
        /// This function is unsafe because it dereferences `lane` pointers.
        unsafe fn RoadPosition_new(lane: *const Lane, lane_pos: &LanePosition) -> UniquePtr<RoadPosition>;
        fn RoadPosition_ToInertialPosition(road_position: &RoadPosition) -> UniquePtr<InertialPosition>;
        fn RoadPosition_lane(road_position: &RoadPosition) -> *const Lane;
        fn RoadPosition_pos(road_position: &RoadPosition) -> UniquePtr<LanePosition>;

        // RoadPositionResult bindings definitions
        type RoadPositionResult;
        fn RoadPositionResult_road_position(result: &RoadPositionResult) -> UniquePtr<RoadPosition>;
        fn RoadPositionResult_nearest_position(result: &RoadPositionResult) -> UniquePtr<InertialPosition>;
        fn RoadPositionResult_distance(result: &RoadPositionResult) -> f64;

        // LanePositionResult bindings definitions
        type LanePositionResult;
        fn LanePositionResult_road_position(result: &LanePositionResult) -> UniquePtr<LanePosition>;
        fn LanePositionResult_nearest_position(result: &LanePositionResult) -> UniquePtr<InertialPosition>;
        fn LanePositionResult_distance(result: &LanePositionResult) -> f64;

        // Rotation bindings definitions
        type Rotation;
        fn Rotation_new() -> UniquePtr<Rotation>;
        fn Rotation_from_quat(q: &Quaternion) -> UniquePtr<Rotation>;
        fn Rotation_from_rpy(rpy: &RollPitchYaw) -> UniquePtr<Rotation>;
        fn roll(self: &Rotation) -> f64;
        fn pitch(self: &Rotation) -> f64;
        fn yaw(self: &Rotation) -> f64;
        fn quat(self: &Rotation) -> &Quaternion;
        fn Distance(self: &Rotation, other: &Rotation) -> f64;
        fn Rotation_set_quat(r: Pin<&mut Rotation>, q: &Quaternion);
        fn Rotation_rpy(r: &Rotation) -> UniquePtr<RollPitchYaw>;
        fn Rotation_matrix(r: &Rotation) -> UniquePtr<Matrix3>;
        fn Rotation_Apply(r: &Rotation, ip: &InertialPosition) -> UniquePtr<InertialPosition>;
        fn Rotation_Reverse(r: &Rotation) -> UniquePtr<Rotation>;

        // RBounds bindings definitions
        type RBounds;
        fn min(self: &RBounds) -> f64;
        fn max(self: &RBounds) -> f64;

        // HBounds bindings definitions
        type HBounds;
        fn min(self: &HBounds) -> f64;
        fn max(self: &HBounds) -> f64;
    }
    impl UniquePtr<RoadNetwork> {}
    impl UniquePtr<LanePosition> {}
}
