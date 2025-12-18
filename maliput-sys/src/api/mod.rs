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

pub mod rules;

#[cxx::bridge(namespace = "maliput::api")]
#[allow(clippy::missing_safety_doc)]
pub mod ffi {
    /// Shared struct for `Lane` pointers.
    /// This is needed because `*const Lane` can't be used directly in the CxxVector collection.
    struct ConstLanePtr {
        pub lane: *const Lane,
    }
    /// Shared struct for `Intersection` pointers.
    /// This is needed because `*mut Intersection` can't be used directly in the CxxVector collection.
    struct MutIntersectionPtr {
        pub intersection: *mut Intersection,
    }
    /// Shared struct for `LaneSRange` references.
    /// This is needed because `&f` can't be used directly in the CxxVector collection.
    struct ConstLaneSRangeRef<'a> {
        pub lane_s_range: &'a LaneSRange,
    }

    /// Shared enum representing different types of lanes.
    /// This is needed to access the enum variant from Rust API since the C++ enum has an opaque implementation.
    /// The order of these variants must match with the order of the enum class defined in maliput C++ API.
    #[repr(u32)]
    pub enum LaneType {
        kUnknown,
        kDriving,
        kTurn,
        kHov,
        kBus,
        kTaxi,
        kEmergency,
        kShoulder,
        kBiking,
        kWalking,
        kParking,
        kStop,
        kBorder,
        kCurb,
        kMedian,
        kRestricted,
        kConstruction,
        kRail,
        kEntry,
        kExit,
        kOnRamp,
        kOffRamp,
        kConnectingRamp,
        kSlipLane,
        kVirtual,
    }

    unsafe extern "C++" {
        include!("api/api.h");
        include!("cxx_utils/error_handling.h");

        #[namespace = "maliput::math"]
        type Vector3 = crate::math::ffi::Vector3;
        #[namespace = "maliput::math"]
        type Quaternion = crate::math::ffi::Quaternion;
        #[namespace = "maliput::math"]
        type Matrix3 = crate::math::ffi::Matrix3;
        #[namespace = "maliput::math"]
        type RollPitchYaw = crate::math::ffi::RollPitchYaw;
        #[namespace = "maliput::api::rules"]
        type RoadRulebook = crate::api::rules::ffi::RoadRulebook;
        #[namespace = "maliput::api::rules"]
        type TrafficLightBook = crate::api::rules::ffi::TrafficLightBook;
        #[namespace = "maliput::api::rules"]
        type PhaseRingBook = crate::api::rules::ffi::PhaseRingBook;
        #[namespace = "maliput::api::rules"]
        type RuleRegistry = crate::api::rules::ffi::RuleRegistry;
        #[namespace = "maliput::api::rules"]
        type PhaseProvider = crate::api::rules::ffi::PhaseProvider;
        #[namespace = "maliput::api::rules"]
        type DiscreteValueRuleStateProvider = crate::api::rules::ffi::DiscreteValueRuleStateProvider;
        #[namespace = "maliput::api::rules"]
        type RangeValueRuleStateProvider = crate::api::rules::ffi::RangeValueRuleStateProvider;
        #[namespace = "maliput::api::rules"]
        type PhaseStateProviderQuery = crate::api::rules::ffi::PhaseStateProviderQuery;
        #[namespace = "maliput::api::rules"]
        type DiscreteValueRuleDiscreteValue = crate::api::rules::ffi::DiscreteValueRuleDiscreteValue;
        #[namespace = "maliput::api::rules"]
        type DiscreteValueRuleState = crate::api::rules::ffi::DiscreteValueRuleState;
        #[namespace = "maliput::api::rules"]
        type Phase = crate::api::rules::ffi::Phase;
        #[namespace = "maliput::api::rules"]
        type NextPhase = crate::api::rules::ffi::NextPhase;
        #[namespace = "maliput::api::rules"]
        type BulbState = crate::api::rules::ffi::BulbState;
        #[namespace = "maliput::api::rules"]
        type UniqueBulbId = crate::api::rules::ffi::UniqueBulbId;

        #[namespace = "maliput::api"]
        // RoadNetwork bindings definitions.
        type RoadNetwork;
        fn road_geometry(self: &RoadNetwork) -> *const RoadGeometry;
        fn RoadNetwork_intersection_book(road_network: &RoadNetwork) -> *const IntersectionBook;
        fn traffic_light_book(self: &RoadNetwork) -> *const TrafficLightBook;
        fn rulebook(self: &RoadNetwork) -> *const RoadRulebook;
        fn phase_ring_book(self: &RoadNetwork) -> *const PhaseRingBook;
        fn rule_registry(self: &RoadNetwork) -> *const RuleRegistry;
        fn RoadNetwork_phase_provider(road_network: &RoadNetwork) -> *const PhaseProvider;
        fn RoadNetwork_discrete_value_rule_state_provider(
            road_network: &RoadNetwork,
        ) -> *const DiscreteValueRuleStateProvider;
        fn RoadNetwork_range_value_rule_state_provider(
            road_network: &RoadNetwork,
        ) -> *const RangeValueRuleStateProvider;

        // RoadGeometry bindings definitions.
        type RoadGeometry;
        fn RoadGeometry_id(road_geometry: &RoadGeometry) -> String;
        fn num_junctions(self: &RoadGeometry) -> i32;
        fn linear_tolerance(self: &RoadGeometry) -> f64;
        fn angular_tolerance(self: &RoadGeometry) -> f64;
        fn num_branch_points(self: &RoadGeometry) -> i32;
        fn junction(self: &RoadGeometry, index: i32) -> Result<*const Junction>;
        fn RoadGeometry_ToRoadPosition(
            rg: &RoadGeometry,
            inertial_position: &InertialPosition,
        ) -> Result<UniquePtr<RoadPositionResult>>;
        fn RoadGeometry_FindRoadPositions(
            rg: &RoadGeometry,
            inertial_position: &InertialPosition,
            radius: f64,
        ) -> Result<UniquePtr<CxxVector<RoadPositionResult>>>;
        fn RoadGeometry_GetLane(rg: &RoadGeometry, lane_id: &String) -> ConstLanePtr;
        fn RoadGeometry_GetLanes(rg: &RoadGeometry) -> UniquePtr<CxxVector<ConstLanePtr>>;
        fn RoadGeometry_GetSegment(rg: &RoadGeometry, segment_id: &String) -> *const Segment;
        fn RoadGeometry_GetJunction(rg: &RoadGeometry, junction_id: &String) -> *const Junction;
        fn RoadGeometry_GetBranchPoint(rg: &RoadGeometry, branch_point_id: &String) -> *const BranchPoint;
        fn RoadGeometry_BackendCustomCommand(rg: &RoadGeometry, command: &String) -> Result<String>;
        fn RoadGeometry_GeoReferenceInfo(rg: &RoadGeometry) -> String;
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
        fn segment(self: &Lane) -> *const Segment;
        fn Lane_id(lane: &Lane) -> String;
        fn Lane_lane_bounds(lane: &Lane, s: f64) -> Result<UniquePtr<RBounds>>;
        fn Lane_segment_bounds(lane: &Lane, s: f64) -> Result<UniquePtr<RBounds>>;
        fn Lane_elevation_bounds(lane: &Lane, s: f64, r: f64) -> Result<UniquePtr<HBounds>>;
        fn Lane_GetOrientation(lane: &Lane, lane_position: &LanePosition) -> Result<UniquePtr<Rotation>>;
        fn Lane_ToInertialPosition(lane: &Lane, lane_position: &LanePosition) -> Result<UniquePtr<InertialPosition>>;
        fn Lane_ToLanePosition(
            lane: &Lane,
            inertial_position: &InertialPosition,
        ) -> Result<UniquePtr<LanePositionResult>>;
        fn Lane_ToSegmentPosition(
            lane: &Lane,
            inertial_position: &InertialPosition,
        ) -> Result<UniquePtr<LanePositionResult>>;
        fn Lane_GetBranchPoint(lane: &Lane, start: bool) -> *const BranchPoint;
        fn Lane_GetConfluentBranches(lane: &Lane, start: bool) -> Result<*const LaneEndSet>;
        fn Lane_GetOngoingBranches(lane: &Lane, start: bool) -> Result<*const LaneEndSet>;
        fn Lane_GetDefaultBranch(lane: &Lane, start: bool) -> UniquePtr<LaneEnd>;
        fn Lane_EvalMotionDerivatives(
            lane: &Lane,
            lane_position: &LanePosition,
            sigma_v: f64,
            rho_v: f64,
            eta_v: f64,
        ) -> UniquePtr<LanePosition>;
        type LaneType;
        fn Lane_type(lane: &Lane) -> LaneType;

        // Segment bindings definitions
        type Segment;
        fn num_lanes(self: &Segment) -> i32;
        fn junction(self: &Segment) -> Result<*const Junction>;
        fn lane(self: &Segment, index: i32) -> Result<*const Lane>;
        fn Segment_id(segment: &Segment) -> String;

        // Junction bindings definitions
        type Junction;
        fn road_geometry(self: &Junction) -> *const RoadGeometry;
        fn num_segments(self: &Junction) -> i32;
        fn segment(self: &Junction, index: i32) -> Result<*const Segment>;
        fn Junction_id(junction: &Junction) -> String;

        // RoadPosition bindings definitions
        type RoadPosition;
        /// # Safety
        ///
        /// This function is unsafe because it dereferences `lane` pointers.
        unsafe fn RoadPosition_new(lane: *const Lane, lane_pos: &LanePosition) -> UniquePtr<RoadPosition>;
        fn RoadPosition_ToInertialPosition(road_position: &RoadPosition) -> Result<UniquePtr<InertialPosition>>;
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

        // SRange bindings definitions
        type SRange;
        fn SRange_new(s0: f64, s1: f64) -> UniquePtr<SRange>;
        fn s0(self: &SRange) -> f64;
        fn s1(self: &SRange) -> f64;
        fn set_s0(self: Pin<&mut SRange>, s0: f64);
        fn set_s1(self: Pin<&mut SRange>, s1: f64);
        fn size(self: &SRange) -> f64;
        fn WithS(self: &SRange) -> bool;
        fn Intersects(self: &SRange, other: &SRange, tolerance: f64) -> bool;
        fn Contains(self: &SRange, s_range: &SRange, tolerance: f64) -> bool;
        fn SRange_GetIntersection(s_range: &SRange, other: &SRange, tolerance: f64) -> UniquePtr<SRange>;

        // LaneSRange bindings definitions
        type LaneSRange;
        fn LaneSRange_new(lane_id: &String, s_range: &SRange) -> UniquePtr<LaneSRange>;
        fn length(self: &LaneSRange) -> f64;
        fn Intersects(self: &LaneSRange, other: &LaneSRange, tolerance: f64) -> bool;
        fn Contains(self: &LaneSRange, lane_s_range: &LaneSRange, tolerance: f64) -> bool;
        fn LaneSRange_lane_id(lane_s_range: &LaneSRange) -> String;
        fn LaneSRange_s_range(lane_s_range: &LaneSRange) -> UniquePtr<SRange>;
        fn LaneSRange_GetIntersection(
            lane_s_range: &LaneSRange,
            other: &LaneSRange,
            tolerance: f64,
        ) -> UniquePtr<LaneSRange>;

        // LaneSRoute bindings definitions
        type LaneSRoute;
        fn LaneSRoute_new(ranges: &CxxVector<ConstLaneSRangeRef>) -> UniquePtr<LaneSRoute>;
        fn length(self: &LaneSRoute) -> f64;
        fn Intersects(self: &LaneSRoute, other: &LaneSRoute, tolerance: f64) -> bool;
        fn ranges(self: &LaneSRoute) -> &CxxVector<LaneSRange>;

        // LaneEnd bindings definitions
        type LaneEnd;
        // maliput::api Rust will have its own LaneEnd enum.
        // However, this LaneEnd_new is expected to be used on methods that return a Cpp LaneEnd
        // and a conversion to Rust LaneEnd is needed.
        /// # Safety
        /// This function is unsafe because it dereferences `lane` pointer.
        unsafe fn LaneEnd_new(lane: *const Lane, start: bool) -> UniquePtr<LaneEnd>;
        fn LaneEnd_lane(lane_end: &LaneEnd) -> *const Lane;
        fn LaneEnd_is_start(lane_end: &LaneEnd) -> bool;

        // LaneEndSet bindings definitions
        type LaneEndSet;
        fn size(self: &LaneEndSet) -> i32;
        fn get(self: &LaneEndSet, index: i32) -> Result<&LaneEnd>;

        // BranchPoint bindings definitions
        type BranchPoint;
        fn BranchPoint_id(branch_point: &BranchPoint) -> String;
        fn road_geometry(self: &BranchPoint) -> *const RoadGeometry;
        fn GetConfluentBranches(self: &BranchPoint, end: &LaneEnd) -> Result<*const LaneEndSet>;
        fn GetOngoingBranches(self: &BranchPoint, end: &LaneEnd) -> Result<*const LaneEndSet>;
        fn GetASide(self: &BranchPoint) -> *const LaneEndSet;
        fn GetBSide(self: &BranchPoint) -> *const LaneEndSet;
        fn BranchPoint_GetDefaultBranch(branch_point: &BranchPoint, end: &LaneEnd) -> UniquePtr<LaneEnd>;

        // Intersection bindings definitions
        type Intersection;
        fn Intersection_id(intersection: &Intersection) -> String;
        fn Intersection_Phase(intersection: &Intersection) -> UniquePtr<PhaseStateProviderQuery>;
        fn region(self: &Intersection) -> &CxxVector<LaneSRange>;
        fn Intersection_ring_id(intersection: &Intersection) -> String;
        fn Intersection_unique_bulb_ids(intersection: &Intersection) -> UniquePtr<CxxVector<UniqueBulbId>>;
        fn Intersection_bulb_state(intersection: &Intersection, bulb_id: &UniqueBulbId) -> UniquePtr<BulbState>;
        fn Intersection_DiscreteValueRuleStates(
            intersection: &Intersection,
        ) -> UniquePtr<CxxVector<DiscreteValueRuleState>>;
        fn Intersection_IncludesTrafficLight(intersection: &Intersection, traffic_light_id: &String) -> bool;
        fn Intersection_IncludesDiscreteValueRule(intersection: &Intersection, rule_id: &String) -> bool;
        fn Intersection_IncludesInertialPosition(
            intersection: &Intersection,
            inertial_position: &InertialPosition,
            road_geometry: &RoadGeometry,
        ) -> bool;

        // IntersectionBook bindings definitions
        type IntersectionBook;
        fn IntersectionBook_GetIntersection(book: &IntersectionBook, id: &String) -> MutIntersectionPtr;
        fn IntersectionBook_GetIntersections(book: &IntersectionBook) -> UniquePtr<CxxVector<MutIntersectionPtr>>;
        fn IntersectionBook_FindIntersectionTrafficLight(
            book: &IntersectionBook,
            traffic_light_id: &String,
        ) -> MutIntersectionPtr;
        fn IntersectionBook_FindIntersectionDiscreteValueRule(
            book: &IntersectionBook,
            rule_id: &String,
        ) -> MutIntersectionPtr;
        fn IntersectionBook_FindIntersectionInertialPosition(
            book: &IntersectionBook,
            inertial_position: &InertialPosition,
        ) -> MutIntersectionPtr;

    }
    impl UniquePtr<RoadNetwork> {}
    impl UniquePtr<LanePosition> {}
}
