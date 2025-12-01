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

#pragma once

#include <memory>
#include <sstream>
#include <vector>

#include <maliput/api/branch_point.h>
#include <maliput/api/intersection.h>
#include <maliput/api/intersection_book.h>
#include <maliput/api/junction.h>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/road_network.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/regions.h>
#include <maliput/api/segment.h>

#include <rust/cxx.h>

#include "maliput-sys/src/api/mod.rs.h"
#include "maliput-sys/src/api/rules/rules.h"

namespace maliput {
namespace api {

struct ConstLanePtr;
struct MutIntersectionPtr;

/// Gets the `RoadNetowrk`s read-only `PhaseProvider`.
const maliput::api::rules::PhaseProvider* RoadNetwork_phase_provider(const RoadNetwork& road_network) {
    maliput::api::RoadNetwork* rn = const_cast<RoadNetwork*>(&road_network);
    maliput::api::rules::PhaseProvider* phase_provider = rn->phase_provider();
    return phase_provider;
}

/// Gets the `RoadNetowrk`s read-only `DiscreteValueRuleStateProvider`.
const maliput::api::rules::DiscreteValueRuleStateProvider* RoadNetwork_discrete_value_rule_state_provider(const RoadNetwork& road_network) {
    maliput::api::RoadNetwork* rn = const_cast<RoadNetwork*>(&road_network);
    maliput::api::rules::DiscreteValueRuleStateProvider* discrete_value_rule_state_provider = rn->discrete_value_rule_state_provider();
    return discrete_value_rule_state_provider;
}

/// Gets the `RoadNetowrk`s read-only `RangeValueRuleStateProvider`.
const maliput::api::rules::RangeValueRuleStateProvider* RoadNetwork_range_value_rule_state_provider(const RoadNetwork& road_network) {
    maliput::api::RoadNetwork* rn = const_cast<RoadNetwork*>(&road_network);
    maliput::api::rules::RangeValueRuleStateProvider* range_value_rule_state_provider = rn->range_value_rule_state_provider();
    return range_value_rule_state_provider;
}

/// Creates a new maliput::api::LanePosition.
/// Forwads to maliput::api::LanePosition(double s, double r, double h) constructor.
std::unique_ptr<LanePosition> LanePosition_new(rust::f64 s, rust::f64 r, rust::f64 h) {
  return std::make_unique<LanePosition>(s, r, h);
}

/// Creates a new maliput::api::LanePosition.
/// Forwads to maliput::api::LanePosition(double s, double r, double h) constructor.
rust::String LanePosition_to_str(const LanePosition& lane_pos) {
  std::stringstream ss;
  ss << lane_pos;
  return {ss.str()};
}

/// Creates a new maliput::api::InertialPosition.
/// Forwads to maliput::api::InertialPosition(double x, double y, double z) constructor.
std::unique_ptr<InertialPosition> InertialPosition_new(rust::f64 x, rust::f64 y, rust::f64 z) {
  return std::make_unique<InertialPosition>(x, y, z);
}

rust::String InertialPosition_to_str(const InertialPosition& inertial_pos) {
  std::stringstream ss;
  ss << inertial_pos;
  return {ss.str()};
}

std::unique_ptr<InertialPosition> InertialPosition_operator_sum(const InertialPosition& lhs, const InertialPosition& rhs) {
  return std::make_unique<InertialPosition>(lhs + rhs);
}

std::unique_ptr<InertialPosition> InertialPosition_operator_sub(const InertialPosition& lhs, const InertialPosition& rhs) {
  return std::make_unique<InertialPosition>(lhs - rhs);
}

std::unique_ptr<InertialPosition> InertialPosition_operator_mul_scalar(const InertialPosition& lhs, rust::f64 scalar) {
  return std::make_unique<InertialPosition>(lhs * scalar);
}

bool InertialPosition_operator_eq(const InertialPosition& lhs, const InertialPosition& rhs) {
  return lhs == rhs;
}

rust::String Segment_id(const Segment& segment) {
  return segment.id().string();
}

rust::String Junction_id(const Junction& junction) {
  return junction.id().string();
}

rust::String Lane_id(const Lane& lane) {
  return lane.id().string();
}

std::unique_ptr<Rotation> Lane_GetOrientation(const Lane& lane, const LanePosition& lane_position) {
  return std::make_unique<Rotation>(lane.GetOrientation(lane_position));
}

std::unique_ptr<InertialPosition> Lane_ToInertialPosition(const Lane& lane, const LanePosition& lane_position) {
  return std::make_unique<InertialPosition>(lane.ToInertialPosition(lane_position));
}

std::unique_ptr<RBounds>Lane_lane_bounds(const Lane& lane, rust::f64 s) {
  return std::make_unique<RBounds>(lane.lane_bounds(s));
}

std::unique_ptr<RBounds>Lane_segment_bounds(const Lane& lane, rust::f64 s) {
  return std::make_unique<RBounds>(lane.segment_bounds(s));
}

std::unique_ptr<HBounds>Lane_elevation_bounds(const Lane& lane, rust::f64 s, rust::f64 r) {
  return std::make_unique<HBounds>(lane.elevation_bounds(s, r));
}

std::unique_ptr<LanePositionResult> Lane_ToLanePosition(const Lane& lane, const InertialPosition& inertial_pos) {
  return std::make_unique<LanePositionResult>(lane.ToLanePosition(inertial_pos));
}

std::unique_ptr<LanePositionResult> Lane_ToSegmentPosition(const Lane& lane, const InertialPosition& inertial_pos) {
  return std::make_unique<LanePositionResult>(lane.ToSegmentPosition(inertial_pos));
}

const BranchPoint* Lane_GetBranchPoint(const Lane& lane, bool start) {
  return lane.GetBranchPoint(start ? LaneEnd::kStart : LaneEnd::kFinish);
}

const LaneEndSet* Lane_GetConfluentBranches(const Lane& lane, bool start) {
  return lane.GetConfluentBranches(start ? LaneEnd::kStart : LaneEnd::kFinish);
}

const LaneEndSet* Lane_GetOngoingBranches(const Lane& lane, bool start) {
  return lane.GetOngoingBranches(start ? LaneEnd::kStart : LaneEnd::kFinish);
}

std::unique_ptr<LaneEnd> Lane_GetDefaultBranch(const Lane& lane, bool start) {
  const auto default_branch = lane.GetDefaultBranch(start ? LaneEnd::kStart : LaneEnd::kFinish);
  return default_branch ? std::make_unique<LaneEnd>(*default_branch) : nullptr;
}

std::unique_ptr<LanePosition> Lane_EvalMotionDerivatives(const Lane& lane, const LanePosition& lane_position, rust::f64 sigma_v, rust::f64 rho_v, rust::f64 eta_v) {
  return std::make_unique<LanePosition>(lane.EvalMotionDerivatives(lane_position, IsoLaneVelocity{sigma_v, rho_v, eta_v}));
}

std::unique_ptr<RoadPosition> RoadPosition_new(const Lane* lane, const LanePosition& pos) {
  return std::make_unique<RoadPosition>(lane, pos);
}

std::unique_ptr<InertialPosition> RoadPosition_ToInertialPosition(const RoadPosition& road_pos) {
  return std::make_unique<InertialPosition>(road_pos.ToInertialPosition());
}

const Lane* RoadPosition_lane(const RoadPosition& road_pos) {
  return road_pos.lane;
}

std::unique_ptr<LanePosition> RoadPosition_pos(const RoadPosition& road_pos) {
  return std::make_unique<LanePosition>(road_pos.pos);
}

std::unique_ptr<RoadPosition> RoadPositionResult_road_position(const RoadPositionResult& road_pos_res) {
  return std::make_unique<RoadPosition>(road_pos_res.road_position);
}

std::unique_ptr<InertialPosition> RoadPositionResult_nearest_position (const RoadPositionResult& road_pos_res) {
  return std::make_unique<InertialPosition>(road_pos_res.nearest_position);
}

double RoadPositionResult_distance(const RoadPositionResult& road_pos_res) {
  return road_pos_res.distance;
}

std::unique_ptr<LanePosition> LanePositionResult_road_position(const LanePositionResult& lane_pos_res) {
  return std::make_unique<LanePosition>(lane_pos_res.lane_position);
}

std::unique_ptr<InertialPosition> LanePositionResult_nearest_position (const LanePositionResult& lane_pos_res) {
  return std::make_unique<InertialPosition>(lane_pos_res.nearest_position);
}

double LanePositionResult_distance(const LanePositionResult& lane_pos_res) {
  return lane_pos_res.distance;
}

rust::String RoadGeometry_id(const RoadGeometry& road_geometry) {
  return road_geometry.id().string();
}

std::unique_ptr<RoadPositionResult> RoadGeometry_ToRoadPosition(const RoadGeometry& road_geometry, const InertialPosition& inertial_pos) {
  return std::make_unique<RoadPositionResult>(road_geometry.ToRoadPosition(inertial_pos));
}

std::unique_ptr<std::vector<RoadPositionResult>> RoadGeometry_FindRoadPositions(const RoadGeometry& road_geometry, const InertialPosition& inertial_pos, double radius) {
  return std::make_unique<std::vector<RoadPositionResult>>(road_geometry.FindRoadPositions(inertial_pos, radius));
}

ConstLanePtr RoadGeometry_GetLane(const RoadGeometry& road_geometry, const rust::String& lane_id) {
  return {road_geometry.ById().GetLane(LaneId{std::string(lane_id)})};
}

std::unique_ptr<std::vector<ConstLanePtr>> RoadGeometry_GetLanes(const RoadGeometry& road_geometry) {
  std::vector<ConstLanePtr> lanes;
  const auto lanes_cpp = road_geometry.ById().GetLanes();
  lanes.reserve(lanes_cpp.size());
  for (const auto& lane : road_geometry.ById().GetLanes()) {
    lanes.push_back(ConstLanePtr{lane.second});
  }
  return std::make_unique<std::vector<ConstLanePtr>>(std::move(lanes));
}

const BranchPoint* RoadGeometry_GetBranchPoint(const RoadGeometry& road_geometry, const rust::String& branch_point_id) {
  return road_geometry.ById().GetBranchPoint(BranchPointId{std::string(branch_point_id)});
}

const Segment* RoadGeometry_GetSegment(const RoadGeometry& road_geometry, const rust::String& segment_id) {
  return road_geometry.ById().GetSegment(SegmentId{std::string(segment_id)});
}

const Junction* RoadGeometry_GetJunction(const RoadGeometry& road_geometry, const rust::String& junction_id) {
  return road_geometry.ById().GetJunction(JunctionId{std::string(junction_id)});
}

rust::String RoadGeometry_BackendCustomCommand(const RoadGeometry& road_geometry, const rust::String& command) {
  return road_geometry.BackendCustomCommand(std::string(command));
}

rust::String RoadGeometry_GeoReferenceInfo(const RoadGeometry& road_geometry) {
  return road_geometry.GeoReferenceInfo();
}

std::unique_ptr<Rotation> Rotation_new() {
  return std::make_unique<Rotation>();
}

std::unique_ptr<Rotation> Rotation_from_quat(const maliput::math::Quaternion& q) {
  return std::make_unique<Rotation>(Rotation::FromQuat(q));
}

std::unique_ptr<Rotation> Rotation_from_rpy(const maliput::math::RollPitchYaw& rpy) {
  return std::make_unique<Rotation>(Rotation::FromRpy(rpy.vector()));
}

void Rotation_set_quat(Rotation& rotation, const maliput::math::Quaternion& q) {
  rotation.set_quat(q);
}

std::unique_ptr<maliput::math::RollPitchYaw> Rotation_rpy(const Rotation& rotation) {
  return std::make_unique<maliput::math::RollPitchYaw>(rotation.rpy());
}

std::unique_ptr<maliput::math::Matrix3> Rotation_matrix(const Rotation& rotation) {
  return std::make_unique<maliput::math::Matrix3>(rotation.matrix());
}

std::unique_ptr<InertialPosition> Rotation_Apply(const Rotation& rotation, const InertialPosition& inertial_pos) {
  return std::make_unique<InertialPosition>(rotation.Apply(inertial_pos));
}

std::unique_ptr<Rotation> Rotation_Reverse(const Rotation& rotation) {
  return std::make_unique<Rotation>(rotation.Reverse());
}

std::unique_ptr<SRange> SRange_new(rust::f64 start, rust::f64 end) {
  return std::make_unique<SRange>(start, end);
}

std::unique_ptr<SRange> SRange_GetIntersection(const SRange& s_range, const SRange& other_s_range, rust::f64 tolerance) {
  const auto intersection = s_range.GetIntersection(other_s_range, tolerance);
  if (intersection) {
    return std::make_unique<SRange>(*intersection);
  }
  return nullptr;
}

std::unique_ptr<LaneSRange> LaneSRange_new(const rust::String& lane_id, const SRange& s_range) {
  return std::make_unique<LaneSRange>(LaneId{std::string(lane_id)}, s_range);
}

rust::String LaneSRange_lane_id(const LaneSRange& lane_s_range) {
  return lane_s_range.lane_id().string();
}

std::unique_ptr<SRange> LaneSRange_s_range(const LaneSRange& lane_s_range) {
  return std::make_unique<SRange>(lane_s_range.s_range());
}

std::unique_ptr<LaneSRange> LaneSRange_GetIntersection(const LaneSRange& lane_s_range, const LaneSRange& other_lane_s_range, rust::f64 tolerance) {
  const auto intersection = lane_s_range.GetIntersection(other_lane_s_range, tolerance);
  if (intersection) {
    return std::make_unique<LaneSRange>(*intersection);
  }
  return nullptr;
}

std::unique_ptr<LaneSRoute> LaneSRoute_new(const std::vector<ConstLaneSRangeRef>& lane_s_ranges) {
  std::vector<LaneSRange> lane_s_ranges_cpp;
  lane_s_ranges_cpp.reserve(lane_s_ranges.size());
  for (const auto& lane_s_range : lane_s_ranges) {
    lane_s_ranges_cpp.push_back(LaneSRange{lane_s_range.lane_s_range.lane_id(), lane_s_range.lane_s_range.s_range()});
  }
  return std::make_unique<LaneSRoute>(lane_s_ranges_cpp);
}

std::unique_ptr<LaneEnd> LaneEnd_new(const Lane* lane, bool start) {
  return std::make_unique<LaneEnd>(lane, start ? LaneEnd::kStart : LaneEnd::kFinish);
}

const Lane* LaneEnd_lane(const LaneEnd& lane_end) {
  return lane_end.lane;
}

bool LaneEnd_is_start(const LaneEnd& lane_end) {
  return lane_end.end == LaneEnd::kStart;
}

rust::String BranchPoint_id(const BranchPoint& branch_point) {
  return branch_point.id().string();
}

std::unique_ptr<LaneEnd> BranchPoint_GetDefaultBranch(const BranchPoint& branch_point, const LaneEnd& end) {
  const auto default_branch = branch_point.GetDefaultBranch(end);
  return default_branch ? std::make_unique<LaneEnd>(*default_branch) : nullptr;
}

rust::String Intersection_id(const Intersection& intersection) {
  return intersection.id().string();
}

std::unique_ptr<rules::PhaseStateProviderQuery> Intersection_Phase(const Intersection& intersection) {
  const auto phase_state_provider_query = intersection.Phase();
  return phase_state_provider_query ? std::make_unique<rules::PhaseStateProviderQuery>(*phase_state_provider_query) : nullptr;
}

void Intersection_SetPhase(Intersection& intersection, const rules::Phase& phase, const rules::NextPhase& next_phase) {
  std::optional<double> duration_until = std::nullopt;
  if (!next_phase.duration_until) {
    duration_until = next_phase.duration_until->value;
  }
  intersection.SetPhase(phase.id(), std::make_optional<rules::Phase::Id>(rules::Phase::Id{std::string(next_phase.phase_id)}), duration_until);
}

rust::String Intersection_ring_id(const Intersection& intersection) {
  return intersection.ring_id().string();
}

// std::vector<rules::UniqueBulbId> Intersection_unique_bulb_ids(const Intersection& intersection) {
//   std::vector<rules::UniqueBulbId> bulb_ids;
//   const auto bulb_ids_cpp = intersection.bulb_states();
  
//   if (!bulb_ids_cpp.has_value()) {
//     return bulb_ids;
//   }
  
//   bulb_ids.reserve(bulb_ids_cpp->size());
  
//   for (const auto& bulb_id_pair : bulb_ids_cpp.value()) {
//     bulb_ids.push_back(bulb_id_pair.first); 
//   }

//   return bulb_ids;
// }

std::unique_ptr<std::vector<rules::UniqueBulbState>> Intersection_bulb_states(const Intersection& intersection) {
  std::vector<rules::UniqueBulbState> bulb_states;
  const auto bulb_states_cpp = intersection.bulb_states();
  
  if (!bulb_states_cpp.has_value()) {
    return nullptr;
  }
  for (const auto& bulb_state_cpp : bulb_states_cpp.value()) {
    rules::UniqueBulbState bulb_state{std::make_unique<rules::UniqueBulbId>(bulb_state_cpp.first), std::make_unique<rules::BulbState>(std::move(bulb_state_cpp.second))};
    bulb_states.emplace_back(std::move(bulb_state));
  }
  
  return std::make_unique<std::vector<rules::UniqueBulbState>>(std::move(bulb_states));
}

std::unique_ptr<std::vector<rules::DiscreteValueRuleState>> Intersection_DiscreteValueRuleStates(const Intersection& intersection) {
  std::vector<rules::DiscreteValueRuleState> discrete_value_rule_states;
  const auto discrete_value_rule_states_cpp = intersection.DiscreteValueRuleStates();
  
  if (!discrete_value_rule_states_cpp.has_value()) {
    return nullptr;
  }
  for (const auto& discrete_value_state : discrete_value_rule_states_cpp.value()) {
    rules::DiscreteValueRuleState discrete_value_rule_state{discrete_value_state.first.string(), std::make_unique<rules::DiscreteValueRuleDiscreteValue>(discrete_value_state.second)};
    discrete_value_rule_states.emplace_back(std::move(discrete_value_rule_state));
  }
  return std::make_unique<std::vector<rules::DiscreteValueRuleState>>(std::move(discrete_value_rule_states));
}

MutIntersectionPtr IntersectionBook_GetIntersection(IntersectionBook& intersection_book, const rust::String& intersection_id) {
  return {intersection_book.GetIntersection(Intersection::Id{std::string(intersection_id)})};
}

// IntersectionBook_GetIntersections
std::unique_ptr<std::vector<MutIntersectionPtr>> IntersectionBook_GetIntersections(IntersectionBook& intersection_book) {
  const auto intersections_cpp = intersection_book.GetIntersections();
  std::vector<MutIntersectionPtr> intersections;
  intersections.reserve(intersections_cpp.size());
  for (const auto& intersection : intersections_cpp) {
    intersections.push_back(MutIntersectionPtr{intersection});
  }
  return std::make_unique<std::vector<MutIntersectionPtr>>(std::move(intersections));
}

} // namespace api
} // namespace maliput
