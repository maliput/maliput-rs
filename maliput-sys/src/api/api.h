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

#include <maliput/api/junction.h>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/road_network.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/segment.h>

#include <rust/cxx.h>

#include "maliput-sys/src/api/mod.rs.h"

namespace maliput {
namespace api {

struct ConstLanePtr;

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

ConstLanePtr RoadGeometry_GetLane(const RoadGeometry& road_geometry, const rust::String& lane_id) {
  return {road_geometry.ById().GetLane(LaneId{std::string(lane_id)})};
}

const std::vector<ConstLanePtr>& RoadGeometry_GetLanes(const RoadGeometry& road_geometry) {
  static std::vector<ConstLanePtr> lanes;
  const auto lanes_cpp = road_geometry.ById().GetLanes();
  if (lanes.size() == lanes_cpp.size()) {
    return lanes;
  }
  lanes.reserve(lanes_cpp.size());
  for (const auto& lane : road_geometry.ById().GetLanes()) {
    lanes.push_back(ConstLanePtr{lane.second});
  }
  return lanes;
}

const Segment* RoadGeometry_GetSegment(const RoadGeometry& road_geometry, const rust::String& segment_id) {
  return road_geometry.ById().GetSegment(SegmentId{std::string(segment_id)});
}

const Junction* RoadGeometry_GetJunction(const RoadGeometry& road_geometry, const rust::String& junction_id) {
  return road_geometry.ById().GetJunction(JunctionId{std::string(junction_id)});
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

} // namespace api
} // namespace maliput
