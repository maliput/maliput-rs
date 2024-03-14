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

#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/road_network.h>
#include <maliput/api/road_geometry.h>

#include <rust/cxx.h>

namespace maliput {
namespace api {

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

rust::String Lane_id(const Lane& lane) {
  return lane.id().string();
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

std::unique_ptr<RoadPositionResult> RoadGeometry_ToRoadPosition(const RoadGeometry& road_geometry, const InertialPosition& inertial_pos) {
  return std::make_unique<RoadPositionResult>(road_geometry.ToRoadPosition(inertial_pos));
}

} // namespace api
} // namespace maliput
