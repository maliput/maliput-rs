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

// This file contains the implementations of the CXX bridge functions for the
// maliput::api::objects module.
// It is compiled separately to avoid circular dependency issues between
// objects.h (function declarations) and the CXX-generated mod.rs.h (shared struct definitions).

#include "maliput-sys/src/api/rules/mod.rs.h"
#include "maliput-sys/src/api/objects/mod.rs.h"

namespace maliput {
namespace api {
namespace objects {

std::unique_ptr<std::vector<ConstRoadObjectPtr>> RoadObjectBook_RoadObjects(const RoadObjectBook& book) {
  const auto road_objects_cpp = book.RoadObjects();
  std::vector<ConstRoadObjectPtr> road_objects;
  road_objects.reserve(road_objects_cpp.size());
  for (const auto road_object : road_objects_cpp) {
    road_objects.push_back({road_object});
  }
  return std::make_unique<std::vector<ConstRoadObjectPtr>>(std::move(road_objects));
}

const RoadObject* RoadObjectBook_GetRoadObject(const RoadObjectBook& book, const rust::String& id) {
  return book.GetRoadObject(RoadObject::Id{std::string(id)});
}

std::unique_ptr<std::vector<ConstRoadObjectPtr>> RoadObjectBook_FindByType(
    const RoadObjectBook& book, RoadObjectType obj_type) {
  const auto road_objects_cpp = book.FindByType(obj_type);
  std::vector<ConstRoadObjectPtr> road_objects;
  road_objects.reserve(road_objects_cpp.size());
  for (const auto road_object : road_objects_cpp) {
    road_objects.push_back({road_object});
  }
  return std::make_unique<std::vector<ConstRoadObjectPtr>>(std::move(road_objects));
}

std::unique_ptr<std::vector<ConstRoadObjectPtr>> RoadObjectBook_FindByLane(
    const RoadObjectBook& book, const rust::String& lane_id) {
  const auto road_objects_cpp = book.FindByLane(maliput::api::LaneId{std::string(lane_id)});
  std::vector<ConstRoadObjectPtr> road_objects;
  road_objects.reserve(road_objects_cpp.size());
  for (const auto road_object : road_objects_cpp) {
    road_objects.push_back({road_object});
  }
  return std::make_unique<std::vector<ConstRoadObjectPtr>>(std::move(road_objects));
}

std::unique_ptr<std::vector<ConstRoadObjectPtr>> RoadObjectBook_FindInRadius(
    const RoadObjectBook& book, rust::f64 x, rust::f64 y, rust::f64 z, rust::f64 radius) {
  const maliput::api::InertialPosition pos{x, y, z};
  const auto road_objects_cpp = book.FindInRadius(pos, radius);
  std::vector<ConstRoadObjectPtr> road_objects;
  road_objects.reserve(road_objects_cpp.size());
  for (const auto road_object : road_objects_cpp) {
    road_objects.push_back({road_object});
  }
  return std::make_unique<std::vector<ConstRoadObjectPtr>>(std::move(road_objects));
}

rust::String RoadObject_id(const RoadObject& obj) {
  return obj.id().string();
}

std::unique_ptr<maliput::api::rules::StringWrapper> RoadObject_name(const RoadObject& obj) {
  if (!obj.name().has_value()) {
    return nullptr;
  }
  return std::make_unique<maliput::api::rules::StringWrapper>(
      maliput::api::rules::StringWrapper{rust::String{*obj.name()}});
}

RoadObjectType RoadObject_object_type(const RoadObject& obj) {
  return obj.type();
}

std::unique_ptr<maliput::api::rules::StringWrapper> RoadObject_subtype(const RoadObject& obj) {
  if (!obj.subtype().has_value()) {
    return nullptr;
  }
  return std::make_unique<maliput::api::rules::StringWrapper>(
      maliput::api::rules::StringWrapper{rust::String{*obj.subtype()}});
}

std::unique_ptr<maliput::api::InertialPosition> RoadObject_position_inertial(const RoadObject& obj) {
  return std::make_unique<maliput::api::InertialPosition>(obj.position().inertial_position());
}

bool RoadObject_position_has_lane_position(const RoadObject& obj) {
  return obj.position().has_lane_position();
}

rust::String RoadObject_position_lane_id(const RoadObject& obj) {
  if (!obj.position().has_lane_position()) {
    return rust::String{};
  }
  return obj.position().lane_id()->string();
}

rust::f64 RoadObject_position_lane_s(const RoadObject& obj) {
  if (!obj.position().has_lane_position()) {
    return 0.0;
  }
  return obj.position().lane_position()->s();
}

rust::f64 RoadObject_position_lane_r(const RoadObject& obj) {
  if (!obj.position().has_lane_position()) {
    return 0.0;
  }
  return obj.position().lane_position()->r();
}

rust::f64 RoadObject_position_lane_h(const RoadObject& obj) {
  if (!obj.position().has_lane_position()) {
    return 0.0;
  }
  return obj.position().lane_position()->h();
}

std::unique_ptr<maliput::api::Rotation> RoadObject_orientation(const RoadObject& obj) {
  return std::make_unique<maliput::api::Rotation>(obj.orientation());
}

std::unique_ptr<maliput::math::BoundingBox> RoadObject_bounding_box(const RoadObject& obj) {
  return std::make_unique<maliput::math::BoundingBox>(obj.bounding_box());
}

rust::Vec<rust::String> RoadObject_related_lanes(const RoadObject& obj) {
  rust::Vec<rust::String> lane_ids;
  for (const auto& lane_id : obj.related_lanes()) {
    lane_ids.push_back(lane_id.string());
  }
  return lane_ids;
}

std::unique_ptr<std::vector<ConstOutlinePtr>> RoadObject_outlines(const RoadObject& obj) {
  std::vector<ConstOutlinePtr> outlines;
  outlines.reserve(obj.num_outlines());
  for (const auto& outline : obj.outlines()) {
    outlines.push_back({outline.get()});
  }
  return std::make_unique<std::vector<ConstOutlinePtr>>(std::move(outlines));
}

rust::Vec<StringPair> RoadObject_properties(const RoadObject& obj) {
  rust::Vec<StringPair> pairs;
  for (const auto& kv : obj.properties()) {
    pairs.push_back({rust::String{kv.first}, rust::String{kv.second}});
  }
  return pairs;
}

rust::String Outline_id(const Outline& outline) {
  return outline.id().string();
}

bool Outline_is_closed(const Outline& outline) {
  return outline.is_closed();
}

rust::i32 Outline_num_corners(const Outline& outline) {
  return outline.num_corners();
}

rust::Vec<OutlineCornerData> Outline_corners(const Outline& outline) {
  rust::Vec<OutlineCornerData> corners;
  for (const auto& corner : outline.corners()) {
    OutlineCornerData data;
    data.x = corner.position().x();
    data.y = corner.position().y();
    data.z = corner.position().z();
    data.has_height = corner.height().has_value();
    data.height = corner.height().value_or(0.0);
    corners.push_back(data);
  }
  return corners;
}

}  // namespace objects
}  // namespace api
}  // namespace maliput
