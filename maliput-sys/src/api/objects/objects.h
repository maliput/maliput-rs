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

// This header contains function declarations for the CXX bridge.
// Implementations are in objects.cc to avoid circular dependency with mod.rs.h.

#pragma once

#include <memory>
#include <optional>
#include <vector>

#include <maliput/api/objects/road_object.h>
#include <maliput/api/objects/road_object_book.h>
#include <maliput/math/bounding_box.h>
#include <maliput/math/vector.h>

#include <rust/cxx.h>

// Include the rules bridge header to get the StringWrapper shared struct definition.
#include "maliput-sys/src/api/rules/mod.rs.h"

namespace maliput {
namespace api {
namespace objects {

// Forward declarations for CXX shared structs.
// These are defined in the CXX-generated mod.rs.h header.
struct ConstRoadObjectPtr;
struct ConstOutlinePtr;
struct OutlineCornerData;
struct StringPair;

// RoadObjectBook bridge function declarations.
std::unique_ptr<std::vector<ConstRoadObjectPtr>> RoadObjectBook_RoadObjects(const RoadObjectBook& book);
const RoadObject* RoadObjectBook_GetRoadObject(const RoadObjectBook& book, const rust::String& id);
std::unique_ptr<std::vector<ConstRoadObjectPtr>> RoadObjectBook_FindByType(const RoadObjectBook& book, RoadObjectType obj_type);
std::unique_ptr<std::vector<ConstRoadObjectPtr>> RoadObjectBook_FindByLane(const RoadObjectBook& book, const rust::String& lane_id);
std::unique_ptr<std::vector<ConstRoadObjectPtr>> RoadObjectBook_FindInRadius(const RoadObjectBook& book, rust::f64 x, rust::f64 y, rust::f64 z, rust::f64 radius);

// RoadObject bridge function declarations.
rust::String RoadObject_id(const RoadObject& obj);
std::unique_ptr<maliput::api::rules::StringWrapper> RoadObject_name(const RoadObject& obj);
RoadObjectType RoadObject_object_type(const RoadObject& obj);
std::unique_ptr<maliput::api::rules::StringWrapper> RoadObject_subtype(const RoadObject& obj);
std::unique_ptr<maliput::api::InertialPosition> RoadObject_position_inertial(const RoadObject& obj);
bool RoadObject_position_has_lane_position(const RoadObject& obj);
rust::String RoadObject_position_lane_id(const RoadObject& obj);
rust::f64 RoadObject_position_lane_s(const RoadObject& obj);
rust::f64 RoadObject_position_lane_r(const RoadObject& obj);
rust::f64 RoadObject_position_lane_h(const RoadObject& obj);
std::unique_ptr<maliput::api::Rotation> RoadObject_orientation(const RoadObject& obj);
std::unique_ptr<maliput::math::BoundingBox> RoadObject_bounding_box(const RoadObject& obj);
rust::Vec<rust::String> RoadObject_related_lanes(const RoadObject& obj);
std::unique_ptr<std::vector<ConstOutlinePtr>> RoadObject_outlines(const RoadObject& obj);
rust::Vec<StringPair> RoadObject_properties(const RoadObject& obj);

// Outline bridge function declarations.
rust::String Outline_id(const Outline& outline);
bool Outline_is_closed(const Outline& outline);
rust::i32 Outline_num_corners(const Outline& outline);
rust::Vec<OutlineCornerData> Outline_corners(const Outline& outline);

}  // namespace objects
}  // namespace api
}  // namespace maliput
