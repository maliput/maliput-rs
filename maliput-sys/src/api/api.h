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

std::unique_ptr<maliput::math::Vector3> LanePosition_srh(const LanePosition& lane_pos) {
  return std::make_unique<math::Vector3>(lane_pos.srh());
}

void LanePosition_set_srh(LanePosition& lane_pos, const maliput::math::Vector3& srh) {
  lane_pos.set_srh(srh);
}

} // namespace api
} // namespace maliput
