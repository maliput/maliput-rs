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
#include <vector>

#include <maliput/api/rules/traffic_lights.h>
#include <maliput/api/rules/traffic_light_book.h>
#include <maliput/math/vector.h>

#include <rust/cxx.h>

#include "maliput-sys/src/api/rules/mod.rs.h"

namespace maliput {
namespace api {
namespace rules {

std::unique_ptr<std::vector<ConstTrafficLightPtr>> TrafficLightBook_TrafficLights(const TrafficLightBook& traffic_light_book) {
  const auto traffic_lights_cpp = traffic_light_book.TrafficLights();
  std::vector<ConstTrafficLightPtr> traffic_lights;
  traffic_lights.reserve(traffic_lights_cpp.size());
  for (const auto traffic_light : traffic_lights_cpp) {
    traffic_lights.push_back({traffic_light});
  }
  return std::make_unique<std::vector<ConstTrafficLightPtr>>(std::move(traffic_lights));
}

const TrafficLight* TrafficLightBook_GetTrafficLight(const TrafficLightBook& traffic_light_book, const rust::String& id) {
  return traffic_light_book.GetTrafficLight(TrafficLight::Id{std::string(id)});
}

rust::String TrafficLight_id(const TrafficLight& traffic_light) {
  return traffic_light.id().string();
}

std::unique_ptr<maliput::api::InertialPosition> TrafficLight_position_road_network(const TrafficLight& traffic_light) {
  return std::make_unique<maliput::api::InertialPosition>(traffic_light.position_road_network());
}

std::unique_ptr<maliput::api::Rotation> TrafficLight_orientation_road_network(const TrafficLight& traffic_light) {
  return std::make_unique<maliput::api::Rotation>(traffic_light.orientation_road_network());
}

rust::String Bulb_id(const Bulb& bulb) {
  return bulb.id().string();
}

std::unique_ptr<maliput::api::InertialPosition> Bulb_position_bulb_group(const Bulb& bulb) {
  return std::make_unique<maliput::api::InertialPosition>(bulb.position_bulb_group());
}

std::unique_ptr<maliput::api::Rotation> Bulb_orientation_bulb_group(const Bulb& bulb) {
  return std::make_unique<maliput::api::Rotation>(bulb.orientation_bulb_group());
}

const BulbType& Bulb_type(const Bulb& bulb) {
  return bulb.type();
}

std::unique_ptr<FloatWrapper> Bulb_arrow_orientation_rad(const Bulb& bulb) {
  const auto orientation = bulb.arrow_orientation_rad();
  return orientation.has_value() ? std::make_unique<FloatWrapper>(FloatWrapper{orientation.value()}) : nullptr;
}

std::unique_ptr<std::vector<ConstBulbStateRef>> Bulb_states(const Bulb& bulb) {
  const auto states_cpp = bulb.states();
  std::vector<ConstBulbStateRef> states;
  states.reserve(states_cpp.size());
  for (const auto state : states_cpp) {
    states.push_back({state});
  }
  return std::make_unique<std::vector<ConstBulbStateRef>>(std::move(states));
}

std::unique_ptr<maliput::math::Vector3> Bulb_bounding_box_min(const Bulb& bulb) {
  return std::make_unique<maliput::math::Vector3>(bulb.bounding_box().p_BMin);
}

std::unique_ptr<maliput::math::Vector3> Bulb_bounding_box_max(const Bulb& bulb) {
  return std::make_unique<maliput::math::Vector3>(bulb.bounding_box().p_BMax);
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
