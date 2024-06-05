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

std::unique_ptr<std::vector<ConstBulbGroupPtr>> TrafficLight_bulb_groups(const TrafficLight& traffic_light) {
  const auto bulb_groups_cpp = traffic_light.bulb_groups();
  std::vector<ConstBulbGroupPtr> bulb_groups;
  bulb_groups.reserve(bulb_groups_cpp.size());
  for (const auto bulb_group : bulb_groups_cpp) {
    bulb_groups.push_back({bulb_group});
  }
  return std::make_unique<std::vector<ConstBulbGroupPtr>>(std::move(bulb_groups));
}

const BulbGroup* TrafficLight_GetBulbGroup(const TrafficLight& traffic_light, const rust::String& id) {
  return traffic_light.GetBulbGroup(BulbGroup::Id{std::string(id)});
}

std::unique_ptr<UniqueBulbId> Bulb_unique_id(const Bulb& bulb) {
  return std::make_unique<UniqueBulbId>(bulb.unique_id());
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

std::unique_ptr<std::vector<BulbState>> Bulb_states(const Bulb& bulb) {
  const auto states_cpp = bulb.states();
  std::vector<BulbState> states;
  states.reserve(states_cpp.size());
  for (const auto state : states_cpp) {
    states.push_back(state);
  }
  return std::make_unique<std::vector<BulbState>>(std::move(states));
}

std::unique_ptr<maliput::math::Vector3> Bulb_bounding_box_min(const Bulb& bulb) {
  return std::make_unique<maliput::math::Vector3>(bulb.bounding_box().p_BMin);
}

std::unique_ptr<maliput::math::Vector3> Bulb_bounding_box_max(const Bulb& bulb) {
  return std::make_unique<maliput::math::Vector3>(bulb.bounding_box().p_BMax);
}

const BulbGroup* Bulb_bulb_group(const Bulb& bulb) {
  return bulb.bulb_group();
}

rust::String BulbGroup_id(const BulbGroup& bulb_group) {
  return bulb_group.id().string();
}

std::unique_ptr<UniqueBulbGroupId> BulbGroup_unique_id(const BulbGroup& bulb_group) {
  return std::make_unique<UniqueBulbGroupId>(bulb_group.unique_id());
}

std::unique_ptr<InertialPosition> BulbGroup_position_traffic_light(const BulbGroup& bulb_group) {
  return std::make_unique<InertialPosition>(bulb_group.position_traffic_light());
}

std::unique_ptr<Rotation> BulbGroup_orientation_traffic_light(const BulbGroup& bulb_group) {
  return std::make_unique<Rotation>(bulb_group.orientation_traffic_light());
}

std::unique_ptr<std::vector<ConstBulbPtr>> BulbGroup_bulbs(const BulbGroup& bulb_group) {
  const auto bulbs_cpp = bulb_group.bulbs();
  std::vector<ConstBulbPtr> bulbs;
  bulbs.reserve(bulbs_cpp.size());
  for (const auto bulb : bulbs_cpp) {
    bulbs.push_back({bulb});
  }
  return std::make_unique<std::vector<ConstBulbPtr>>(std::move(bulbs));
}

const Bulb* BulbGroup_GetBulb(const BulbGroup& bulb_group, const rust::String& id) {
  return bulb_group.GetBulb(Bulb::Id{std::string(id)});
}

const TrafficLight* BulbGroup_traffic_light(const BulbGroup& bulb_group) {
  return bulb_group.traffic_light();
}

rust::String UniqueBulbId_traffic_light_id(const UniqueBulbId& id) {
  return id.traffic_light_id().string();
}

rust::String UniqueBulbId_bulb_group_id(const UniqueBulbId& id) {
  return id.bulb_group_id().string();
}

rust::String UniqueBulbId_bulb_id(const UniqueBulbId& id) {
  return id.bulb_id().string();
}

rust::String UniqueBulbGroupId_traffic_light_id(const UniqueBulbGroupId& id) {
  return id.traffic_light_id().string();
}

rust::String UniqueBulbGroupId_bulb_group_id(const UniqueBulbGroupId& id) {
  return id.bulb_group_id().string();
}

rust::String DiscreteValueRuleDiscreteValue_value(const DiscreteValueRuleDiscreteValue& discrete_value) {
  return rust::String(discrete_value.value);
}

rust::i32 DiscreteValueRuleDiscreteValue_severity(const DiscreteValueRuleDiscreteValue& discrete_value) {
  return discrete_value.severity;
}

std::unique_ptr<std::vector<RelatedRule>> DiscreteValueRuleDiscreteValue_related_rules(const DiscreteValueRuleDiscreteValue& discrete_value) {
  std::vector<RelatedRule> related_rules;
  for (const auto& related_rule : discrete_value.related_rules) {
    rust::Vec<rust::String> rule_ids;
    for (const auto& rule_id : related_rule.second) {
      rule_ids.push_back({rule_id.string()});
    }
    related_rules.push_back({related_rule.first, rule_ids});
  }
  return std::make_unique<std::vector<RelatedRule>>(std::move(related_rules));
}

std::unique_ptr<std::vector<RelatedUniqueId>> DiscreteValueRuleDiscreteValue_related_unique_ids(const DiscreteValueRuleDiscreteValue& discrete_value) {
  std::vector<RelatedUniqueId> related_unique_ids;
  for (const auto& related_unique_id : discrete_value.related_unique_ids) {
    rust::Vec<rust::String> unique_ids;
    for (const auto& rule_id : related_unique_id.second) {
      unique_ids.push_back({rule_id.string()});
    }
    related_unique_ids.push_back({related_unique_id.first, unique_ids});
  }
  return std::make_unique<std::vector<RelatedUniqueId>>(std::move(related_unique_ids));
}

rust::String DiscreteValueRule_id(const DiscreteValueRule& rule) {
  return rule.id().string();
}

rust::String DiscreteValueRule_type_id(const DiscreteValueRule& rule) {
  return rule.type_id().string();
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
