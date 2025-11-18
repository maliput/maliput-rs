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
#include <optional>
#include <vector>

#include <maliput/api/rules/discrete_value_rule.h>
#include <maliput/api/rules/phase.h>
#include <maliput/api/rules/phase_ring.h>
#include <maliput/api/rules/phase_ring_book.h>
#include <maliput/api/rules/range_value_rule.h>
#include <maliput/api/rules/rule_registry.h>
#include <maliput/api/rules/traffic_lights.h>
#include <maliput/api/rules/traffic_light_book.h>
#include <maliput/math/vector.h>

#include <rust/cxx.h>

#include "maliput-sys/src/api/rules/mod.rs.h"

namespace maliput {
namespace api {
namespace rules {
namespace {

std::unique_ptr<std::vector<RelatedRule>> related_rules_from_state(const Rule::State& rule_state) {
  std::vector<RelatedRule> related_rules;
  for (const auto& related_rule : rule_state.related_rules) {
    rust::Vec<rust::String> rule_ids;
    for (const auto& rule_id : related_rule.second) {
      rule_ids.push_back({rule_id.string()});
    }
    related_rules.push_back({related_rule.first, rule_ids});
  }
  return std::make_unique<std::vector<RelatedRule>>(std::move(related_rules));
}

std::unique_ptr<std::vector<RelatedUniqueId>> related_unique_ids_from_state(const Rule::State& rule_state) {
  std::vector<RelatedUniqueId> related_unique_ids;
  for (const auto& related_unique_id : rule_state.related_unique_ids) {
    rust::Vec<rust::String> unique_ids;
    for (const auto& rule_id : related_unique_id.second) {
      unique_ids.push_back({rule_id.string()});
    }
    related_unique_ids.push_back({related_unique_id.first, unique_ids});
  }
  return std::make_unique<std::vector<RelatedUniqueId>>(std::move(related_unique_ids));
}

} // namespace

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

std::unique_ptr<DiscreteValueRule> RoadRulebook_GetDiscreteValueRule(const RoadRulebook& road_rulebook, const rust::String& id) {
  const std::optional<DiscreteValueRule> rule = road_rulebook.GetDiscreteValueRule(Rule::Id{std::string(id)});
  if (rule.has_value()) {
    return std::make_unique<DiscreteValueRule>(rule.value());
  }
  return nullptr;
}

std::unique_ptr<std::vector<RelatedRule>> DiscreteValueRuleDiscreteValue_related_rules(const DiscreteValueRuleDiscreteValue& discrete_value) {
  return related_rules_from_state(discrete_value);
}

std::unique_ptr<std::vector<RelatedUniqueId>> DiscreteValueRuleDiscreteValue_related_unique_ids(const DiscreteValueRuleDiscreteValue& discrete_value) {
  return related_unique_ids_from_state(discrete_value);
}

rust::String DiscreteValueRule_id(const DiscreteValueRule& rule) {
  return rule.id().string();
}

rust::String DiscreteValueRule_type_id(const DiscreteValueRule& rule) {
  return rule.type_id().string();
}

std::unique_ptr<LaneSRoute> DiscreteValueRule_zone(const DiscreteValueRule& rule) {
  return std::make_unique<LaneSRoute>(rule.zone());
}

rust::String RangeValueRuleRange_description(const RangeValueRuleRange& range) {
  return rust::String(range.description);
}

rust::f64 RangeValueRuleRange_min(const RangeValueRuleRange& range) {
  return range.min;
}

rust::f64 RangeValueRuleRange_max(const RangeValueRuleRange& range) {
  return range.max;
}

rust::i32 RangeValueRuleRange_severity(const RangeValueRuleRange& range) {
  return range.severity;
}

std::unique_ptr<std::vector<RelatedRule>> RangeValueRuleRange_related_rules(const RangeValueRuleRange& range) {
  return related_rules_from_state(range);
}

std::unique_ptr<std::vector<RelatedUniqueId>> RangeValueRuleRange_related_unique_ids(const RangeValueRuleRange& range) {
  return related_unique_ids_from_state(range);
}

rust::String RangeValueRule_id(const RangeValueRule& rule) {
  return rule.id().string();
}

rust::String RangeValueRule_type_id(const RangeValueRule& rule) {
  return rule.type_id().string();
}

std::unique_ptr<LaneSRoute> RangeValueRule_zone(const RangeValueRule& rule) {
  return std::make_unique<LaneSRoute>(rule.zone());
}

std::unique_ptr<RangeValueRule> RoadRulebook_GetRangeValueRule(const RoadRulebook& road_rulebook, const rust::String& id) {
  const std::optional<RangeValueRule> rule = road_rulebook.GetRangeValueRule(Rule::Id{std::string(id)});
  if (rule.has_value()) {
    return std::make_unique<RangeValueRule>(rule.value());
  }
  return nullptr;
}

std::unique_ptr<QueryResults> RoadRulebook_Rules(const RoadRulebook& road_rulebook) {
  return std::make_unique<QueryResults>(road_rulebook.Rules());
}

std::unique_ptr<QueryResults> RoadRulebook_FindRules(const RoadRulebook& road_rulebook, const rust::Vec<ConstLaneSRangeRef>& ranges, double tolerance) {
  std::vector<LaneSRange> ranges_cpp;
  for (const auto& range : ranges) {
    ranges_cpp.push_back(range.lane_s_range);
  }
  return std::make_unique<QueryResults>(road_rulebook.FindRules(ranges_cpp, tolerance));
}

rust::Vec<rust::String> QueryResults_discrete_value_rules(const QueryResults& query_results) {
  const auto discrete_value_rules_cpp = query_results.discrete_value_rules;
  rust::Vec<rust::String> discrete_value_rules_id;
  discrete_value_rules_id.reserve(discrete_value_rules_cpp.size());
  for (const auto& discrete_value_rule : discrete_value_rules_cpp) {
    discrete_value_rules_id.push_back({discrete_value_rule.first.string()});
  }
  return discrete_value_rules_id;
}

rust::Vec<rust::String> QueryResults_range_value_rules(const QueryResults& query_results) {
  const auto range_value_rules_cpp = query_results.range_value_rules;
  rust::Vec<rust::String> range_value_rules_id;
  range_value_rules_id.reserve(range_value_rules_cpp.size());
  for (const auto& range_value_rule : range_value_rules_cpp) {
    range_value_rules_id.push_back({range_value_rule.first.string()});
  }
  return range_value_rules_id;
}

rust::String Phase_id(const Phase& phase) {
  return phase.id().string();
}

std::unique_ptr<std::vector<DiscreteValueRuleState>> Phase_discrete_value_rule_states(const Phase& phase) {
  const auto& discrete_value_rule_states_cpp = phase.discrete_value_rule_states();
  std::vector<DiscreteValueRuleState> discrete_value_rule_states;
  discrete_value_rule_states.reserve(discrete_value_rule_states_cpp.size());
  for (const auto& discrete_value_rule_state_cpp : discrete_value_rule_states_cpp) {
    DiscreteValueRuleState discrete_value_rule_state{discrete_value_rule_state_cpp.first.string(), std::make_unique<DiscreteValueRuleDiscreteValue>(discrete_value_rule_state_cpp.second)};
    discrete_value_rule_states.push_back(std::move(discrete_value_rule_state));
  }
  return std::make_unique<std::vector<DiscreteValueRuleState>>(std::move(discrete_value_rule_states));
}

std::unique_ptr<std::vector<UniqueBulbId>> Phase_unique_bulb_ids(const Phase& phase) {
  const auto& bulb_states = phase.bulb_states();
  if (!bulb_states.has_value()) {
    return nullptr;
  }
  std::vector<UniqueBulbId> bulbs;
  bulbs.reserve(bulb_states.value().size());
  for (const auto& bulb_state : bulb_states.value()) {
    bulbs.push_back(bulb_state.first);
  }
  return std::make_unique<std::vector<UniqueBulbId>>(std::move(bulbs));
}

std::unique_ptr<BulbState> Phase_bulb_state(const Phase& phase, const UniqueBulbId& bulb_id) {
  const auto& bulb_states = phase.bulb_states();
  if (!bulb_states.has_value()) {
    return nullptr;
  }
  return std::make_unique<BulbState>(bulb_states.value().at(bulb_id));
}

std::unique_ptr<UniqueBulbId> ptr_from_unique_bulb_id(const UniqueBulbId& unique_bulb_id) {
  return std::make_unique<UniqueBulbId>(unique_bulb_id);
}

rust::String PhaseRing_id(const PhaseRing& phase_ring) {
  return phase_ring.id().string();
}

std::unique_ptr<Phase> PhaseRing_GetPhase(const PhaseRing& phase_ring, const rust::String& id) {
  const auto phase = phase_ring.GetPhase(Phase::Id{std::string(id)});
  if (phase.has_value()) {
    return std::make_unique<Phase>(phase.value());
  }
  return nullptr;
}

rust::Vec<rust::String> PhaseRing_phases_ids(const PhaseRing& phase_ring) {
  const auto phases_cpp = phase_ring.phases();
  rust::Vec<rust::String> phases_ids;
  for (const auto& phase_pair : phases_cpp) {
    phases_ids.push_back({phase_pair.first.string()});
  }
  return phases_ids;
}

std::unique_ptr<std::vector<NextPhase>> PhaseRing_GetNextPhases(const PhaseRing& phase_ring, const rust::String& id) {
  const auto next_phases_cpp = phase_ring.GetNextPhases(Phase::Id{std::string(id)});
  std::vector<NextPhase> next_phases;
  next_phases.reserve(next_phases_cpp.size());
  for (const auto& next_phase_cpp : next_phases_cpp) {
    std::unique_ptr<FloatWrapper> duration_until = nullptr;
    if (next_phase_cpp.duration_until.has_value()) {
      duration_until = std::make_unique<FloatWrapper>(FloatWrapper{next_phase_cpp.duration_until.value()});
    }
    next_phases.push_back({next_phase_cpp.id.string(), std::move(duration_until)});
  }
  return std::make_unique<std::vector<NextPhase>>(std::move(next_phases));
}

std::unique_ptr<PhaseStateProvider> PhaseProvider_GetPhase(const PhaseProvider& phase_provider, const rust::String& phase_ring_id) {
  const auto phase_ring = phase_provider.GetPhase(PhaseRing::Id{std::string(phase_ring_id)});
  if (!phase_ring.has_value()) {
    return nullptr;
  }
  std::unique_ptr<PhaseStateProvider> state_provider = std::make_unique<PhaseStateProvider>(std::move(phase_ring.value()));
  return state_provider;
}

rust::Vec<rust::String> PhaseRingBook_GetPhaseRingsId(const PhaseRingBook& phase_ring_book) {
  const auto phase_rings_cpp = phase_ring_book.GetPhaseRings();
  rust::Vec<rust::String> phase_rings;
  for (const auto& phase_ring_id : phase_rings_cpp) {
    phase_rings.push_back({phase_ring_id.string()});
  }
  return phase_rings;
}

std::unique_ptr<PhaseRing> PhaseRingBook_GetPhaseRing(const PhaseRingBook& phase_ring_book, const rust::String& id) {
  const auto phase_ring = phase_ring_book.GetPhaseRing(PhaseRing::Id{std::string(id)});
  if (phase_ring.has_value()) {
    return std::make_unique<PhaseRing>(phase_ring.value());
  }
  return nullptr;
}

std::unique_ptr<PhaseRing> PhaseRingBook_FindPhaseRing(const PhaseRingBook& phase_ring_book, const rust::String& rule_id) {
  const auto phase_ring = phase_ring_book.FindPhaseRing(Rule::Id{std::string(rule_id)});
  if (phase_ring.has_value()) {
    return std::make_unique<PhaseRing>(phase_ring.value());
  }
  return nullptr;
}

std::unique_ptr<std::vector<DiscreteValueRuleType>> RuleRegistry_DiscreteValueRuleTypes(const RuleRegistry& rule_registry) {
  const auto discrete_value_rule_types_cpp = rule_registry.DiscreteValueRuleTypes();
  std::vector<DiscreteValueRuleType> discrete_value_rule_types;
  discrete_value_rule_types.reserve(discrete_value_rule_types_cpp.size());
  for (const auto& discrete_value_rule_type : discrete_value_rule_types_cpp) {
    std::vector<DiscreteValueRuleDiscreteValue> discrete_values;
    for (const auto& discrete_value : discrete_value_rule_type.second) {
      discrete_values.push_back(discrete_value);
    }
    DiscreteValueRuleType rule_type{discrete_value_rule_type.first.string(), std::make_unique<std::vector<DiscreteValueRuleDiscreteValue>>(std::move(discrete_values))};
    discrete_value_rule_types.push_back(std::move(rule_type));
  }
  return std::make_unique<std::vector<DiscreteValueRuleType>>(std::move(discrete_value_rule_types));
}

std::unique_ptr<std::vector<RangeValueRuleType>> RuleRegistry_RangeValueRuleTypes(const RuleRegistry& rule_registry) {
  const auto range_value_rule_types_cpp = rule_registry.RangeValueRuleTypes();
  std::vector<RangeValueRuleType> range_value_rule_types;
  range_value_rule_types.reserve(range_value_rule_types_cpp.size());
  for (const auto& range_value_rule_type : range_value_rule_types_cpp) {
    std::vector<RangeValueRuleRange> range_values;
    for (const auto& range_value : range_value_rule_type.second) {
      range_values.push_back(range_value);
    }
    RangeValueRuleType rule_type{range_value_rule_type.first.string(), std::make_unique<std::vector<RangeValueRuleRange>>(std::move(range_values))};
    range_value_rule_types.push_back(std::move(rule_type));
  }
  return std::make_unique<std::vector<RangeValueRuleType>>(std::move(range_value_rule_types));
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
