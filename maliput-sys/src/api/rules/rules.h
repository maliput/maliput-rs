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

// This header contains function declarations for the CXX bridge.
// Implementations are in rules_impl.cc to avoid circular dependency with mod.rs.h.

#pragma once

#include <memory>
#include <optional>
#include <vector>

#include <maliput/api/intersection.h>
#include <maliput/api/rules/discrete_value_rule.h>
#include <maliput/api/rules/discrete_value_rule_state_provider.h>
#include <maliput/api/rules/phase.h>
#include <maliput/api/rules/phase_ring.h>
#include <maliput/api/rules/phase_ring_book.h>
#include <maliput/api/rules/range_value_rule.h>
#include <maliput/api/rules/range_value_rule_state_provider.h>
#include <maliput/api/rules/rule_registry.h>
#include <maliput/api/rules/traffic_lights.h>
#include <maliput/api/rules/traffic_light_book.h>
#include <maliput/math/vector.h>

#include <rust/cxx.h>

#include "api/rules/aliases.h"

namespace maliput {
namespace api {
namespace rules {

// Forward declarations for CXX shared structs.
// These are defined in the CXX-generated mod.rs.h header.
// NOTE: Do NOT forward declare types from aliases.h (DiscreteValueRuleDiscreteValue,
// PhaseStateProviderQuery, etc.) - they are type aliases, not structs.
struct ConstTrafficLightPtr;
struct ConstBulbGroupPtr;
struct ConstBulbPtr;
struct FloatWrapper;
struct RelatedRule;
struct RelatedUniqueId;
struct DiscreteValueRuleState;
struct NextPhase;
struct DiscreteValueRuleType;
struct RangeValueRuleType;
struct DiscreteValueNextState;
struct RangeValueNextState;
struct ConstLaneSRangeRef;

// Function declarations - implementations are in rules.cc

std::unique_ptr<std::vector<ConstTrafficLightPtr>> TrafficLightBook_TrafficLights(const TrafficLightBook& traffic_light_book);
const TrafficLight* TrafficLightBook_GetTrafficLight(const TrafficLightBook& traffic_light_book, const rust::String& id);

rust::String TrafficLight_id(const TrafficLight& traffic_light);
std::unique_ptr<maliput::api::InertialPosition> TrafficLight_position_road_network(const TrafficLight& traffic_light);
std::unique_ptr<maliput::api::Rotation> TrafficLight_orientation_road_network(const TrafficLight& traffic_light);
std::unique_ptr<std::vector<ConstBulbGroupPtr>> TrafficLight_bulb_groups(const TrafficLight& traffic_light);
const BulbGroup* TrafficLight_GetBulbGroup(const TrafficLight& traffic_light, const rust::String& id);

std::unique_ptr<UniqueBulbId> Bulb_unique_id(const Bulb& bulb);
rust::String Bulb_id(const Bulb& bulb);
std::unique_ptr<maliput::api::InertialPosition> Bulb_position_bulb_group(const Bulb& bulb);
std::unique_ptr<maliput::api::Rotation> Bulb_orientation_bulb_group(const Bulb& bulb);
const BulbType& Bulb_type(const Bulb& bulb);
std::unique_ptr<FloatWrapper> Bulb_arrow_orientation_rad(const Bulb& bulb);
std::unique_ptr<std::vector<BulbState>> Bulb_states(const Bulb& bulb);
std::unique_ptr<maliput::math::Vector3> Bulb_bounding_box_min(const Bulb& bulb);
std::unique_ptr<maliput::math::Vector3> Bulb_bounding_box_max(const Bulb& bulb);
const BulbGroup* Bulb_bulb_group(const Bulb& bulb);

rust::String BulbGroup_id(const BulbGroup& bulb_group);
std::unique_ptr<UniqueBulbGroupId> BulbGroup_unique_id(const BulbGroup& bulb_group);
std::unique_ptr<InertialPosition> BulbGroup_position_traffic_light(const BulbGroup& bulb_group);
std::unique_ptr<Rotation> BulbGroup_orientation_traffic_light(const BulbGroup& bulb_group);
std::unique_ptr<std::vector<ConstBulbPtr>> BulbGroup_bulbs(const BulbGroup& bulb_group);
const Bulb* BulbGroup_GetBulb(const BulbGroup& bulb_group, const rust::String& id);
const TrafficLight* BulbGroup_traffic_light(const BulbGroup& bulb_group);

rust::String UniqueBulbId_traffic_light_id(const UniqueBulbId& id);
rust::String UniqueBulbId_bulb_group_id(const UniqueBulbId& id);
rust::String UniqueBulbId_bulb_id(const UniqueBulbId& id);

rust::String UniqueBulbGroupId_traffic_light_id(const UniqueBulbGroupId& id);
rust::String UniqueBulbGroupId_bulb_group_id(const UniqueBulbGroupId& id);

std::unique_ptr<UniqueBulbId> UniqueBulbId_create_unique_ptr(const UniqueBulbId& id);

rust::String DiscreteValueRuleDiscreteValue_value(const DiscreteValueRuleDiscreteValue& discrete_value);
rust::i32 DiscreteValueRuleDiscreteValue_severity(const DiscreteValueRuleDiscreteValue& discrete_value);
std::unique_ptr<std::vector<RelatedRule>> DiscreteValueRuleDiscreteValue_related_rules(const DiscreteValueRuleDiscreteValue& discrete_value);
std::unique_ptr<std::vector<RelatedUniqueId>> DiscreteValueRuleDiscreteValue_related_unique_ids(const DiscreteValueRuleDiscreteValue& discrete_value);

std::unique_ptr<DiscreteValueRule> RoadRulebook_GetDiscreteValueRule(const RoadRulebook& road_rulebook, const rust::String& id);
std::unique_ptr<RangeValueRule> RoadRulebook_GetRangeValueRule(const RoadRulebook& road_rulebook, const rust::String& id);
std::unique_ptr<QueryResults> RoadRulebook_Rules(const RoadRulebook& road_rulebook);
std::unique_ptr<QueryResults> RoadRulebook_FindRules(const RoadRulebook& road_rulebook, const rust::Vec<ConstLaneSRangeRef>& ranges, double tolerance);

rust::String DiscreteValueRule_id(const DiscreteValueRule& rule);
rust::String DiscreteValueRule_type_id(const DiscreteValueRule& rule);
std::unique_ptr<LaneSRoute> DiscreteValueRule_zone(const DiscreteValueRule& rule);

rust::String RangeValueRuleRange_description(const RangeValueRuleRange& range);
rust::f64 RangeValueRuleRange_min(const RangeValueRuleRange& range);
rust::f64 RangeValueRuleRange_max(const RangeValueRuleRange& range);
rust::i32 RangeValueRuleRange_severity(const RangeValueRuleRange& range);
std::unique_ptr<std::vector<RelatedRule>> RangeValueRuleRange_related_rules(const RangeValueRuleRange& range);
std::unique_ptr<std::vector<RelatedUniqueId>> RangeValueRuleRange_related_unique_ids(const RangeValueRuleRange& range);

rust::String RangeValueRule_id(const RangeValueRule& rule);
rust::String RangeValueRule_type_id(const RangeValueRule& rule);
std::unique_ptr<LaneSRoute> RangeValueRule_zone(const RangeValueRule& rule);

rust::Vec<rust::String> QueryResults_discrete_value_rules(const QueryResults& query_results);
rust::Vec<rust::String> QueryResults_range_value_rules(const QueryResults& query_results);

rust::String Phase_id(const Phase& phase);
std::unique_ptr<std::vector<DiscreteValueRuleState>> Phase_discrete_value_rule_states(const Phase& phase);
std::unique_ptr<std::vector<UniqueBulbId>> Phase_unique_bulb_ids(const Phase& phase);
std::unique_ptr<BulbState> Phase_bulb_state(const Phase& phase, const UniqueBulbId& bulb_id);

std::unique_ptr<UniqueBulbId> ptr_from_unique_bulb_id(const UniqueBulbId& unique_bulb_id);

rust::String PhaseRing_id(const PhaseRing& phase_ring);
std::unique_ptr<Phase> PhaseRing_GetPhase(const PhaseRing& phase_ring, const rust::String& id);
rust::Vec<rust::String> PhaseRing_phases_ids(const PhaseRing& phase_ring);
std::unique_ptr<std::vector<NextPhase>> PhaseRing_GetNextPhases(const PhaseRing& phase_ring, const rust::String& id);

rust::String PhaseStateProvider_state(const PhaseStateProviderQuery &phase_state_provider);
std::unique_ptr<NextPhase> PhaseStateProvider_next(const PhaseStateProviderQuery &phase_state_provider);

std::unique_ptr<PhaseStateProviderQuery> PhaseProvider_GetPhase(const PhaseProvider& phase_provider, const rust::String& phase_ring_id);

rust::Vec<rust::String> PhaseRingBook_GetPhaseRingsId(const PhaseRingBook& phase_ring_book);
std::unique_ptr<PhaseRing> PhaseRingBook_GetPhaseRing(const PhaseRingBook& phase_ring_book, const rust::String& id);
std::unique_ptr<PhaseRing> PhaseRingBook_FindPhaseRing(const PhaseRingBook& phase_ring_book, const rust::String& rule_id);

std::unique_ptr<std::vector<DiscreteValueRuleType>> RuleRegistry_DiscreteValueRuleTypes(const RuleRegistry& rule_registry);
std::unique_ptr<std::vector<RangeValueRuleType>> RuleRegistry_RangeValueRuleTypes(const RuleRegistry& rule_registry);

std::unique_ptr<DiscreteValueRuleStateProviderQuery> DiscreteValueRuleStateProvider_GetStateById(const DiscreteValueRuleStateProvider& state_provider, const rust::String& rule_id);
std::unique_ptr<DiscreteValueRuleStateProviderQuery> DiscreteValueRuleStateProvider_GetStateByType(const DiscreteValueRuleStateProvider& state_provider, const RoadPosition& road_position, const rust::String& rule_type, double tolerance);

std::unique_ptr<RangeValueRuleStateProviderQuery> RangeValueRuleStateProvider_GetStateById(const RangeValueRuleStateProvider& state_provider, const rust::String& rule_id);
std::unique_ptr<RangeValueRuleStateProviderQuery> RangeValueRuleStateProvider_GetStateByType(const RangeValueRuleStateProvider& state_provider, const RoadPosition& road_position, const rust::String& rule_type, double tolerance);

std::unique_ptr<DiscreteValueRuleDiscreteValue> DiscreteValueRuleStateProviderQuery_state(const DiscreteValueRuleStateProviderQuery& query);
std::unique_ptr<DiscreteValueNextState> DiscreteValueRuleStateProviderQuery_next(const DiscreteValueRuleStateProviderQuery& query);

std::unique_ptr<RangeValueRuleRange> RangeValueRuleStateProviderQuery_state(const RangeValueRuleStateProviderQuery& query);
std::unique_ptr<RangeValueNextState> RangeValueRuleStateProviderQuery_next(const RangeValueRuleStateProviderQuery& query);

}  // namespace rules
}  // namespace api
}  // namespace maliput
