// BSD 3-Clause License
//
// Copyright (c) 2025, Woven by Toyota.
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

use maliput::api::rules::{RuleType, RuleValueTypes};

mod common;

#[test]
fn rule_registry_test() {
    let road_network = common::create_loop_road_pedestrian_crosswalk_road_network_with_books();

    let rule_registry = road_network.rule_registry();
    let discrete_value_rule_types = rule_registry.get_discrete_value_rule_types();
    assert!(!discrete_value_rule_types.is_empty());
    assert!(discrete_value_rule_types.contains(&RuleType::RightOfWay.to_string()));

    let discrete_values = rule_registry.discrete_values_by_type(discrete_value_rule_types[0].clone());
    assert!(discrete_values.is_some());
    let discrete_values = discrete_values.unwrap();
    assert!(!discrete_values.is_empty());
    let discrete_values = rule_registry.discrete_values_by_type(RuleType::RightOfWay.to_string());
    assert!(discrete_values.is_some());
    let discrete_values = discrete_values.unwrap();
    assert!(!discrete_values.is_empty());

    let range_value_rule_types = rule_registry.get_range_rule_types();
    assert!(!range_value_rule_types.is_empty());
    assert!(range_value_rule_types.contains(&RuleType::SpeedLimit.to_string()));

    let range_values = rule_registry.range_values_by_type(range_value_rule_types[0].clone());
    assert!(range_values.is_some());
    let range_values = range_values.unwrap();
    assert!(!range_values.is_empty());
    let range_values = rule_registry.range_values_by_type(RuleType::SpeedLimit.to_string());
    assert!(range_values.is_some());
    let range_values = range_values.unwrap();
    assert!(!range_values.is_empty());

    let rule_value_types = rule_registry.get_possible_states_of_rule_type(RuleType::RightOfWay.to_string());
    assert!(rule_value_types.is_some());
    let rule_value_types = rule_value_types.unwrap();
    assert!(matches!(rule_value_types, RuleValueTypes::DiscreteValues(_)));
    let rule_value_types = rule_registry.get_possible_states_of_rule_type(RuleType::SpeedLimit.to_string());
    assert!(rule_value_types.is_some());
    let rule_value_types = rule_value_types.unwrap();
    assert!(matches!(rule_value_types, RuleValueTypes::Ranges(_)));

    assert!(rule_registry
        .get_possible_states_of_rule_type("InvalidRuleType".to_string())
        .is_none());
}
