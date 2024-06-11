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
mod common;

// LoopRoadPedestrianCrosswalk map is being used to test the discrete value rule API and its components.
// YAML information about the RoadRulebook can be found at:
// https://github.com/maliput/maliput_malidrive/blob/352601969b1363cc13fe2008c198a3d95843bf5b/resources/LoopRoadPedestrianCrosswalk.yaml#L64

#[test]
fn range_value_rule_test_api() {
    use maliput::api::rules::RuleState;

    let road_network = common::create_loop_road_pedestrian_crosswalk_road_network_with_books();

    let book = road_network.rulebook();
    let expected_rule_id = String::from("Speed-Limit Rule Type/1_0_1_1");
    let expected_type_id = String::from("Speed-Limit Rule Type");
    let rule = book.get_range_value_rule(&expected_rule_id);
    assert_eq!(rule.id(), expected_rule_id);
    assert_eq!(rule.type_id(), expected_type_id);

    let states = rule.states();
    assert_eq!(states.len(), 1); // Only one speed limit state
    let speed_limit_state = states.first().expect("State not found");
    assert_eq!(speed_limit_state.description(), "m/s");
    assert_eq!(speed_limit_state.min(), 0.);
    assert_eq!(speed_limit_state.max(), 11.11111111111111);

    let severity = speed_limit_state.severity();
    assert_eq!(severity, 0);

    let related_rules = speed_limit_state.related_rules();
    assert_eq!(related_rules.len(), 0);
    let related_unique_ids = speed_limit_state.related_unique_ids();
    assert_eq!(related_unique_ids.len(), 0);
}
