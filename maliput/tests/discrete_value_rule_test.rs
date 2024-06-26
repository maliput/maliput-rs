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
fn discrete_value_rule_test_api() {
    static TOLERANCE: f64 = 1e-9;
    use maliput::api::rules::RuleState;

    let road_network = common::create_loop_road_pedestrian_crosswalk_road_network_with_books();

    let book = road_network.rulebook();
    let expected_rule_id = String::from("Right-Of-Way Rule Type/WestToEastSouth");
    let expected_type_id = String::from("Right-Of-Way Rule Type");
    let rule = book.get_discrete_value_rule(&expected_rule_id);
    assert_eq!(rule.id(), expected_rule_id);
    assert_eq!(rule.type_id(), expected_type_id);
    let zone = rule.zone();
    let expected_zone_length = 15.;
    assert!((zone.length() - expected_zone_length).abs() < TOLERANCE);

    let states = rule.states();
    assert_eq!(states.len(), 2); // Go and Stop

    // Find DiscreteValue with value "Go"
    let expected_value = "Go";
    let go_discrete_value = states.iter().find(|state| state.value() == expected_value).unwrap();
    assert_eq!(go_discrete_value.value(), expected_value);

    let severity = go_discrete_value.severity();
    assert_eq!(severity, 0);

    let related_rules = go_discrete_value.related_rules();
    assert_eq!(related_rules.len(), 1);
    let related_rule_group = String::from("Vehicle-Stop-In-Zone-Behavior Rule Type");
    let related_rules_for_vehicle_stop_behavior_group = related_rules
        .get(&related_rule_group)
        .expect("Related rule group not found");
    assert_eq!(related_rules_for_vehicle_stop_behavior_group.len(), 1);
    assert_eq!(
        related_rules_for_vehicle_stop_behavior_group
            .first()
            .expect("Related rule not found"),
        "Vehicle-Stop-In-Zone-Behavior Rule Type/WestToEastSouth"
    );

    let related_unique_ids = go_discrete_value.related_unique_ids();
    assert_eq!(related_unique_ids.len(), 1);
    let related_unique_id_group = String::from("Bulb Group");
    let related_unique_ids_for_bulb_group = related_unique_ids
        .get(&related_unique_id_group)
        .expect("Related unique id group not found");
    assert_eq!(related_unique_ids_for_bulb_group.len(), 1);
    assert_eq!(
        related_unique_ids_for_bulb_group
            .first()
            .expect("Related unique id not found"),
        "WestFacingSouth-WestFacingBulbsSouth"
    );

    // Find DiscreteValue with value "Stop"
    let expected_value = "Stop";
    let stop_discrete_value = states.iter().find(|state| state.value() == expected_value).unwrap();
    assert_eq!(stop_discrete_value.value(), expected_value);

    let severity = go_discrete_value.severity();
    assert_eq!(severity, 0);

    let related_rules = stop_discrete_value.related_rules();
    assert_eq!(related_rules.len(), 1);
    let related_rule_group = String::from("Vehicle-Stop-In-Zone-Behavior Rule Type");
    let related_rules_for_vehicle_stop_behavior_group = related_rules
        .get(&related_rule_group)
        .expect("Related rule group not found");
    assert_eq!(related_rules_for_vehicle_stop_behavior_group.len(), 1);
    assert_eq!(
        related_rules_for_vehicle_stop_behavior_group
            .first()
            .expect("Related rule not found"),
        "Vehicle-Stop-In-Zone-Behavior Rule Type/WestToEastSouth"
    );

    let related_unique_ids = stop_discrete_value.related_unique_ids();
    assert_eq!(related_unique_ids.len(), 1);
    let related_unique_id_group = String::from("Bulb Group");
    let related_unique_ids_for_bulb_group = related_unique_ids
        .get(&related_unique_id_group)
        .expect("Related unique id group not found");
    assert_eq!(related_unique_ids_for_bulb_group.len(), 1);
    assert_eq!(
        related_unique_ids_for_bulb_group
            .first()
            .expect("Related unique id not found"),
        "WestFacingSouth-WestFacingBulbsSouth"
    );
}
