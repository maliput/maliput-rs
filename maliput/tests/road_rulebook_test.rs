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

// TShapeRoad maps is being used to test the road rulebook API and its components.
// YAML information about the RoadRulebook can be found at:
// https://github.com/maliput/maliput_malidrive/blob/352601969b1363cc13fe2008c198a3d95843bf5b/resources/LoopRoadPedestrianCrosswalk.yaml#L64

#[test]
fn road_rulebook_test_api() {
    let road_network = common::create_loop_road_pedestrian_crosswalk_road_network_with_books();

    let book = road_network.rulebook();
    // Test get_discrete_value_rule method.
    let expected_rule_id = String::from("Vehicle-Stop-In-Zone-Behavior Rule Type/WestToEastSouth");
    let expected_type_id = String::from("Vehicle-Stop-In-Zone-Behavior Rule Type");
    let rule = book.get_discrete_value_rule(&expected_rule_id);
    assert_eq!(rule.id(), expected_rule_id);
    assert_eq!(rule.type_id(), expected_type_id);

    // Test get_range_value_rule method.
    let expected_rule_id = String::from("Speed-Limit Rule Type/1_0_1_1");
    let expected_type_id = String::from("Speed-Limit Rule Type");
    let rule = book.get_range_value_rule(&expected_rule_id);
    assert_eq!(rule.id(), expected_rule_id);
    assert_eq!(rule.type_id(), expected_type_id);

    // Test rules method.
    let rules = book.rules();
    assert_eq!(rules.discrete_value_rules.len(), 40);
    assert_eq!(rules.range_value_rules.len(), 16);
    let dvr_rule = rules
        .discrete_value_rules
        .get("Vehicle-Stop-In-Zone-Behavior Rule Type/WestToEastSouth");
    assert!(dvr_rule.is_some());
    assert_eq!(
        dvr_rule.unwrap().id(),
        "Vehicle-Stop-In-Zone-Behavior Rule Type/WestToEastSouth"
    );
    let rvr_rule = rules.range_value_rules.get("Speed-Limit Rule Type/1_0_1_1");
    assert!(rvr_rule.is_some());
    assert_eq!(rvr_rule.unwrap().id(), "Speed-Limit Rule Type/1_0_1_1");

    // Test find_rules method.
    let lane_s_range_1 = maliput::api::LaneSRange::new(&String::from("1_0_1"), &maliput::api::SRange::new(0.0, 100.0));
    let lane_s_range_2 = maliput::api::LaneSRange::new(&String::from("2_0_1"), &maliput::api::SRange::new(0.0, 200.0));
    let ranges = vec![lane_s_range_1, lane_s_range_2];
    let rules: maliput::api::rules::QueryResults = book.find_rules(&ranges, 1e-3);
    assert_eq!(rules.discrete_value_rules.len(), 4);
    assert_eq!(rules.range_value_rules.len(), 2);
}
