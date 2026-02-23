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

fn main() -> Result<(), Box<dyn std::error::Error>> {
    use maliput::api::{RoadNetwork, RoadNetworkBackend};
    use std::collections::HashMap;

    // Get location of odr resources
    let package_location = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let xodr_path = format!("{}/data/xodr/LoopRoadPedestrianCrosswalk.xodr", package_location);
    let yaml_path = format!("{}/data/xodr/LoopRoadPedestrianCrosswalk.yaml", package_location);
    let road_network_properties = HashMap::from([
        ("road_geometry_id", "my_rg_from_rust"),
        ("opendrive_file", xodr_path.as_str()),
        ("road_rule_book", yaml_path.as_str()),
        ("rule_registry", yaml_path.as_str()),
        ("traffic_light_book", yaml_path.as_str()),
        ("phase_ring_book", yaml_path.as_str()),
        ("intersection_book", yaml_path.as_str()),
        ("linear_tolerance", "0.01"),
    ]);

    let road_network = RoadNetwork::new(RoadNetworkBackend::MaliputMalidrive, &road_network_properties)?;
    let rulebook = road_network.rulebook();
    let all_rules = rulebook.rules();

    // Get all the Discrete Value Rules in the Rulebook.
    let discrete_value_rules = all_rules.discrete_value_rules;
    for (_, rule) in discrete_value_rules {
        println!("Discrete Value Rule ID: {}", rule.id());
        println!("Type ID: {}", rule.type_id());
        println!("Zone:");
        let zone = rule.zone();
        println!("\t Length: {}", zone.length());
        println!("\t Ranges:");
        for range in zone.ranges() {
            println!("\t\t- Lane id: {}", range.lane_id());
            println!("\t\t  s0: {}", range.s_range().s0());
            println!("\t\t  s1: {}", range.s_range().s1());
            println!("\t\t  length: {}", range.length());
        }
        let states = rule.states();
        println!("States:");
        for state in states {
            println!("\t- state: {}", state.value());
            println!("\t  severity: {}", state.severity());
            let related_rules = state.related_rules();
            println!("\t  related_rules:");
            for (group, rules) in related_rules {
                println!("\t\t  Related Rules Group: {}", group);
                for related_rule in rules {
                    println!("\t\t\t  Related Rule: {}", related_rule);
                }
            }
            println!("\t  related_unique_ids:");
            let related_unique_ids = state.related_unique_ids();
            for (group, unique_ids) in related_unique_ids {
                println!("\t\t  Related Unique IDs Group: {}", group);
                for unique_id in unique_ids {
                    println!("\t\t\t  Unique ID: {}", unique_id);
                }
            }
        }
        println!();
    }
    println!();
    println!("*********************************************************************");
    println!();
    // Find all the discrete value rules in a zone. The zone is defined by a vector of LaneSRange, which is basically a vector of lane IDs and a SRange.
    use maliput::api::rules::RuleState;
    let road_geometry = road_network.road_geometry();
    let lane_1_0_1 = road_geometry.get_lane(&String::from("1_0_1")).unwrap();
    // Find rules in the entire range of lane 1_0_1.
    let ranges_to_find_rules = vec![maliput::api::LaneSRange::new(
        &String::from("1_0_1"),
        &maliput::api::SRange::new(0.0, lane_1_0_1.length()),
    )];
    let rules_at_1_0_1 = rulebook.find_rules(&ranges_to_find_rules, 1e-3)?;
    let discrete_value_rules_at_1_0_1 = rules_at_1_0_1.discrete_value_rules;
    println!("All Discrete Value Rules at 1_0_1:");
    for (rule_id, _) in discrete_value_rules_at_1_0_1.iter() {
        println!("Discrete Value Rule at 1_0_1 ID: {}", rule_id);
    }

    let rules_of_type_direction_usage_at_1_0_1 = discrete_value_rules_at_1_0_1
        .iter()
        .filter_map(|(_, rule)| {
            if rule.type_id() == "Direction-Usage Rule Type" {
                Some(rule)
            } else {
                None
            }
        })
        .collect::<Vec<_>>();
    assert_eq!(rules_of_type_direction_usage_at_1_0_1.len(), 1);
    let direction_usage_rule = rules_of_type_direction_usage_at_1_0_1[0];
    assert_eq!(direction_usage_rule.type_id(), "Direction-Usage Rule Type");
    assert_eq!(direction_usage_rule.id(), "Direction-Usage Rule Type/1_0_1");

    let states = direction_usage_rule.states();
    assert_eq!(states.len(), 1); // Only one state for Direction-Usage Rule Type
    let state = states.first().unwrap();
    assert!((state.value() == "WithS") || (state.value() == "AgainstS"));
    println!("Direction-Usage Rule Type State: {}", state.value());

    println!();
    println!("*********************************************************************");
    println!();
    // Alternatively, if you know the type of the rule and the lane id, you can use the `get_discrete_value_rule` method to get the rule directly.
    let expected_rule_id = String::from("Direction-Usage Rule Type/1_0_1");
    let rule = rulebook.get_discrete_value_rule(&expected_rule_id);
    assert!(rule.is_some());
    let rule = rule.unwrap();
    assert_eq!(rule.id(), expected_rule_id);
    assert_eq!(rule.type_id(), "Direction-Usage Rule Type");
    let states = rule.states();
    assert_eq!(states.len(), 1); // Only one state for Direction-Usage Rule Type
    let state = states.first().unwrap();
    assert!((state.value() == "WithS") || (state.value() == "AgainstS"));
    println!("Direction-Usage Rule Type State: {}", state.value());

    Ok(())
}
