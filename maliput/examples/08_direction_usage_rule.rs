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

use maliput::api::rules::RuleType;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    use maliput::api::RoadNetwork;
    use std::collections::HashMap;

    // Get location of odr resources
    let package_location = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let xodr_path = format!("{}/data/xodr/TShapeRoad.xodr", package_location);
    let road_network_properties = HashMap::from([
        ("road_geometry_id", "my_rg_from_rust"),
        ("opendrive_file", xodr_path.as_str()),
        ("linear_tolerance", "0.01"),
        ("use_userdata_traffic_direction", "false"),
    ]);

    let road_network = RoadNetwork::new("maliput_malidrive", &road_network_properties)?;
    let rg = road_network.road_geometry();
    let rulebook = road_network.rulebook();

    // Let's find the direction usage rule for all the lanes
    let lanes = rg.get_lanes();
    for lane in lanes {
        let rule_type = RuleType::DirectionUsage;
        let lane_id = lane.id();
        let rule = rulebook.get_discrete_value_rule(&rule_type.get_rule_id(&lane_id));
        if let Ok(rule) = rule {
            // Print the DiscreteValueRule for the lane.
            println!("Direction Usage Rule for lane {}:\n{:?}", lane_id, rule);
            // Print just the state for better showcase.
            let states = rule.states();
            assert!(
                !states.is_empty(),
                "No states found for Direction Usage Rule for lane {}",
                lane_id
            );
            assert!(
                states.len() == 1,
                "Expected exactly one state for Direction Usage Rule for lane {}, found {}",
                lane_id,
                states.len()
            );
            let state = &states[0];
            println!("\tState: {}\n", state.value());
        } else {
            println!("No Direction Usage Rule found for lane {}", lane_id);
            panic!("Expected a Direction Usage Rule for lane {}", lane_id);
        }
    }
    Ok(())
}
