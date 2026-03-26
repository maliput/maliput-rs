// BSD 3-Clause License
//
// Copyright (c) 2026, Woven by Toyota.
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

// This example demonstrates how to load a road network that uses a traffic signal
// type database (traffic_signal_db) to automatically generate TrafficLight objects
// from OpenDRIVE signal elements, and how to query the TrafficLightBook by lane ID.
//
// The TwoRoadsWithTrafficLights.xodr map has two roads, each with a traffic light
// signal defined inline. The TrafficSignalDatabase.yaml describes the physical
// structure (bulbs, colors, states) of each signal type.
//
// Road layout (viewed from above):
//
//      Road 1 (100m)               Road 2 (80m)
//   =======================     =======================
//        lane 1_0_1                   lane 2_0_1
//   - - - - - - - - - - -      - - - - - - - - - - -
//   (0,0) ──── (100,0)          (110,0) ──── (190,0)
//   - - - - - - - - - - -      - - - - - - - - - - -
//        lane 1_0_-1                  lane 2_0_-1
//   =======================     =======================
//         S1 at s=50                   S2 at s=40
//
// S1 has validity fromLane=-1 toLane=-1 (only lane 1_0_-1).
// S2 has no validity element (both lanes of road 2).
// Road 1 also has a signalReference to S2, and road 2 has one to S1,
// so find_by_lane("1_0_-1") returns both S1 and S2.

fn main() -> Result<(), Box<dyn std::error::Error>> {
    use maliput::api::{RoadNetwork, RoadNetworkBackend};
    use std::collections::HashMap;

    let package_location = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let xodr_path = format!("{}/tests/data/xodr/TwoRoadsWithTrafficLights.xodr", package_location);
    let db_path = format!(
        "{}/tests/data/traffic_signal_db/TrafficSignalDatabase.yaml",
        package_location
    );

    let road_network_properties = HashMap::from([
        ("road_geometry_id", "two_roads_with_traffic_lights"),
        ("opendrive_file", xodr_path.as_str()),
        // The traffic_signal_db key points to a YAML file that defines signal type
        // templates (bulbs, colors, states, rule mappings). The backend uses this
        // together with signal descriptions to build TrafficLight objects.
        ("traffic_signal_db", db_path.as_str()),
        ("linear_tolerance", "0.01"),
    ]);

    let road_network = RoadNetwork::new(RoadNetworkBackend::MaliputMalidrive, &road_network_properties)?;

    // --- TrafficLightBook overview ---
    let book = road_network.traffic_light_book();
    let all_lights = book.traffic_lights();
    println!("Total traffic lights: {}", all_lights.len());
    for tl in &all_lights {
        println!("  TrafficLight '{}':", tl.id());
        let pos = tl.position_road_network();
        println!("    position (x={:.2}, y={:.2}, z={:.2})", pos.x(), pos.y(), pos.z());
        println!("    related_lanes: {:?}", tl.related_lanes());
        for bg in tl.bulb_groups() {
            println!("    BulbGroup '{}':", bg.id());
            for bulb in bg.bulbs() {
                println!(
                    "      Bulb '{}': color={:?} type={:?}",
                    bulb.id(),
                    bulb.color(),
                    bulb.bulb_type()
                );
            }
        }
    }

    // --- Query by lane ---
    // find_by_lane returns all TrafficLights whose related_lanes() includes the given lane ID.
    let lane_id = String::from("1_0_-1");
    println!("\nTrafficLights relevant to lane '{lane_id}':");
    let found = book.find_by_lane(&lane_id);
    if found.is_empty() {
        println!("  (none)");
    } else {
        for tl in &found {
            println!("  '{}'", tl.id());
        }
    }

    Ok(())
}
