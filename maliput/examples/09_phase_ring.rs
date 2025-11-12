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

use maliput::api::rules::BulbState;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    use maliput::api::RoadNetwork;
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

    let road_network = RoadNetwork::new("maliput_malidrive", &road_network_properties)?;

    let phase_ring_book = road_network.phase_ring_book();
    let phase_ring_ids = phase_ring_book.get_phase_rings_ids();
    for phase_ring_id in &phase_ring_ids {
        println!("Phase Ring ID: {phase_ring_id}");
        println!("Phases:");

        if let Some(phase_ring) = phase_ring_book.get_phase_ring(phase_ring_id) {
            let phases = phase_ring.phases();

            for phase_id in &phases {
                println!("\t  Phase ID: {phase_id}");

                if let Some(phase) = phase_ring.get_phase(phase_id) {
                    let states = phase.discrete_value_rule_states();
                    println!("\t  States:");

                    for state in &states {
                        println!("\t\t- Discrete Value Rule ID: {}", state.0);
                    }

                    println!("\t  Unique Bulb IDs:");
                    for bulb_id in phase.unique_bulb_ids() {
                        println!("\t\t- Bulb ID: {}", bulb_id.string());
                        if let Some(bulb_state) = phase.bulb_state(&bulb_id) {
                            println!(
                                "\t\t  Bulb state: {}",
                                match bulb_state {
                                    BulbState::Blinking => "Blinking",
                                    BulbState::On => "On",
                                    BulbState::Off => "Off",
                                }
                            );
                        }
                    }
                }

                let next_phases = phase_ring.get_next_phases(phase_id)?;
                println!("\t  Next Phases:");

                for next_phase in &next_phases {
                    println!("\t\t- next phase: {}", next_phase.next_phase.id());
                    if let Some(duration_until) = next_phase.duration_until {
                        println!("\t\t  duration until: {}", duration_until);
                    }
                }
                println!();
            }
        }
    }
    Ok(())
}
