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

use std::{any::Any, collections::HashMap};

/// Returns a very simple HashMap with the road network properties based on the provided xodr_path.
fn get_road_network_properties(xodr_path: &str) -> HashMap<&str, &str> {
    HashMap::from([("road_geometry_id", "my_rg_from_rust"), ("opendrive_file", xodr_path)])
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    use maliput::api::{RoadNetwork, RoadNetworkBackend};

    // Use a wrong xodr_path
    let invalid_xodr_path = "/hopefully/this/path/does/not/exist.xodr";
    let road_network_properties = get_road_network_properties(invalid_xodr_path);
    let road_network = RoadNetwork::new(RoadNetworkBackend::MaliputMalidrive, &road_network_properties);
    assert!(
        road_network.is_err(),
        "Expected an error when creating RoadNetwork with an invalid xodr_path"
    );
    match road_network {
        Ok(_) => panic!("Expected RoadNetwork creation to fail with an invalid xodr_path"),
        Err(e) => {
            assert!(
                e.type_id() == std::any::TypeId::of::<maliput::common::MaliputError>(),
                "Expected MaliputError, got: {:?}",
                e.type_id()
            );
            if let maliput::common::MaliputError::AssertionError(_) = e {
                // This is the expected error type.
            } else {
                panic!("Expected MaliputError::AssertionError, got: {:?}", e);
            }
        }
    }

    // Use a valid xodr_path
    // Get location of odr resources
    let package_location = std::env::var("CARGO_MANIFEST_DIR")?;
    let xodr_path = format!("{}/data/xodr/TShapeRoad.xodr", package_location);
    let road_network_properties = get_road_network_properties(xodr_path.as_str());
    let road_network_result = RoadNetwork::new(RoadNetworkBackend::MaliputMalidrive, &road_network_properties);
    assert!(
        road_network_result.is_ok(),
        "Expected RoadNetwork to be created successfully with a valid xodr_path"
    );

    // If we reach here, the RoadNetwork was created successfully.
    let road_network = road_network_result.unwrap();
    let road_geometry = road_network.road_geometry();
    // Print number of lanes to show that we can query the road geometry.
    let lanes = road_geometry.get_lanes();
    println!("Number of lanes: {}", lanes.len());
    // TODO(francocipollone): Proceed to do some queries that might fail.
    Ok(())
}
