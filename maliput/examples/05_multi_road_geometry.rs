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

use maliput::api::RoadGeometry;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    use maliput::api::{RoadNetwork, RoadNetworkBackend};
    use std::collections::HashMap;

    // Get location of odr resources
    let package_location = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let xodr_path_t_shape = format!("{}/data/xodr/TShapeRoad.xodr", package_location);
    let xodr_path_town_01 = format!("{}/data/xodr/Town01.xodr", package_location);

    let road_network_properties_rg_1 = HashMap::from([
        ("road_geometry_id", "rg_1_t_shape_road"),
        ("opendrive_file", xodr_path_t_shape.as_str()),
    ]);
    let road_network_properties_2 = HashMap::from([
        ("road_geometry_id", "rg_2_town_01"),
        ("opendrive_file", xodr_path_town_01.as_str()),
    ]);

    let road_network_1 = RoadNetwork::new(RoadNetworkBackend::MaliputMalidrive, &road_network_properties_rg_1)?;
    let road_network_2 = RoadNetwork::new(RoadNetworkBackend::MaliputMalidrive, &road_network_properties_2)?;
    let road_geometry_1 = road_network_1.road_geometry();
    let road_geometry_2 = road_network_2.road_geometry();

    let print_rg = |rg: &RoadGeometry| {
        let lanes = rg.get_lanes();
        println!("RoadGeometry ID: {}", rg.id());
        println!("\tlinear_tolerance: {}", rg.linear_tolerance());
        println!("\tangular_tolerance: {}", rg.angular_tolerance());
        println!("\tnum_junctions: {}", rg.num_junctions());
        println!("\tnum_lanes: {}", lanes.len());
        println!("\tlanes: ");
        for lane in lanes {
            println!("\t\tlane id: {}", lane.id());
        }
    };
    print_rg(&road_geometry_1);
    print_rg(&road_geometry_2);
    Ok(())
}
