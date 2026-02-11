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

fn main() -> Result<(), Box<dyn std::error::Error>> {
    use maliput::api::{RoadNetwork, RoadNetworkBackend};
    use std::collections::HashMap;

    // Get location of odr resources
    let package_location = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let xodr_path = format!("{}/data/xodr/TShapeRoad.xodr", package_location);

    let road_network_properties = HashMap::from([
        ("road_geometry_id", "my_rg_from_rust"),
        ("opendrive_file", xodr_path.as_str()),
    ]);

    let road_network = RoadNetwork::new(RoadNetworkBackend::MaliputMalidrive, &road_network_properties)?;
    let road_geometry = road_network.road_geometry();

    // Exercise the RoadGeometry API.
    println!("linear_tolerance: {}", road_geometry.linear_tolerance());
    println!("angular_tolerance: {}", road_geometry.angular_tolerance());
    println!("num_junctions: {}", road_geometry.num_junctions());

    // Verify junctions / segments / lanes can be accessed.
    for i in 0..road_geometry.num_junctions() {
        let junction = road_geometry.junction(i).unwrap();
        println!(" junction id: {}", junction.id());
        for j in 0..junction.num_segments() {
            let segment = junction.segment(j).unwrap();
            println!("\tsegment id: {}", segment.id());
            for k in 0..segment.num_lanes() {
                let lane = segment.lane(k).unwrap();
                println!("\t\tlane id: {}", lane.id());
            }
        }
    }

    // All the lanes can be obtained from RoadGeometry directly.
    let lanes = road_geometry.get_lanes();
    println!("num_lanes: {}", lanes.len());
    println!("lanes: ");
    for lane in lanes {
        println!("\tlane id: {}", lane.id());
    }

    let inertial_position = maliput::api::InertialPosition::new(10.0, 0.0, 0.0);
    let road_position_query = road_geometry.to_road_position(&inertial_position)?;
    let round_inertial_pos = road_position_query.road_position.to_inertial_position()?;
    assert!((inertial_position.x() - round_inertial_pos.x()).abs() < road_geometry.linear_tolerance());

    Ok(())
}
