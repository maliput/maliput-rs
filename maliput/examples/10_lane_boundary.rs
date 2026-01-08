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

fn main() -> Result<(), Box<dyn std::error::Error>> {
    use maliput::api::RoadNetwork;
    use std::collections::HashMap;

    // Get location of odr resources
    let package_location = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let xodr_path = format!("{}/data/xodr/TShapeRoad.xodr", package_location);

    let road_network_properties = HashMap::from([
        ("road_geometry_id", "my_rg_from_rust"),
        ("opendrive_file", xodr_path.as_str()),
        ("omit-nondrivable-lanes", "false"),
    ]);

    let road_network = RoadNetwork::new("maliput_malidrive", &road_network_properties)?;
    let road_geometry = road_network.road_geometry();

    println!("\n\nAccessing boundaries from segments.\n");
    for i in 0..road_geometry.num_junctions() {
        let junction = road_geometry.junction(i).unwrap();
        for j in 0..junction.num_segments() {
            let segment = junction.segment(j).unwrap();
            println!("\n- Segment ID: {}", segment.id());
            for boundary in segment.boundaries()? {
                println!("\n\t- Boundary ID: {}", boundary.id());
                // Get all the markings in the boundary.
                for marking in boundary.get_markings() {
                    println!("\t\t- Marking");
                    println!("\t\t    s-range: [{:.2}; {:.2}]", marking.s_start, marking.s_end);
                    println!("\t\t    type: {}", marking.lane_marking.get_type().to_string());
                    println!("\t\t    color: {}", marking.lane_marking.color().to_string());
                    println!("\t\t    material: {}", marking.lane_marking.material());
                    println!("\t\t    width: {:.2}", marking.lane_marking.width());
                    println!("\t\t    height: {:.2}", marking.lane_marking.height());
                    println!("\t\t    weight: {}", marking.lane_marking.weight().to_string());
                    println!(
                        "\t\t    lane change permission: {}",
                        marking.lane_marking.lane_change().to_string()
                    );
                }
                // Get markings by s.
                let marking = boundary.get_marking(10.);
                println!("\t\tQuerying marking at s = 10 ...");
                if marking.is_some() {
                    let marking = marking.unwrap();
                    println!("\t\t- Marking s-range: [{:.2}; {:.2}]", marking.s_start, marking.s_end);
                } else {
                    println!("\t\tNo marking at s = 10");
                }
                // Get markings in s-range.
                let markings = boundary.get_markings_by_range(7., 13.);
                println!("\t\tQuerying markings in s-range [7; 13] ...");
                if markings.is_empty() {
                    println!("\t\tNo markings in s-range [7; 13]");
                }
                for marking in markings {
                    println!("\t\t- Marking s-range: [{:.2}; {:.2}]", marking.s_start, marking.s_end);
                }
                let lane_to_left = boundary.lane_to_left();
                if lane_to_left.is_some() {
                    println!("\t\t- Lane to the left: {}", lane_to_left.unwrap().id());
                }
                let lane_to_right = boundary.lane_to_right();
                if lane_to_right.is_some() {
                    println!("\t\t- Lane to the right: {}", lane_to_right.unwrap().id());
                }
            }
        }
    }

    println!("\n\nAccessing boundaries from lanes.\n");
    // Verify boundaries can be accessed from the lanes.
    for lane in road_geometry.get_lanes() {
        println!("\n- Lane ID: {}", lane.id());
        let boundary = lane.left_boundary()?;
        println!("\t- Left boundary ID: {}", boundary.id());
        let boundary = lane.right_boundary()?;
        println!("\t- Right boundary ID: {}", boundary.id());
    }

    Ok(())
}
