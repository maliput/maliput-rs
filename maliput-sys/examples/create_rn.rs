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

use maliput_sys::plugin::ffi::CreateRoadNetwork;

fn main() {
    println!("\nExecuting create_rn.rs example: ");

    let road_network_loader_id = "maliput_malidrive".to_string();
    let properties = vec![
        "road_geometry_id:my_rg_from_rust".to_string(),
        "opendrive_file:/opt/ros/foxy/share/maliput_malidrive/resources/odr/TShapeRoad.xodr".to_string(),
    ];

    let road_network = CreateRoadNetwork(&road_network_loader_id, &properties);
    let road_geometry = road_network.road_geometry();

    // Excercise the RoadGeometry bindings.
    unsafe {
        // This is unsafe because we are dereferencing a raw pointer.
        println!("linear_tolerance: {}", (*road_geometry).linear_tolerance());
        println!("angular_tolerance: {}", (*road_geometry).angular_tolerance());
        println!("num_junctions: {}", (*road_geometry).num_junctions());
        println!("num_branch_points: {}", (*road_geometry).num_branch_points());
    }
}
