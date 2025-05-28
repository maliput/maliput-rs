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

//! A command line tool to query a road network built from an OpenDRIVE file.
//!
//! e.g: cargo run --bin maliput_query -- maliput/data/xodr/ArcLane.xodr

use clap::Parser;

use maliput::api::RoadNetwork;
use std::collections::HashMap;

// Define the MaliputQuery enum to represent different types of queries that can be made to the road network.
enum MaliputQuery {
    GetNumberOfLanes,
    GetTotalLengthOfTheRoadGeometry,
    GetLaneLength(String),
    ToRoadPosition(f64, f64, f64),             // x, y, z
    ToLanePosition(String, f64, f64, f64),     // lane_id, x, y, z
    ToSegmentPosition(String, f64, f64, f64),  // lane_id, x, y, z
    ToInertialPosition(String, f64, f64, f64), // lane_id, s, r, h
    GetOrientaiton(String, f64, f64, f64),     // lane_id, s, r, h
    Invalid,
}

impl From<Vec<&str>> for MaliputQuery {
    fn from(args: Vec<&str>) -> Self {
        match args.as_slice() {
            ["GetNumberOfLanes"] => MaliputQuery::GetNumberOfLanes,
            ["GetTotalLengthOfTheRoadGeometry"] => MaliputQuery::GetTotalLengthOfTheRoadGeometry,
            ["GetLaneLength", lane_id] => MaliputQuery::GetLaneLength(lane_id.to_string()),
            ["ToRoadPosition", x, y, z] => {
                MaliputQuery::ToRoadPosition(x.parse().unwrap(), y.parse().unwrap(), z.parse().unwrap())
            }
            ["ToLanePosition", lane_id, x, y, z] => MaliputQuery::ToLanePosition(
                lane_id.to_string(),
                x.parse().unwrap(),
                y.parse().unwrap(),
                z.parse().unwrap(),
            ),
            ["ToSegmentPosition", lane_id, x, y, z] => MaliputQuery::ToSegmentPosition(
                lane_id.to_string(),
                x.parse().unwrap(),
                y.parse().unwrap(),
                z.parse().unwrap(),
            ),
            ["ToInertialPosition", lane_id, s, r, h] => MaliputQuery::ToInertialPosition(
                lane_id.to_string(),
                s.parse().unwrap(),
                r.parse().unwrap(),
                h.parse().unwrap(),
            ),
            ["GetOrientation", lane_id, s, r, h] => MaliputQuery::GetOrientaiton(
                lane_id.to_string(),
                s.parse().unwrap(),
                r.parse().unwrap(),
                h.parse().unwrap(),
            ),
            _ => MaliputQuery::Invalid,
        }
    }
}

impl MaliputQuery {
    fn print_available_queries() {
        println!("Available commands:");
        println!("1. GetNumberOfLanes");
        println!("2. GetTotalLengthOfTheRoadGeometry");
        println!("3. GetLaneLength <lane_id>");
        println!("4. ToRoadPosition <x> <y> <z>");
        println!("5. ToLanePosition <lane_id> <x> <y> <z>");
        println!("6. ToSegmentPosition <lane_id> <x> <y> <z>");
        println!("7. ToInertialPosition <lane_id> <s> <r> <h>");
        println!("8. GetOrientation <lane_id> <s> <r> <h>");
    }
}

struct RoadNetworkQuery<'a> {
    rn: &'a RoadNetwork,
}

impl<'a> RoadNetworkQuery<'a> {
    fn new(rn: &'a RoadNetwork) -> Self {
        RoadNetworkQuery { rn }
    }

    fn execute_query(&self, query: MaliputQuery) {
        let rg = self.rn.road_geometry();
        let start_time = std::time::Instant::now();
        match query {
            MaliputQuery::GetNumberOfLanes => {
                let len = rg.get_lanes().len();
                print_elapsed_time(start_time);
                println!("Number of lanes: {}", len);
            }
            MaliputQuery::GetTotalLengthOfTheRoadGeometry => {
                let lanes_num = rg.get_lanes().len();
                let total_length = rg.get_lanes().iter().map(|lane| lane.length()).sum::<f64>();
                println!("Total length of the road geometry: {} meters along {} lanes. The average lane length is {} meters.",
                total_length,
                lanes_num,
                total_length / lanes_num as f64);
                print_elapsed_time(start_time);
            }
            MaliputQuery::GetLaneLength(lane_id) => {
                if let Some(lane) = rg.get_lane(&lane_id) {
                    let lane_length = lane.length();
                    print_elapsed_time(start_time);
                    println!("Length of lane {}: {} meters", lane_id, lane_length);
                } else {
                    println!("Lane with ID {} not found.", lane_id);
                }
            }
            MaliputQuery::ToRoadPosition(x, y, z) => {
                let road_position_result = rg.to_road_position(&maliput::api::InertialPosition::new(x, y, z));
                print_elapsed_time(start_time);
                println!("Road Position Result:");
                println!("\t* Road Position:");
                println!("\t\t* lane_id: {}", road_position_result.road_position.lane().id());
                println!("\t\t* lane_position: {}", road_position_result.road_position.pos());
                println!("\t* Nearest Position:");
                println!("\t\t* inertial_position: {}", road_position_result.nearest_position);
                println!("\t* Distance:");
                println!("\t\t* distance: {}", road_position_result.distance);
            }
            MaliputQuery::ToLanePosition(lane_id, x, y, z) => {
                if let Some(lane) = rg.get_lane(&lane_id) {
                    let lane_position_result = lane.to_lane_position(&maliput::api::InertialPosition::new(x, y, z));
                    print_elapsed_time(start_time);
                    println!("Lane Position Result for lane {}:", lane_id);
                    println!("\t* Lane Position:");
                    println!("\t\t* lane_position: {}", lane_position_result.lane_position);
                    println!("\t* Nearest Position:");
                    println!("\t\t* inertial_position: {}", lane_position_result.nearest_position);
                    println!("\t* Distance:");
                    println!("\t\t* distance: {}", lane_position_result.distance);
                } else {
                    println!("Lane with ID {} not found.", lane_id);
                }
            }
            MaliputQuery::ToSegmentPosition(lane_id, x, y, z) => {
                if let Some(lane) = rg.get_lane(&lane_id) {
                    let lane_position_result = lane.to_segment_position(&maliput::api::InertialPosition::new(x, y, z));
                    print_elapsed_time(start_time);
                    println!("Segment Position Result for lane {}:", lane_id);
                    println!("\t* Lane Position:");
                    println!("\t\t* lane_position: {}", lane_position_result.lane_position);
                    println!("\t* Nearest Position:");
                    println!("\t\t* inertial_position: {}", lane_position_result.nearest_position);
                    println!("\t* Distance:");
                    println!("\t\t* distance: {}", lane_position_result.distance);
                } else {
                    println!("Lane with ID {} not found.", lane_id);
                }
            }
            MaliputQuery::ToInertialPosition(lane_id, s, r, h) => {
                if let Some(lane) = rg.get_lane(&lane_id) {
                    let inertial_position = lane.to_inertial_position(&maliput::api::LanePosition::new(s, r, h));
                    print_elapsed_time(start_time);
                    println!("Inertial Position Result for lane {}:", lane_id);
                    println!("\t* inertial_position: {}", inertial_position);
                } else {
                    println!("Lane with ID {} not found.", lane_id);
                }
            }
            MaliputQuery::GetOrientaiton(lane_id, s, r, h) => {
                if let Some(lane) = rg.get_lane(&lane_id) {
                    let orientation = lane.get_orientation(&maliput::api::LanePosition::new(s, r, h));
                    print_elapsed_time(start_time);
                    println!("Orientation Result for lane {}:", lane_id);
                    println!("\t* orientation:");
                    println!(
                        "\t\t* roll: {}, pitch: {}, yaw: {}",
                        orientation.roll(),
                        orientation.pitch(),
                        orientation.yaw()
                    );
                } else {
                    println!("Lane with ID {} not found.", lane_id);
                }
            }
            MaliputQuery::Invalid => {
                println!("Invalid query command. Please try again.");
                println!();
                MaliputQuery::print_available_queries();
            }
        }
        /// Print the elapsed time for the query execution.
        fn print_elapsed_time(start_time: std::time::Instant) {
            let elapsed = start_time.elapsed();
            println!("Query executed in {:.2?}.", elapsed);
        }
    }
}

#[derive(Parser, Debug)]
#[command(author, version, about)]
struct Args {
    road_network_xodr_file_path: String,

    /// Tolerance indicates the precision of the geometry being built in order
    /// to make G1 continuity guarantees. A value of 0.5 is high. Ideally,
    /// tolerances of 0.05 or 0.005 is ideal.
    #[arg(short, long, default_value_t = 0.05)]
    linear_tolerance: f64,

    #[arg(short, long)]
    max_linear_tolerance: Option<f64>,

    #[arg(short, long, default_value_t = 0.01)]
    angular_tolerance: f64,

    #[arg(long, default_value_t = false)]
    allow_non_drivable_lanes: bool,

    #[arg(long, default_value_t = false)]
    disable_parallel_builder_policy: bool,

    #[arg(long, default_value_t = maliput::common::LogLevel::Info)]
    set_log_level: maliput::common::LogLevel,
}

// Parses the OpenDRIVE file path from the command line arguments.
//  - if this is a relative path, it will be resolved against the current working directory.
//  - if this is an absolute path, it will be used as is.
use std::path::PathBuf;
fn parse_xodr_file_path(path: &str) -> PathBuf {
    let mut path_buf = PathBuf::from(path);
    if !path_buf.is_absolute() {
        // If the path is relative, resolve it against the current working directory.
        if let Ok(current_dir) = std::env::current_dir() {
            path_buf = current_dir.join(path_buf);
        }
    }
    path_buf
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    // Configuration for the road network.
    let xodr_path = parse_xodr_file_path(args.road_network_xodr_file_path.as_str());
    let linear_tolerance: String = args.linear_tolerance.to_string();
    let max_linear_tolerance = args.max_linear_tolerance;
    let angular_tolerance = args.angular_tolerance.to_string();
    let omit_non_drivable_lanes = if args.allow_non_drivable_lanes { "false" } else { "true" };
    let parallel_builder_policy = !args.disable_parallel_builder_policy;

    let mut road_network_properties = HashMap::from([
        ("road_geometry_id", "maliput_query_rg"),
        ("opendrive_file", xodr_path.to_str().unwrap()),
        ("linear_tolerance", linear_tolerance.as_str()),
        ("omit_nondrivable_lanes", omit_non_drivable_lanes),
        ("angular_tolerance", angular_tolerance.as_str()),
        (
            "build_policy",
            if parallel_builder_policy {
                "parallel"
            } else {
                "sequential"
            },
        ),
    ]);
    let max_linear_tolerance_str;
    if let Some(max_tolerance) = max_linear_tolerance {
        max_linear_tolerance_str = max_tolerance.to_string();
        road_network_properties.insert("max_linear_tolerance", max_linear_tolerance_str.as_str());
    }

    // Print the parsed arguments
    println!(
        "maliput_query application started with the following arguments: {:?}",
        args
    );
    println!();
    // Print the road network properties for debugging purposes.
    println!("Road network properties: {:?}", road_network_properties);

    maliput::common::set_log_level(args.set_log_level);
    let now = std::time::Instant::now();
    let road_network = RoadNetwork::new("maliput_malidrive", &road_network_properties);
    let elapsed = now.elapsed();
    println!("Road network loaded in {:.2?}.", elapsed);

    // Print available commands
    println!();
    MaliputQuery::print_available_queries();
    println!("To exit the application, type 'exit'.");

    loop {
        println!();
        let mut input = String::new();
        println!("Enter a query command (or 'exit' to quit):");
        std::io::stdin().read_line(&mut input)?;
        let input = input.trim();

        if input == "exit" {
            break;
        }

        let query: MaliputQuery = input.split_whitespace().collect::<Vec<&str>>().into();
        let query_handler = RoadNetworkQuery::new(&road_network);
        query_handler.execute_query(query);
    }

    Ok(())
}
