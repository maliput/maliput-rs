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

//! A command line tool to query a road network built from a road network file.
//!
//! e.g: cargo run --bin maliput_query -- maliput/data/xodr/ArcLane.xodr
//! e.g: cargo run --bin maliput_query -- --backend maliput_geopackage path/to/file.gpkg

use clap::Parser;

use maliput::api::{LanePosition, RoadNetwork, RoadNetworkBackend};
use maliput::common::MaliputError;
use std::collections::HashMap;

// Define the MaliputQuery enum to represent different types of queries that can be made to the road network.
enum MaliputQuery {
    PrintAllLanes,
    GetNumberOfLanes,
    GetTotalLengthOfTheRoadGeometry,
    GetLinearTolerance,
    GetAngularTolerance,
    GetLaneLength(String),
    GetLaneBounds(String, f64),                                           // lane_id, s
    GetSegmentBounds(String, f64),                                        // lane_id, s
    ToRoadPosition(f64, f64, f64),                                        // x, y, z
    ToLanePosition(String, f64, f64, f64),                                // lane_id, x, y, z
    ToSegmentPosition(String, f64, f64, f64),                             // lane_id, x, y, z
    ToInertialPosition(String, f64, f64, f64),                            // lane_id, s, r, h
    GetOrientaiton(String, f64, f64, f64),                                // lane_id, s, r, h
    OpenScenarioRoadPositionToMaliputRoadPosition(String, f64, f64),      // xodr_road_id, xodr_s, xodr_t
    OpenScenarioLanePositionToMaliputRoadPosition(String, f64, i64, f64), // xodr_road_id, xodr_s, xodr_lane_id, offset
    MaliputRoadPositionToOpenScenarioRoadPosition(String, f64, f64, f64), // lane_id, s, r, h
    MaliputRoadPositionToOpenScenarioLanePosition(String, f64, f64, f64), // lane_id, s, r, h
    OpenScenarioRelativeRoadPositionToMaliputRoadPosition(String, f64, f64, f64, f64), // xodr_road_id, xodr_s, xodr_t, xodr_ds, xodr_dt
    OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition(String, i64, f64, f64, f64, f64), // xodr_road_id, xodr_lane_id, xodr_s, d_lane, xodr_ds, offset
    OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition(String, i64, f64, f64, f64, f64), // xodr_road_id, xodr_lane_id, xodr_s, d_lane, xodr_ds_lane, offset
    GetRoadOrientationAtOpenScenarioRoadPosition(String, f64, f64), // xodr_road_id, xodr_s, xodr_t
    Invalid,
}

impl MaliputQuery {
    /// Converts the MaliputQuery to a string format that can be used in backend custom commands architecture.
    /// ### Returns
    /// - Some(String): A string representation of the command if it is a valid query.
    /// - None: If the query is not a valid command for backend custom commands.
    fn to_backend_custom_command_format(&self) -> Option<String> {
        match self {
            MaliputQuery::OpenScenarioRoadPositionToMaliputRoadPosition(xodr_road_id, xodr_s, xodr_t) => Some(format!(
                "OpenScenarioRoadPositionToMaliputRoadPosition,{},{},{}",
                xodr_road_id, xodr_s, xodr_t
            )),
            MaliputQuery::OpenScenarioLanePositionToMaliputRoadPosition(xodr_road_id, xodr_s, xodr_lane_id, offset) => {
                Some(format!(
                    "OpenScenarioLanePositionToMaliputRoadPosition,{},{},{},{}",
                    xodr_road_id, xodr_s, xodr_lane_id, offset
                ))
            }
            MaliputQuery::MaliputRoadPositionToOpenScenarioRoadPosition(lane_id, s, r, h) => Some(format!(
                "MaliputRoadPositionToOpenScenarioRoadPosition,{},{},{},{}",
                lane_id, s, r, h
            )),
            MaliputQuery::MaliputRoadPositionToOpenScenarioLanePosition(lane_id, s, r, h) => Some(format!(
                "MaliputRoadPositionToOpenScenarioLanePosition,{},{},{},{}",
                lane_id, s, r, h
            )),
            MaliputQuery::OpenScenarioRelativeRoadPositionToMaliputRoadPosition(
                xodr_road_id,
                xodr_s,
                xodr_t,
                xodr_ds,
                xodr_dt,
            ) => Some(format!(
                "OpenScenarioRelativeRoadPositionToMaliputRoadPosition,{},{},{},{},{}",
                xodr_road_id, xodr_s, xodr_t, xodr_ds, xodr_dt
            )),
            MaliputQuery::OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition(
                xodr_road_id,
                xodr_lane_id,
                xodr_s,
                d_lane,
                xodr_ds,
                offset,
            ) => Some(format!(
                "OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition,{},{},{},{},{},{}",
                xodr_road_id, xodr_lane_id, xodr_s, d_lane, xodr_ds, offset
            )),
            MaliputQuery::OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition(
                xodr_road_id,
                xodr_lane_id,
                xodr_s,
                d_lane,
                xodr_ds_lane,
                offset,
            ) => Some(format!(
                "OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition,{},{},{},{},{},{}",
                xodr_road_id, xodr_lane_id, xodr_s, d_lane, xodr_ds_lane, offset
            )),
            MaliputQuery::GetRoadOrientationAtOpenScenarioRoadPosition(xodr_road_id, xodr_s, xodr_t) => Some(format!(
                "GetRoadOrientationAtOpenScenarioRoadPosition,{},{},{}",
                xodr_road_id, xodr_s, xodr_t
            )),
            _ => None,
        }
    }
}

impl From<Vec<&str>> for MaliputQuery {
    fn from(args: Vec<&str>) -> Self {
        match args.as_slice() {
            ["PrintAllLanes"] => MaliputQuery::PrintAllLanes,
            ["GetNumberOfLanes"] => MaliputQuery::GetNumberOfLanes,
            ["GetTotalLengthOfTheRoadGeometry"] => MaliputQuery::GetTotalLengthOfTheRoadGeometry,
            ["GetLinearTolerance"] => MaliputQuery::GetLinearTolerance,
            ["GetAngularTolerance"] => MaliputQuery::GetAngularTolerance,
            ["GetLaneLength", lane_id] => MaliputQuery::GetLaneLength(lane_id.to_string()),
            ["GetLaneBounds", lane_id, s] => MaliputQuery::GetLaneBounds(lane_id.to_string(), s.parse().unwrap()),
            ["GetSegmentBounds", lane_id, s] => MaliputQuery::GetSegmentBounds(lane_id.to_string(), s.parse().unwrap()),
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
            ["OpenScenarioRoadPositionToMaliputRoadPosition", xodr_road_id, xodr_s, xodr_t] => {
                MaliputQuery::OpenScenarioRoadPositionToMaliputRoadPosition(
                    xodr_road_id.to_string(),
                    xodr_s.parse().unwrap(),
                    xodr_t.parse().unwrap(),
                )
            }
            ["OpenScenarioLanePositionToMaliputRoadPosition", xodr_road_id, xodr_s, xodr_lane_id, offset] => {
                MaliputQuery::OpenScenarioLanePositionToMaliputRoadPosition(
                    xodr_road_id.to_string(),
                    xodr_s.parse().unwrap(),
                    xodr_lane_id.parse().unwrap(),
                    offset.parse().unwrap(),
                )
            }
            ["MaliputRoadPositionToOpenScenarioRoadPosition", lane_id, s, r, h] => {
                MaliputQuery::MaliputRoadPositionToOpenScenarioRoadPosition(
                    lane_id.to_string(),
                    s.parse().unwrap(),
                    r.parse().unwrap(),
                    h.parse().unwrap(),
                )
            }
            ["MaliputRoadPositionToOpenScenarioLanePosition", lane_id, s, r, h] => {
                MaliputQuery::MaliputRoadPositionToOpenScenarioLanePosition(
                    lane_id.to_string(),
                    s.parse().unwrap(),
                    r.parse().unwrap(),
                    h.parse().unwrap(),
                )
            }
            ["OpenScenarioRelativeRoadPositionToMaliputRoadPosition", xodr_road_id, xodr_s, xodr_t, xodr_ds, xodr_dt] => {
                MaliputQuery::OpenScenarioRelativeRoadPositionToMaliputRoadPosition(
                    xodr_road_id.to_string(),
                    xodr_s.parse().unwrap(),
                    xodr_t.parse().unwrap(),
                    xodr_ds.parse().unwrap(),
                    xodr_dt.parse().unwrap(),
                )
            }
            ["OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition", xodr_road_id, xodr_lane_id, xodr_s, d_lane, xodr_ds, offset] => {
                MaliputQuery::OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition(
                    xodr_road_id.to_string(),
                    xodr_lane_id.parse().unwrap(),
                    xodr_s.parse().unwrap(),
                    d_lane.parse().unwrap(),
                    xodr_ds.parse().unwrap(),
                    offset.parse().unwrap(),
                )
            }
            ["OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition", xodr_road_id, xodr_lane_id, xodr_s, d_lane, xodr_ds_lane, offset] => {
                MaliputQuery::OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition(
                    xodr_road_id.to_string(),
                    xodr_lane_id.parse().unwrap(),
                    xodr_s.parse().unwrap(),
                    d_lane.parse().unwrap(),
                    xodr_ds_lane.parse().unwrap(),
                    offset.parse().unwrap(),
                )
            }
            ["GetRoadOrientationAtOpenScenarioRoadPosition", xodr_road_id, xodr_s, xodr_t] => {
                MaliputQuery::GetRoadOrientationAtOpenScenarioRoadPosition(
                    xodr_road_id.to_string(),
                    xodr_s.parse().unwrap(),
                    xodr_t.parse().unwrap(),
                )
            }
            _ => MaliputQuery::Invalid,
        }
    }
}

impl MaliputQuery {
    fn print_available_queries() {
        println!("-> Available commands:");
        println!("\t* General commands:");
        println!("\t\t1. PrintAllLanes");
        println!("\t\t2. GetNumberOfLanes");
        println!("\t\t3. GetTotalLengthOfTheRoadGeometry");
        println!("\t\t4. GetLinearTolerance");
        println!("\t\t5. GetAngularTolerance");
        println!("\t\t6. GetLaneLength <lane_id>");
        println!("\t\t7. GetLaneBounds <lane_id> <s>");
        println!("\t\t8. GetSegmentBounds <lane_id> <s>");
        println!("\t\t9. ToRoadPosition <x> <y> <z>");
        println!("\t\t10. ToLanePosition <lane_id> <x> <y> <z>");
        println!("\t\t11. ToSegmentPosition <lane_id> <x> <y> <z>");
        println!("\t\t12. ToInertialPosition <lane_id> <s> <r> <h>");
        println!("\t\t13. GetOrientation <lane_id> <s> <r> <h>");
        println!("\t* Commands particular to maliput_malidrive / OpenDRIVE specification:");
        println!("\t\t1. OpenScenarioRoadPositionToMaliputRoadPosition <xodr_road_id> <xodr_s> <xodr_t>");
        println!(
            "\t\t2. OpenScenarioLanePositionToMaliputRoadPosition <xodr_road_id> <xodr_s> <xodr_lane_id> <offset>"
        );
        println!("\t\t3. MaliputRoadPositionToOpenScenarioRoadPosition <lane_id> <s> <r> <h>");
        println!("\t\t4. MaliputRoadPositionToOpenScenarioLanePosition <lane_id> <s> <r> <h>");
        println!("\t\t5. OpenScenarioRelativeRoadPositionToMaliputRoadPosition <xodr_road_id> <xodr_s> <xodr_t> <xodr_ds> <xodr_dt>");
        println!("\t\t6. OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition <xodr_road_id> <xodr_lane_id> <xodr_s> <d_lane> <xodr_ds> <offset>");
        println!("\t\t7. OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition <xodr_road_id> <xodr_lane_id> <xodr_s> <d_lane> <xodr_ds_lane> <offset>");
        println!("\t\t8. GetRoadOrientationAtOpenScenarioRoadPosition <xodr_road_id> <xodr_s> <xodr_t>");
    }
}

struct RoadNetworkQuery<'a> {
    rn: &'a RoadNetwork,
}

impl<'a> RoadNetworkQuery<'a> {
    fn new(rn: &'a RoadNetwork) -> Self {
        RoadNetworkQuery { rn }
    }

    fn execute_query(&self, query: MaliputQuery) -> Result<(), MaliputError> {
        let rg = self.rn.road_geometry();
        let start_time = std::time::Instant::now();
        match query {
            MaliputQuery::PrintAllLanes => {
                // Sort it by lane length in descending order and then print them
                let mut lanes = rg.get_lanes();
                lanes.sort_by(|a, b| b.length().partial_cmp(&a.length()).unwrap());
                print_elapsed_time(start_time);
                println!("-> All lanes in the road geometry: {} lanes", lanes.len());
                for lane in lanes {
                    println!(
                        "\t* Lane ID: {}\t Length: {} meters \t InertiaPos at (s=0,r=0,h=0): {}",
                        lane.id(),
                        lane.length(),
                        lane.to_inertial_position(&LanePosition::new(0., 0., 0.))?
                    );
                }
            }
            MaliputQuery::GetNumberOfLanes => {
                let len = rg.get_lanes().len();
                print_elapsed_time(start_time);
                println!("-> Number of lanes: {}", len);
            }
            MaliputQuery::GetTotalLengthOfTheRoadGeometry => {
                let lanes_num = rg.get_lanes().len();
                let total_length = rg.get_lanes().iter().map(|lane| lane.length()).sum::<f64>();
                print_elapsed_time(start_time);
                println!("-> Total length of the road geometry: {} meters along {} lanes. The average lane length is {} meters.",
                total_length,
                lanes_num,
                total_length / lanes_num as f64);
            }
            MaliputQuery::GetLinearTolerance => {
                let linear_tolerance = rg.linear_tolerance();
                print_elapsed_time(start_time);
                println!("-> Linear tolerance of the road geometry: {} meters", linear_tolerance);
            }
            MaliputQuery::GetAngularTolerance => {
                let angular_tolerance = rg.angular_tolerance();
                print_elapsed_time(start_time);
                println!(
                    "-> Angular tolerance of the road geometry: {} radians",
                    angular_tolerance
                );
            }
            MaliputQuery::GetLaneLength(lane_id) => {
                if let Some(lane) = rg.get_lane(&lane_id) {
                    let lane_length = lane.length();
                    print_elapsed_time(start_time);
                    println!("-> Length of lane {}: {} meters", lane_id, lane_length);
                } else {
                    println!("-> Lane with ID {} not found.", lane_id);
                }
            }
            MaliputQuery::GetLaneBounds(lane_id, s) => {
                if let Some(lane) = rg.get_lane(&lane_id) {
                    let lane_bounds = lane.lane_bounds(s)?;
                    print_elapsed_time(start_time);
                    println!(
                        "-> Bounds of lane {} at s={}: max: {}, min: {}",
                        lane_id,
                        s,
                        lane_bounds.max(),
                        lane_bounds.min()
                    );
                } else {
                    println!("-> Lane with ID {} not found.", lane_id);
                }
            }
            MaliputQuery::GetSegmentBounds(lane_id, s) => {
                if let Some(lane) = rg.get_lane(&lane_id) {
                    let segment_bounds = lane.segment_bounds(s)?;
                    print_elapsed_time(start_time);
                    println!(
                        "-> Segment bounds of lane {} at s={}: max: {}, min: {}",
                        lane_id,
                        s,
                        segment_bounds.max(),
                        segment_bounds.min()
                    );
                } else {
                    println!("-> Lane with ID {} not found.", lane_id);
                }
            }
            MaliputQuery::ToRoadPosition(x, y, z) => {
                let road_position_result = rg.to_road_position(&maliput::api::InertialPosition::new(x, y, z))?;
                print_elapsed_time(start_time);
                println!("-> Road Position Result:");
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
                    let lane_position_result = lane.to_lane_position(&maliput::api::InertialPosition::new(x, y, z))?;
                    print_elapsed_time(start_time);
                    println!("-> Lane Position Result for lane {}:", lane_id);
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
                    let lane_position_result =
                        lane.to_segment_position(&maliput::api::InertialPosition::new(x, y, z))?;
                    print_elapsed_time(start_time);
                    println!("-> Segment Position Result for lane {}:", lane_id);
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
                    let inertial_position = lane.to_inertial_position(&maliput::api::LanePosition::new(s, r, h))?;
                    print_elapsed_time(start_time);
                    println!("-> Inertial Position Result for lane {}:", lane_id);
                    println!("\t* inertial_position: {}", inertial_position);
                } else {
                    println!("-> Lane with ID {} not found.", lane_id);
                }
            }
            MaliputQuery::GetOrientaiton(lane_id, s, r, h) => {
                if let Some(lane) = rg.get_lane(&lane_id) {
                    let orientation = lane.get_orientation(&maliput::api::LanePosition::new(s, r, h))?;
                    print_elapsed_time(start_time);
                    println!("-> Orientation Result for lane {}:", lane_id);
                    println!("\t* orientation:");
                    println!(
                        "\t\t* roll: {}, pitch: {}, yaw: {}",
                        orientation.roll(),
                        orientation.pitch(),
                        orientation.yaw()
                    );
                } else {
                    println!("-> Lane with ID {} not found.", lane_id);
                }
            }
            MaliputQuery::OpenScenarioRoadPositionToMaliputRoadPosition(..) => {
                let command = query
                    .to_backend_custom_command_format()
                    .expect("Invalid query command format for OpenScenarioRoadPositionToMaliputRoadPosition");
                let res = rg.backend_custom_command(&command)?;
                print_elapsed_time(start_time);
                let res: Vec<&str> = res.split(',').collect();
                println!("-> OpenScenarioRoadPositionToMaliputRoadPosition Result:");
                if res.len() == 4 {
                    println!("\t* Maliput Road Position:");
                    println!("\t\t* lane_id: {}", res[0]);
                    println!("\t\t* s: {}", res[1]);
                    println!("\t\t* r: {}", res[2]);
                    println!("\t\t* h: {}", res[3]);
                } else {
                    println!("-> Invalid response from backend custom command: {}", res.join(", "));
                }
            }
            MaliputQuery::OpenScenarioLanePositionToMaliputRoadPosition(..) => {
                let command = query
                    .to_backend_custom_command_format()
                    .expect("Invalid query command format for OpenScenarioLanePositionToMaliputRoadPosition");
                let res = rg.backend_custom_command(&command)?;
                print_elapsed_time(start_time);
                let res: Vec<&str> = res.split(',').collect();
                println!("-> OpenScenarioLanePositionToMaliputRoadPosition Result:");
                if res.len() == 4 {
                    println!("\t* Maliput Road Position:");
                    println!("\t\t* lane_id: {}", res[0]);
                    println!("\t\t* s: {}", res[1]);
                    println!("\t\t* r: {}", res[2]);
                    println!("\t\t* h: {}", res[3]);
                } else {
                    println!("-> Invalid response from backend custom command: {}", res.join(", "));
                }
            }
            MaliputQuery::MaliputRoadPositionToOpenScenarioRoadPosition(..) => {
                let command = query
                    .to_backend_custom_command_format()
                    .expect("Invalid query command format for MaliputRoadPositionToOpenScenarioRoadPosition");
                let res = rg.backend_custom_command(&command)?;
                print_elapsed_time(start_time);
                let res: Vec<&str> = res.split(',').collect();
                println!("-> MaliputRoadPositionToOpenScenarioRoadPosition Result:");
                if res.len() == 3 {
                    println!("\t* OpenScenario Road Position:");
                    println!("\t\t* xodr_road_id: {}", res[0]);
                    println!("\t\t* xodr_s: {}", res[1]);
                    println!("\t\t* xodr_t: {}", res[2]);
                } else {
                    println!("-> Invalid response from backend custom command: {}", res.join(", "));
                }
            }
            MaliputQuery::MaliputRoadPositionToOpenScenarioLanePosition(..) => {
                let command = query
                    .to_backend_custom_command_format()
                    .expect("Invalid query command format for MaliputRoadPositionToOpenScenarioLanePosition");
                let res = rg.backend_custom_command(&command)?;
                print_elapsed_time(start_time);
                let res: Vec<&str> = res.split(',').collect();
                println!("-> MaliputRoadPositionToOpenScenarioLanePosition Result for lane:");
                if res.len() == 4 {
                    println!("\t* OpenScenario Lane Position:");
                    println!("\t\t* xodr_road_id: {}", res[0]);
                    println!("\t\t* xodr_s: {}", res[1]);
                    println!("\t\t* xodr_lane_id: {}", res[2]);
                    println!("\t\t* offset: {}", res[3]);
                } else {
                    println!("-> Invalid response from backend custom command: {}", res.join(", "));
                }
            }
            MaliputQuery::OpenScenarioRelativeRoadPositionToMaliputRoadPosition(..) => {
                let command = query
                    .to_backend_custom_command_format()
                    .expect("Invalid query command format for OpenScenarioRelativeRoadPositionToMaliputRoadPosition");
                let res = rg.backend_custom_command(&command)?;
                print_elapsed_time(start_time);
                let res: Vec<&str> = res.split(',').collect();
                println!("-> OpenScenarioRelativeRoadPositionToMaliputRoadPosition Result:");
                if res.len() == 4 {
                    println!("\t* Maliput Road Position:");
                    println!("\t\t* lane_id: {}", res[0]);
                    println!("\t\t* s: {}", res[1]);
                    println!("\t\t* r: {}", res[2]);
                    println!("\t\t* h: {}", res[3]);
                } else {
                    println!("-> Invalid response from backend custom command: {}", res.join(", "));
                }
            }
            MaliputQuery::OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition(..) => {
                let command = query.to_backend_custom_command_format().expect(
                    "Invalid query command format for OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition",
                );
                let res = rg.backend_custom_command(&command)?;
                print_elapsed_time(start_time);
                let res: Vec<&str> = res.split(',').collect();
                println!("-> OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition Result:");
                if res.len() == 4 {
                    println!("\t* Maliput Road Position:");
                    println!("\t\t* lane_id: {}", res[0]);
                    println!("\t\t* s: {}", res[1]);
                    println!("\t\t* r: {}", res[2]);
                    println!("\t\t* h: {}", res[3]);
                } else {
                    println!("-> Invalid response from backend custom command: {}", res.join(", "));
                }
            }
            MaliputQuery::OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition(..) => {
                let command = query.to_backend_custom_command_format().expect(
                    "Invalid query command format for OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition",
                );
                let res = rg.backend_custom_command(&command)?;
                print_elapsed_time(start_time);
                let res: Vec<&str> = res.split(',').collect();
                println!("-> OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition Result:");
                if res.len() == 4 {
                    println!("\t* Maliput Road Position:");
                    println!("\t\t* lane_id: {}", res[0]);
                    println!("\t\t* s: {}", res[1]);
                    println!("\t\t* r: {}", res[2]);
                    println!("\t\t* h: {}", res[3]);
                } else {
                    println!("-> Invalid response from backend custom command: {}", res.join(", "));
                }
            }
            MaliputQuery::GetRoadOrientationAtOpenScenarioRoadPosition(..) => {
                let command = query
                    .to_backend_custom_command_format()
                    .expect("Invalid query command format for GetRoadOrientationAtOpenScenarioRoadPosition");
                let res = rg.backend_custom_command(&command)?;
                print_elapsed_time(start_time);
                let res: Vec<&str> = res.split(',').collect();
                println!("-> GetRoadOrientationAtOpenScenarioRoadPosition Result:");
                if res.len() == 3 {
                    println!("\t* Orientation:");
                    println!("\t\t* roll: {}", res[0]);
                    println!("\t\t* pitch: {}", res[1]);
                    println!("\t\t* yaw: {}", res[2]);
                } else {
                    println!("-> Invalid response from backend custom command: {}", res.join(", "));
                }
            }
            MaliputQuery::Invalid => {
                println!("-> Invalid query command. Please try again.");
                println!();
                MaliputQuery::print_available_queries();
            }
        }
        /// Print the elapsed time for the query execution.
        fn print_elapsed_time(start_time: std::time::Instant) {
            let elapsed = start_time.elapsed();
            println!("-> Query executed in {:.2?}.", elapsed);
        }
        Ok(())
    }
}

#[derive(Parser, Debug)]
#[command(author, version, about)]
struct Args {
    /// Path to the road network file (e.g., .xodr for maliput_malidrive, .gpkg for maliput_geopackage).
    road_network_file_path: String,

    /// The maliput backend to use for loading the road network.
    #[cfg(feature = "maliput_malidrive")]
    #[arg(short, long, default_value_t = RoadNetworkBackend::MaliputMalidrive)]
    backend: RoadNetworkBackend,

    /// The maliput backend to use for loading the road network.
    #[cfg(all(not(feature = "maliput_malidrive"), feature = "maliput_geopackage"))]
    #[arg(short, long, default_value_t = RoadNetworkBackend::MaliputGeopackage)]
    backend: RoadNetworkBackend,

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

    #[arg(long, default_value_t = maliput::common::LogLevel::Error)]
    set_log_level: maliput::common::LogLevel,
}

// Parses the road network file path from the command line arguments.
//  - if this is a relative path, it will be resolved against the current working directory.
//  - if this is an absolute path, it will be used as is.
use std::path::PathBuf;
fn parse_file_path(path: &str) -> PathBuf {
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
    println!("Welcome to maliput_query app:");
    println!("\t* This application allows you to query a road network built from a road network file.");
    println!("\t* Usage: maliput_query <ROAD_NETWORK_FILE_PATH> [OPTIONS]");
    println!();
    let args = Args::parse();

    // Configuration for the road network.
    let file_path = parse_file_path(args.road_network_file_path.as_str());
    let linear_tolerance: String = args.linear_tolerance.to_string();
    let max_linear_tolerance = args.max_linear_tolerance;
    let angular_tolerance = args.angular_tolerance.to_string();

    // Build backend-specific road network properties.
    let mut road_network_properties = HashMap::from([
        ("road_geometry_id", "maliput_query_rg"),
        ("linear_tolerance", linear_tolerance.as_str()),
        ("angular_tolerance", angular_tolerance.as_str()),
    ]);
    match args.backend {
        #[cfg(feature = "maliput_malidrive")]
        RoadNetworkBackend::MaliputMalidrive => {
            let omit_non_drivable_lanes = if args.allow_non_drivable_lanes { "false" } else { "true" };
            let parallel_builder_policy = !args.disable_parallel_builder_policy;
            road_network_properties.insert("opendrive_file", file_path.to_str().unwrap());
            road_network_properties.insert("omit_nondrivable_lanes", omit_non_drivable_lanes);
            road_network_properties.insert(
                "build_policy",
                if parallel_builder_policy {
                    "parallel"
                } else {
                    "sequential"
                },
            );
        }
        #[cfg(feature = "maliput_geopackage")]
        RoadNetworkBackend::MaliputGeopackage => {
            road_network_properties.insert("gpkg_file", file_path.to_str().unwrap());
        }
    }
    let max_linear_tolerance_str;
    if let Some(max_tolerance) = max_linear_tolerance {
        max_linear_tolerance_str = max_tolerance.to_string();
        road_network_properties.insert("max_linear_tolerance", max_linear_tolerance_str.as_str());
    }

    println!();
    println!("-> Setting maliput log level to: {}", args.set_log_level);
    maliput::common::set_log_level(args.set_log_level);
    println!("-> Road network builder options: {:?}", road_network_properties);
    println!("->");
    println!(
        "-> Building Maliput Road Network using backend '{}' from file: {}",
        args.backend,
        file_path.display()
    );
    println!("-> This may take a while, depending on the size of the file and the complexity of the road network...");
    println!("-> ...");
    let now = std::time::Instant::now();
    let road_network = RoadNetwork::new(args.backend, &road_network_properties)?;
    let elapsed = now.elapsed();
    println!("-> Road network loaded in {:.2?}.", elapsed);

    // Print available commands
    println!();
    MaliputQuery::print_available_queries();
    println!("To exit the application, type 'exit'.");

    loop {
        println!();
        let mut input = String::new();
        println!("-> Enter a query command (or 'exit' to quit):");
        std::io::stdin().read_line(&mut input)?;
        let input = input.trim();

        if input == "exit" {
            break;
        }

        let query: MaliputQuery = input.split_whitespace().collect::<Vec<&str>>().into();
        let query_handler = RoadNetworkQuery::new(&road_network);
        query_handler.execute_query(query)?;
    }

    Ok(())
}
