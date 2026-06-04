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

//! A TUI (Terminal User Interface) tool to interactively query a maliput road network.
//!
//! Built with [ratatui](https://ratatui.rs) for a rich terminal experience.
//!
//! ## Usage
//!
//! ```bash
//! cargo run --bin maliput_query --features tui -- maliput/data/xodr/TShapeRoad.xodr
//! cargo run --bin maliput_query --features tui -- --backend maliput_geopackage path/to/file.gpkg
//! ```
//!
//! ## Keybindings
//!
//! - `Tab` / `Shift+Tab`: Cycle focus between panels (Commands, Input, Results)
//! - `Up`/`Down`: Navigate command list or scroll results
//! - `Enter`: Execute query (when input panel focused) or select command
//! - `q` / `Ctrl+C`: Quit the application

use std::collections::HashMap;
use std::io;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use clap::Parser;
use crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use crossterm::terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen};
use crossterm::ExecutableCommand as _;
use ratatui::layout::{Constraint, Direction, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span, Text};
use ratatui::widgets::{
    Block, Borders, List, ListItem, ListState, Paragraph, Scrollbar, ScrollbarOrientation, ScrollbarState, Wrap,
};
use ratatui::{DefaultTerminal, Frame};

use maliput::api::{LanePosition, RoadNetwork, RoadNetworkBackend};

// ─── Command Definitions ───────────────────────────────────────────────────────

/// Metadata describing every query the TUI can execute.
#[derive(Clone)]
struct CommandDef {
    /// Display name shown in the command list.
    name: &'static str,
    /// Short description of the command.
    description: &'static str,
    /// Ordered list of parameter names the user must fill in.
    params: &'static [&'static str],
    /// Category for grouping in the list.
    category: CommandCategory,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum CommandCategory {
    General,
    OpenDrive,
}

impl CommandCategory {
    fn label(self) -> &'static str {
        match self {
            CommandCategory::General => "General",
            CommandCategory::OpenDrive => "OpenDRIVE / OpenSCENARIO",
        }
    }
}

/// All available commands. Order here determines list order.
fn command_definitions() -> Vec<CommandDef> {
    vec![
        // ── General ──
        CommandDef {
            name: "PrintAllLanes",
            description: "List all lanes sorted by length",
            params: &[],
            category: CommandCategory::General,
        },
        CommandDef {
            name: "GetNumberOfLanes",
            description: "Total number of lanes",
            params: &[],
            category: CommandCategory::General,
        },
        CommandDef {
            name: "GetTotalLength",
            description: "Accumulated lane lengths",
            params: &[],
            category: CommandCategory::General,
        },
        CommandDef {
            name: "GetLinearTolerance",
            description: "Road geometry linear tolerance",
            params: &[],
            category: CommandCategory::General,
        },
        CommandDef {
            name: "GetAngularTolerance",
            description: "Road geometry angular tolerance",
            params: &[],
            category: CommandCategory::General,
        },
        CommandDef {
            name: "GetLaneLength",
            description: "Length of a specific lane",
            params: &["lane_id"],
            category: CommandCategory::General,
        },
        CommandDef {
            name: "GetLaneBounds",
            description: "Lane bounds at longitudinal s",
            params: &["lane_id", "s"],
            category: CommandCategory::General,
        },
        CommandDef {
            name: "GetSegmentBounds",
            description: "Segment bounds at longitudinal s",
            params: &["lane_id", "s"],
            category: CommandCategory::General,
        },
        CommandDef {
            name: "ToRoadPosition",
            description: "Nearest road position to (x,y,z)",
            params: &["x", "y", "z"],
            category: CommandCategory::General,
        },
        CommandDef {
            name: "ToLanePosition",
            description: "Nearest position in a specific lane",
            params: &["lane_id", "x", "y", "z"],
            category: CommandCategory::General,
        },
        CommandDef {
            name: "ToSegmentPosition",
            description: "Nearest segment position in a lane",
            params: &["lane_id", "x", "y", "z"],
            category: CommandCategory::General,
        },
        CommandDef {
            name: "ToInertialPosition",
            description: "Lane position to inertial (x,y,z)",
            params: &["lane_id", "s", "r", "h"],
            category: CommandCategory::General,
        },
        CommandDef {
            name: "GetOrientation",
            description: "Road orientation at lane position",
            params: &["lane_id", "s", "r", "h"],
            category: CommandCategory::General,
        },
        CommandDef {
            name: "FindSurfaceRoadPositionsAtXY",
            description: "Find road positions near (x,y)",
            params: &["x", "y", "radius"],
            category: CommandCategory::General,
        },
        // ── OpenDRIVE / OpenSCENARIO ──
        CommandDef {
            name: "OscRoadPos→Maliput",
            description: "OpenSCENARIO road position to maliput",
            params: &["xodr_road_id", "xodr_s", "xodr_t"],
            category: CommandCategory::OpenDrive,
        },
        CommandDef {
            name: "OscLanePos→Maliput",
            description: "OpenSCENARIO lane position to maliput",
            params: &["xodr_road_id", "xodr_s", "xodr_lane_id", "offset"],
            category: CommandCategory::OpenDrive,
        },
        CommandDef {
            name: "Maliput→OscRoadPos",
            description: "Maliput position to OpenSCENARIO road",
            params: &["lane_id", "s", "r", "h"],
            category: CommandCategory::OpenDrive,
        },
        CommandDef {
            name: "Maliput→OscLanePos",
            description: "Maliput position to OpenSCENARIO lane",
            params: &["lane_id", "s", "r", "h"],
            category: CommandCategory::OpenDrive,
        },
        CommandDef {
            name: "OscRelRoadPos→Maliput",
            description: "Relative road position to maliput",
            params: &["xodr_road_id", "xodr_s", "xodr_t", "xodr_ds", "xodr_dt"],
            category: CommandCategory::OpenDrive,
        },
        CommandDef {
            name: "OscRelLaneDs→Maliput",
            description: "Relative lane position (ds) to maliput",
            params: &["xodr_road_id", "xodr_lane_id", "xodr_s", "d_lane", "xodr_ds", "offset"],
            category: CommandCategory::OpenDrive,
        },
        CommandDef {
            name: "OscRelLaneDsLane→Maliput",
            description: "Relative lane position (ds_lane) to maliput",
            params: &[
                "xodr_road_id",
                "xodr_lane_id",
                "xodr_s",
                "d_lane",
                "xodr_ds_lane",
                "offset",
            ],
            category: CommandCategory::OpenDrive,
        },
        CommandDef {
            name: "GetRoadOrientationAtOsc",
            description: "Orientation at OpenSCENARIO road pos",
            params: &["xodr_road_id", "xodr_s", "xodr_t"],
            category: CommandCategory::OpenDrive,
        },
    ]
}

// ─── TUI Application State ────────────────────────────────────────────────────

/// Which panel currently has focus.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Focus {
    Commands,
    Input,
    Results,
}

impl Focus {
    fn next(self) -> Self {
        match self {
            Focus::Commands => Focus::Input,
            Focus::Input => Focus::Results,
            Focus::Results => Focus::Commands,
        }
    }
    fn prev(self) -> Self {
        match self {
            Focus::Commands => Focus::Results,
            Focus::Input => Focus::Commands,
            Focus::Results => Focus::Input,
        }
    }
}

/// One entry in the results history.
struct QueryResult {
    command_name: String,
    elapsed: Duration,
    lines: Vec<String>,
}

/// The main application state.
struct App {
    /// All command definitions.
    commands: Vec<CommandDef>,
    /// ListState for the command list widget.
    command_list_state: ListState,
    /// Current focus panel.
    focus: Focus,
    /// Current parameter input values, indexed by parameter position.
    param_values: Vec<String>,
    /// Which parameter field is currently being edited (index into param_values).
    active_param: usize,
    /// History of query results (newest first).
    results: Vec<QueryResult>,
    /// Scroll offset for the results panel.
    results_scroll: usize,
    /// Total visible lines in the results panel (updated on each render).
    results_total_lines: usize,
    /// Road network summary info (shown in the header).
    rn_info: RoadNetworkInfo,
    /// Should the app quit?
    should_quit: bool,
    /// Status message (e.g. errors from bad input).
    status_message: Option<(String, Instant)>,
}

struct RoadNetworkInfo {
    file_path: String,
    backend: String,
    load_time: Duration,
    num_lanes: usize,
    total_length: f64,
    linear_tolerance: f64,
    angular_tolerance: f64,
}

impl App {
    fn new(rn: &RoadNetwork, file_path: &str, backend: &str, load_time: Duration) -> Self {
        let commands = command_definitions();
        let rg = rn.road_geometry();
        let lanes = rg.get_lanes();
        let total_length: f64 = lanes.iter().map(|l| l.length()).sum();
        let rn_info = RoadNetworkInfo {
            file_path: file_path.to_string(),
            backend: backend.to_string(),
            load_time,
            num_lanes: lanes.len(),
            total_length,
            linear_tolerance: rg.linear_tolerance(),
            angular_tolerance: rg.angular_tolerance(),
        };

        let mut command_list_state = ListState::default();
        command_list_state.select(Some(0));

        let initial_params = commands[0].params.iter().map(|_| String::new()).collect();

        App {
            commands,
            command_list_state,
            focus: Focus::Commands,
            param_values: initial_params,
            active_param: 0,
            results: Vec::new(),
            results_scroll: 0,
            results_total_lines: 0,
            rn_info,
            should_quit: false,
            status_message: None,
        }
    }

    fn selected_command(&self) -> &CommandDef {
        let idx = self.command_list_state.selected().unwrap_or(0);
        &self.commands[idx]
    }

    fn select_command(&mut self, idx: usize) {
        self.command_list_state.select(Some(idx));
        // Reset params for the newly selected command.
        let cmd = &self.commands[idx];
        self.param_values = cmd.params.iter().map(|_| String::new()).collect();
        self.active_param = 0;
    }

    fn set_status(&mut self, msg: String) {
        self.status_message = Some((msg, Instant::now()));
    }

    /// Execute the currently selected command with the filled-in parameter values.
    fn execute(&mut self, rn: &RoadNetwork) {
        let cmd = self.selected_command().clone();
        let params = self.param_values.clone();

        // Validate that all params are filled.
        for (i, p) in cmd.params.iter().enumerate() {
            if params[i].trim().is_empty() {
                self.set_status(format!("Parameter '{}' is required.", p));
                return;
            }
        }

        let start = Instant::now();
        match execute_command(rn, &cmd, &params) {
            Ok(lines) => {
                let elapsed = start.elapsed();
                self.results.insert(
                    0,
                    QueryResult {
                        command_name: cmd.name.to_string(),
                        elapsed,
                        lines,
                    },
                );
                self.results_scroll = 0;
                self.status_message = None;
            }
            Err(e) => {
                self.set_status(format!("Error: {}", e));
            }
        }
    }
}

// ─── Query Execution ───────────────────────────────────────────────────────────

/// Execute a command against the road network and return result lines.
fn execute_command(
    rn: &RoadNetwork,
    cmd: &CommandDef,
    params: &[String],
) -> Result<Vec<String>, Box<dyn std::error::Error>> {
    let rg = rn.road_geometry();
    let mut out: Vec<String> = Vec::new();

    match cmd.name {
        "PrintAllLanes" => {
            let mut lanes = rg.get_lanes();
            lanes.sort_by(|a, b| b.length().partial_cmp(&a.length()).unwrap());
            out.push(format!("All lanes: {} total", lanes.len()));
            for lane in lanes {
                let ip = lane.to_inertial_position(&LanePosition::new(0., 0., 0.))?;
                out.push(format!(
                    "  {} | len={:.3}m | pos(0,0,0)={}",
                    lane.id(),
                    lane.length(),
                    ip
                ));
            }
        }
        "GetNumberOfLanes" => {
            out.push(format!("Number of lanes: {}", rg.get_lanes().len()));
        }
        "GetTotalLength" => {
            let lanes = rg.get_lanes();
            let total: f64 = lanes.iter().map(|l| l.length()).sum();
            let avg = total / lanes.len() as f64;
            out.push(format!("Total length: {:.3} m ({} lanes)", total, lanes.len()));
            out.push(format!("Average lane length: {:.3} m", avg));
        }
        "GetLinearTolerance" => {
            out.push(format!("Linear tolerance: {} m", rg.linear_tolerance()));
        }
        "GetAngularTolerance" => {
            out.push(format!("Angular tolerance: {} rad", rg.angular_tolerance()));
        }
        "GetLaneLength" => {
            let lane_id = &params[0];
            if let Some(lane) = rg.get_lane(lane_id) {
                out.push(format!("Lane '{}' length: {:.3} m", lane_id, lane.length()));
            } else {
                out.push(format!("Lane '{}' not found.", lane_id));
            }
        }
        "GetLaneBounds" => {
            let lane_id = &params[0];
            let s: f64 = params[1].parse()?;
            if let Some(lane) = rg.get_lane(lane_id) {
                let bounds = lane.lane_bounds(s)?;
                out.push(format!(
                    "Lane '{}' bounds at s={}: min={}, max={}",
                    lane_id,
                    s,
                    bounds.min(),
                    bounds.max()
                ));
            } else {
                out.push(format!("Lane '{}' not found.", lane_id));
            }
        }
        "GetSegmentBounds" => {
            let lane_id = &params[0];
            let s: f64 = params[1].parse()?;
            if let Some(lane) = rg.get_lane(lane_id) {
                let bounds = lane.segment_bounds(s)?;
                out.push(format!(
                    "Lane '{}' segment bounds at s={}: min={}, max={}",
                    lane_id,
                    s,
                    bounds.min(),
                    bounds.max()
                ));
            } else {
                out.push(format!("Lane '{}' not found.", lane_id));
            }
        }
        "ToRoadPosition" => {
            let x: f64 = params[0].parse()?;
            let y: f64 = params[1].parse()?;
            let z: f64 = params[2].parse()?;
            let result = rg.to_road_position(&maliput::api::InertialPosition::new(x, y, z))?;
            out.push("Road Position Result:".to_string());
            out.push(format!("  lane_id: {}", result.road_position.lane().id()));
            out.push(format!("  lane_position: {}", result.road_position.pos()));
            out.push(format!("  nearest_position: {}", result.nearest_position));
            out.push(format!("  distance: {}", result.distance));
        }
        "FindSurfaceRoadPositionsAtXY" => {
            let x: f64 = params[0].parse()?;
            let y: f64 = params[1].parse()?;
            let radius: f64 = params[2].parse()?;
            let positions = rg.find_surface_road_positions_at_xy(x, y, radius)?;
            out.push(format!("Found {} position(s):", positions.len()));
            for (i, rpr) in positions.iter().enumerate() {
                out.push(format!("  Result #{}:", i));
                out.push(format!("    lane_id: {}", rpr.road_position.lane().id()));
                out.push(format!("    lane_position: {}", rpr.road_position.pos()));
                out.push(format!("    nearest_position: {}", rpr.nearest_position));
                out.push(format!("    distance: {}", rpr.distance));
            }
        }
        "ToLanePosition" => {
            let lane_id = &params[0];
            let x: f64 = params[1].parse()?;
            let y: f64 = params[2].parse()?;
            let z: f64 = params[3].parse()?;
            if let Some(lane) = rg.get_lane(lane_id) {
                let result = lane.to_lane_position(&maliput::api::InertialPosition::new(x, y, z))?;
                out.push(format!("Lane Position Result (lane '{}'):", lane_id));
                out.push(format!("  lane_position: {}", result.lane_position));
                out.push(format!("  nearest_position: {}", result.nearest_position));
                out.push(format!("  distance: {}", result.distance));
            } else {
                out.push(format!("Lane '{}' not found.", lane_id));
            }
        }
        "ToSegmentPosition" => {
            let lane_id = &params[0];
            let x: f64 = params[1].parse()?;
            let y: f64 = params[2].parse()?;
            let z: f64 = params[3].parse()?;
            if let Some(lane) = rg.get_lane(lane_id) {
                let result = lane.to_segment_position(&maliput::api::InertialPosition::new(x, y, z))?;
                out.push(format!("Segment Position Result (lane '{}'):", lane_id));
                out.push(format!("  lane_position: {}", result.lane_position));
                out.push(format!("  nearest_position: {}", result.nearest_position));
                out.push(format!("  distance: {}", result.distance));
            } else {
                out.push(format!("Lane '{}' not found.", lane_id));
            }
        }
        "ToInertialPosition" => {
            let lane_id = &params[0];
            let s: f64 = params[1].parse()?;
            let r: f64 = params[2].parse()?;
            let h: f64 = params[3].parse()?;
            if let Some(lane) = rg.get_lane(lane_id) {
                let ip = lane.to_inertial_position(&maliput::api::LanePosition::new(s, r, h))?;
                out.push(format!("Inertial Position (lane '{}'):", lane_id));
                out.push(format!("  position: {}", ip));
            } else {
                out.push(format!("Lane '{}' not found.", lane_id));
            }
        }
        "GetOrientation" => {
            let lane_id = &params[0];
            let s: f64 = params[1].parse()?;
            let r: f64 = params[2].parse()?;
            let h: f64 = params[3].parse()?;
            if let Some(lane) = rg.get_lane(lane_id) {
                let orient = lane.get_orientation(&maliput::api::LanePosition::new(s, r, h))?;
                out.push(format!("Orientation (lane '{}'):", lane_id));
                out.push(format!(
                    "  roll={:.6}  pitch={:.6}  yaw={:.6}",
                    orient.roll(),
                    orient.pitch(),
                    orient.yaw()
                ));
            } else {
                out.push(format!("Lane '{}' not found.", lane_id));
            }
        }
        // ── OpenDRIVE / OpenSCENARIO backend custom commands ──
        "OscRoadPos→Maliput" => {
            let command = format!(
                "OpenScenarioRoadPositionToMaliputRoadPosition,{},{},{}",
                params[0], params[1], params[2]
            );
            let res = rg.backend_custom_command(&command)?;
            let parts: Vec<&str> = res.split(',').collect();
            out.push("OpenSCENARIO Road Position → Maliput:".to_string());
            if parts.len() == 4 {
                out.push(format!("  lane_id: {}", parts[0]));
                out.push(format!("  s: {}  r: {}  h: {}", parts[1], parts[2], parts[3]));
            } else {
                out.push(format!("  Unexpected response: {}", res));
            }
        }
        "OscLanePos→Maliput" => {
            let command = format!(
                "OpenScenarioLanePositionToMaliputRoadPosition,{},{},{},{}",
                params[0], params[1], params[2], params[3]
            );
            let res = rg.backend_custom_command(&command)?;
            let parts: Vec<&str> = res.split(',').collect();
            out.push("OpenSCENARIO Lane Position → Maliput:".to_string());
            if parts.len() == 4 {
                out.push(format!("  lane_id: {}", parts[0]));
                out.push(format!("  s: {}  r: {}  h: {}", parts[1], parts[2], parts[3]));
            } else {
                out.push(format!("  Unexpected response: {}", res));
            }
        }
        "Maliput→OscRoadPos" => {
            let command = format!(
                "MaliputRoadPositionToOpenScenarioRoadPosition,{},{},{},{}",
                params[0], params[1], params[2], params[3]
            );
            let res = rg.backend_custom_command(&command)?;
            let parts: Vec<&str> = res.split(',').collect();
            out.push("Maliput → OpenSCENARIO Road Position:".to_string());
            if parts.len() == 3 {
                out.push(format!("  xodr_road_id: {}", parts[0]));
                out.push(format!("  xodr_s: {}  xodr_t: {}", parts[1], parts[2]));
            } else {
                out.push(format!("  Unexpected response: {}", res));
            }
        }
        "Maliput→OscLanePos" => {
            let command = format!(
                "MaliputRoadPositionToOpenScenarioLanePosition,{},{},{},{}",
                params[0], params[1], params[2], params[3]
            );
            let res = rg.backend_custom_command(&command)?;
            let parts: Vec<&str> = res.split(',').collect();
            out.push("Maliput → OpenSCENARIO Lane Position:".to_string());
            if parts.len() == 4 {
                out.push(format!("  xodr_road_id: {}", parts[0]));
                out.push(format!(
                    "  xodr_s: {}  xodr_lane_id: {}  offset: {}",
                    parts[1], parts[2], parts[3]
                ));
            } else {
                out.push(format!("  Unexpected response: {}", res));
            }
        }
        "OscRelRoadPos→Maliput" => {
            let command = format!(
                "OpenScenarioRelativeRoadPositionToMaliputRoadPosition,{},{},{},{},{}",
                params[0], params[1], params[2], params[3], params[4]
            );
            let res = rg.backend_custom_command(&command)?;
            let parts: Vec<&str> = res.split(',').collect();
            out.push("OpenSCENARIO Relative Road Position → Maliput:".to_string());
            if parts.len() == 4 {
                out.push(format!("  lane_id: {}", parts[0]));
                out.push(format!("  s: {}  r: {}  h: {}", parts[1], parts[2], parts[3]));
            } else {
                out.push(format!("  Unexpected response: {}", res));
            }
        }
        "OscRelLaneDs→Maliput" => {
            let command = format!(
                "OpenScenarioRelativeLanePositionWithDsToMaliputRoadPosition,{},{},{},{},{},{}",
                params[0], params[1], params[2], params[3], params[4], params[5]
            );
            let res = rg.backend_custom_command(&command)?;
            let parts: Vec<&str> = res.split(',').collect();
            out.push("OpenSCENARIO Relative Lane Pos (ds) → Maliput:".to_string());
            if parts.len() == 4 {
                out.push(format!("  lane_id: {}", parts[0]));
                out.push(format!("  s: {}  r: {}  h: {}", parts[1], parts[2], parts[3]));
            } else {
                out.push(format!("  Unexpected response: {}", res));
            }
        }
        "OscRelLaneDsLane→Maliput" => {
            let command = format!(
                "OpenScenarioRelativeLanePositionWithDsLaneToMaliputRoadPosition,{},{},{},{},{},{}",
                params[0], params[1], params[2], params[3], params[4], params[5]
            );
            let res = rg.backend_custom_command(&command)?;
            let parts: Vec<&str> = res.split(',').collect();
            out.push("OpenSCENARIO Relative Lane Pos (ds_lane) → Maliput:".to_string());
            if parts.len() == 4 {
                out.push(format!("  lane_id: {}", parts[0]));
                out.push(format!("  s: {}  r: {}  h: {}", parts[1], parts[2], parts[3]));
            } else {
                out.push(format!("  Unexpected response: {}", res));
            }
        }
        "GetRoadOrientationAtOsc" => {
            let command = format!(
                "GetRoadOrientationAtOpenScenarioRoadPosition,{},{},{}",
                params[0], params[1], params[2]
            );
            let res = rg.backend_custom_command(&command)?;
            let parts: Vec<&str> = res.split(',').collect();
            out.push("Orientation at OpenSCENARIO Road Position:".to_string());
            if parts.len() == 3 {
                out.push(format!("  roll: {}  pitch: {}  yaw: {}", parts[0], parts[1], parts[2]));
            } else {
                out.push(format!("  Unexpected response: {}", res));
            }
        }
        _ => {
            out.push(format!("Unknown command: {}", cmd.name));
        }
    }

    Ok(out)
}

// ─── TUI Rendering ─────────────────────────────────────────────────────────────

fn draw(frame: &mut Frame, app: &mut App) {
    let size = frame.area();

    // ┌─────────────────────────────────────────────────────────────────────┐
    // │ Header (road network info)                                         │
    // ├───────────────────┬─────────────────────────────────────────────────┤
    // │ Commands (left)   │ Input + Results (right)                        │
    // │                   │  ┌── Input ──────────────────────────────────┐  │
    // │                   │  │                                           │  │
    // │                   │  ├── Results ────────────────────────────────┤  │
    // │                   │  │                                           │  │
    // │                   │  └───────────────────────────────────────────┘  │
    // ├───────────────────┴─────────────────────────────────────────────────┤
    // │ Help bar                                                           │
    // └─────────────────────────────────────────────────────────────────────┘

    let outer_layout = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3), // Header
            Constraint::Min(10),   // Main body
            Constraint::Length(2), // Help bar
        ])
        .split(size);

    draw_header(frame, app, outer_layout[0]);
    draw_body(frame, app, outer_layout[1]);
    draw_help_bar(frame, app, outer_layout[2]);
}

fn draw_header(frame: &mut Frame, app: &App, area: Rect) {
    let info = &app.rn_info;
    let header_text = format!(
        " {} | backend: {} | loaded in {:.2?} | {} lanes | {:.1} m total | lin_tol: {} | ang_tol: {}",
        info.file_path,
        info.backend,
        info.load_time,
        info.num_lanes,
        info.total_length,
        info.linear_tolerance,
        info.angular_tolerance,
    );
    let header = Paragraph::new(header_text)
        .style(Style::default().fg(Color::White).bg(Color::DarkGray))
        .block(
            Block::default()
                .borders(Borders::BOTTOM)
                .border_style(Style::default().fg(Color::Gray))
                .title(" maliput_query TUI ")
                .title_style(Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
        );
    frame.render_widget(header, area);
}

fn draw_body(frame: &mut Frame, app: &mut App, area: Rect) {
    let body_layout = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Percentage(30), // Command list
            Constraint::Percentage(70), // Input + Results
        ])
        .split(area);

    draw_command_list(frame, app, body_layout[0]);
    draw_right_panel(frame, app, body_layout[1]);
}

fn draw_command_list(frame: &mut Frame, app: &mut App, area: Rect) {
    let commands = &app.commands;
    let is_focused = app.focus == Focus::Commands;

    // Build list items with category headers.
    let mut items: Vec<ListItem> = Vec::new();
    let mut last_category: Option<CommandCategory> = None;
    let mut list_index_to_cmd_index: Vec<Option<usize>> = Vec::new();

    for (i, cmd) in commands.iter().enumerate() {
        if last_category != Some(cmd.category) {
            // Add a category header.
            items.push(
                ListItem::new(Line::from(vec![Span::styled(
                    format!("── {} ──", cmd.category.label()),
                    Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD),
                )]))
                .style(Style::default()),
            );
            list_index_to_cmd_index.push(None);
            last_category = Some(cmd.category);
        }
        let style = if app.command_list_state.selected() == Some(i) {
            Style::default()
                .fg(Color::Black)
                .bg(Color::Cyan)
                .add_modifier(Modifier::BOLD)
        } else {
            Style::default().fg(Color::White)
        };
        items.push(ListItem::new(format!("  {}", cmd.name)).style(style));
        list_index_to_cmd_index.push(Some(i));
    }

    let border_style = if is_focused {
        Style::default().fg(Color::Cyan)
    } else {
        Style::default().fg(Color::Gray)
    };

    let title = if is_focused {
        " Commands [▲▼ navigate] "
    } else {
        " Commands "
    };

    let list = List::new(items).block(
        Block::default()
            .borders(Borders::ALL)
            .border_style(border_style)
            .title(title)
            .title_style(Style::default().fg(Color::White).add_modifier(Modifier::BOLD)),
    );

    // We render the list as a plain widget (not stateful) since we handle
    // highlighting manually above.
    frame.render_widget(list, area);
}

fn draw_right_panel(frame: &mut Frame, app: &mut App, area: Rect) {
    let cmd = app.selected_command().clone();
    let input_height = if cmd.params.is_empty() {
        4 // Minimal height for "no parameters" message
    } else {
        // 2 (borders) + 1 (description) + 1 (blank) + params + 1 (status/hint)
        (cmd.params.len() as u16) + 5
    };

    let right_layout = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(input_height), // Input panel
            Constraint::Min(5),               // Results panel
        ])
        .split(area);

    draw_input_panel(frame, app, &cmd, right_layout[0]);
    draw_results_panel(frame, app, right_layout[1]);
}

fn draw_input_panel(frame: &mut Frame, app: &App, cmd: &CommandDef, area: Rect) {
    let is_focused = app.focus == Focus::Input;

    let border_style = if is_focused {
        Style::default().fg(Color::Green)
    } else {
        Style::default().fg(Color::Gray)
    };

    let title = format!(" {} ", cmd.name);
    let block = Block::default()
        .borders(Borders::ALL)
        .border_style(border_style)
        .title(title)
        .title_style(Style::default().fg(Color::White).add_modifier(Modifier::BOLD));

    let inner = block.inner(area);
    frame.render_widget(block, area);

    if cmd.params.is_empty() {
        let msg = if is_focused {
            "No parameters needed. Press Enter to execute."
        } else {
            "No parameters needed."
        };
        let p = Paragraph::new(msg).style(Style::default().fg(Color::DarkGray));
        frame.render_widget(p, inner);
        return;
    }

    // Render description line.
    let mut lines: Vec<Line> = Vec::new();
    lines.push(Line::from(Span::styled(
        cmd.description,
        Style::default().fg(Color::DarkGray).add_modifier(Modifier::ITALIC),
    )));
    lines.push(Line::from("")); // blank separator

    // Render parameter fields.
    for (i, param_name) in cmd.params.iter().enumerate() {
        let is_active = is_focused && i == app.active_param;
        let label_style = if is_active {
            Style::default().fg(Color::Green).add_modifier(Modifier::BOLD)
        } else {
            Style::default().fg(Color::White)
        };
        let value = &app.param_values[i];
        let cursor = if is_active { "█" } else { "" };
        let value_style = if is_active {
            Style::default().fg(Color::Cyan)
        } else {
            Style::default().fg(Color::Gray)
        };
        let indicator = if is_active { "▶ " } else { "  " };

        lines.push(Line::from(vec![
            Span::styled(indicator, label_style),
            Span::styled(format!("{}: ", param_name), label_style),
            Span::styled(value.clone(), value_style),
            Span::styled(cursor.to_string(), Style::default().fg(Color::Green)),
        ]));
    }

    // Status message.
    if let Some((ref msg, ref time)) = app.status_message {
        if time.elapsed() < Duration::from_secs(5) {
            lines.push(Line::from(Span::styled(msg.clone(), Style::default().fg(Color::Red))));
        }
    }

    let paragraph = Paragraph::new(Text::from(lines));
    frame.render_widget(paragraph, inner);
}

fn draw_results_panel(frame: &mut Frame, app: &mut App, area: Rect) {
    let is_focused = app.focus == Focus::Results;

    let border_style = if is_focused {
        Style::default().fg(Color::Magenta)
    } else {
        Style::default().fg(Color::Gray)
    };

    let title = if app.results.is_empty() {
        " Results "
    } else if is_focused {
        " Results [▲▼ scroll] "
    } else {
        " Results "
    };

    let block = Block::default()
        .borders(Borders::ALL)
        .border_style(border_style)
        .title(title)
        .title_style(Style::default().fg(Color::White).add_modifier(Modifier::BOLD));

    let inner = block.inner(area);
    frame.render_widget(block, area);

    if app.results.is_empty() {
        let p = Paragraph::new("Execute a query to see results here.").style(Style::default().fg(Color::DarkGray));
        frame.render_widget(p, inner);
        return;
    }

    // Build all result lines.
    let mut all_lines: Vec<Line> = Vec::new();
    for (ri, result) in app.results.iter().enumerate() {
        if ri > 0 {
            all_lines.push(Line::from(""));
            all_lines.push(Line::from(Span::styled(
                "───────────────────────────────────────────",
                Style::default().fg(Color::DarkGray),
            )));
            all_lines.push(Line::from(""));
        }
        // Query header.
        all_lines.push(Line::from(vec![
            Span::styled("● ", Style::default().fg(Color::Green)),
            Span::styled(
                &result.command_name,
                Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD),
            ),
            Span::styled(
                format!("  ({:.2?})", result.elapsed),
                Style::default().fg(Color::DarkGray),
            ),
        ]));
        for line in &result.lines {
            all_lines.push(Line::from(line.as_str()));
        }
    }

    app.results_total_lines = all_lines.len();

    // Apply scroll offset.
    let visible_height = inner.height as usize;
    let max_scroll = all_lines.len().saturating_sub(visible_height);
    if app.results_scroll > max_scroll {
        app.results_scroll = max_scroll;
    }

    let paragraph = Paragraph::new(Text::from(all_lines))
        .scroll((app.results_scroll as u16, 0))
        .wrap(Wrap { trim: false });
    frame.render_widget(paragraph, inner);

    // Scrollbar.
    if app.results_total_lines > visible_height {
        let scrollbar = Scrollbar::new(ScrollbarOrientation::VerticalRight);
        let mut scrollbar_state =
            ScrollbarState::new(app.results_total_lines.saturating_sub(visible_height)).position(app.results_scroll);
        // Render scrollbar in the results area (right edge).
        frame.render_stateful_widget(
            scrollbar,
            area.inner(ratatui::layout::Margin {
                vertical: 1,
                horizontal: 0,
            }),
            &mut scrollbar_state,
        );
    }
}

fn draw_help_bar(frame: &mut Frame, _app: &App, area: Rect) {
    let keys = vec![
        ("Tab/S-Tab", "switch panel"),
        ("↑/↓", "navigate"),
        ("Enter", "execute"),
        ("Esc", "back to commands"),
        ("q/Ctrl+C", "quit"),
    ];
    let spans: Vec<Span> = keys
        .into_iter()
        .flat_map(|(key, desc)| {
            vec![
                Span::styled(
                    format!(" {} ", key),
                    Style::default()
                        .fg(Color::Black)
                        .bg(Color::Gray)
                        .add_modifier(Modifier::BOLD),
                ),
                Span::styled(format!(" {}  ", desc), Style::default().fg(Color::DarkGray)),
            ]
        })
        .collect();

    let help = Paragraph::new(Line::from(spans)).style(Style::default());
    frame.render_widget(help, area);
}

// ─── Event Handling ─────────────────────────────────────────────────────────────

fn handle_events(app: &mut App, rn: &RoadNetwork) -> io::Result<()> {
    if event::poll(Duration::from_millis(50))? {
        if let Event::Key(key) = event::read()? {
            if key.kind != KeyEventKind::Press {
                return Ok(());
            }

            // Global keybindings.
            match key.code {
                KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                    app.should_quit = true;
                    return Ok(());
                }
                KeyCode::Char('q') if app.focus != Focus::Input => {
                    app.should_quit = true;
                    return Ok(());
                }
                KeyCode::Tab => {
                    app.focus = app.focus.next();
                    return Ok(());
                }
                KeyCode::BackTab => {
                    app.focus = app.focus.prev();
                    return Ok(());
                }
                KeyCode::Esc => {
                    app.focus = Focus::Commands;
                    return Ok(());
                }
                _ => {}
            }

            // Panel-specific keybindings.
            match app.focus {
                Focus::Commands => handle_commands_input(app, key.code),
                Focus::Input => handle_input_panel(app, rn, key.code),
                Focus::Results => handle_results_input(app, key.code),
            }
        }
    }
    Ok(())
}

fn handle_commands_input(app: &mut App, key: KeyCode) {
    let num_commands = app.commands.len();
    match key {
        KeyCode::Up => {
            let current = app.command_list_state.selected().unwrap_or(0);
            let new_idx = if current == 0 { num_commands - 1 } else { current - 1 };
            app.select_command(new_idx);
        }
        KeyCode::Down => {
            let current = app.command_list_state.selected().unwrap_or(0);
            let new_idx = if current >= num_commands - 1 { 0 } else { current + 1 };
            app.select_command(new_idx);
        }
        KeyCode::Enter => {
            // Move focus to input panel.
            app.focus = Focus::Input;
        }
        _ => {}
    }
}

fn handle_input_panel(app: &mut App, rn: &RoadNetwork, key: KeyCode) {
    let cmd = app.selected_command().clone();
    match key {
        KeyCode::Enter => {
            app.execute(rn);
        }
        KeyCode::Up => {
            if app.active_param > 0 {
                app.active_param -= 1;
            }
        }
        KeyCode::Down => {
            if !cmd.params.is_empty() && app.active_param < cmd.params.len() - 1 {
                app.active_param += 1;
            }
        }
        KeyCode::Char(c) => {
            if !cmd.params.is_empty() && app.active_param < app.param_values.len() {
                app.param_values[app.active_param].push(c);
            }
        }
        KeyCode::Backspace => {
            if !cmd.params.is_empty() && app.active_param < app.param_values.len() {
                app.param_values[app.active_param].pop();
            }
        }
        _ => {}
    }
}

fn handle_results_input(app: &mut App, key: KeyCode) {
    match key {
        KeyCode::Up => {
            app.results_scroll = app.results_scroll.saturating_sub(1);
        }
        KeyCode::Down => {
            app.results_scroll += 1;
        }
        KeyCode::PageUp => {
            app.results_scroll = app.results_scroll.saturating_sub(10);
        }
        KeyCode::PageDown => {
            app.results_scroll += 10;
        }
        KeyCode::Home => {
            app.results_scroll = 0;
        }
        _ => {}
    }
}

// ─── CLI Arguments ──────────────────────────────────────────────────────────────

#[derive(Parser, Debug)]
#[command(author, version, about = "maliput_query TUI — interactive road network explorer")]
struct Args {
    /// Path to the road network file (e.g., .xodr for maliput_malidrive, .gpkg for maliput_geopackage).
    road_network_file_path: String,

    /// The maliput backend to use.
    #[cfg(feature = "maliput_malidrive")]
    #[arg(short, long, default_value_t = RoadNetworkBackend::MaliputMalidrive)]
    backend: RoadNetworkBackend,

    /// The maliput backend to use.
    #[cfg(all(not(feature = "maliput_malidrive"), feature = "maliput_geopackage"))]
    #[arg(short, long, default_value_t = RoadNetworkBackend::MaliputGeopackage)]
    backend: RoadNetworkBackend,

    /// Linear tolerance for the road geometry.
    #[arg(short, long, default_value_t = 0.05)]
    linear_tolerance: f64,

    #[arg(short, long)]
    max_linear_tolerance: Option<f64>,

    /// Angular tolerance for the road geometry.
    #[arg(short, long, default_value_t = 0.01)]
    angular_tolerance: f64,

    /// Include non-drivable lanes.
    #[arg(long, default_value_t = false)]
    allow_non_drivable_lanes: bool,

    /// Use sequential builder policy.
    #[arg(long, default_value_t = false)]
    disable_parallel_builder_policy: bool,

    /// Maliput C++ log level.
    #[arg(long, default_value_t = maliput::common::LogLevel::Error)]
    set_log_level: maliput::common::LogLevel,
}

fn parse_file_path(path: &str) -> PathBuf {
    let mut path_buf = PathBuf::from(path);
    if !path_buf.is_absolute() {
        if let Ok(current_dir) = std::env::current_dir() {
            path_buf = current_dir.join(path_buf);
        }
    }
    path_buf
}

// ─── Main ───────────────────────────────────────────────────────────────────────

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    let file_path = parse_file_path(args.road_network_file_path.as_str());
    let linear_tolerance = args.linear_tolerance.to_string();
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

    maliput::common::set_log_level(args.set_log_level);

    // ── Load road network (before entering TUI mode) ──
    eprintln!(
        "Loading road network from {}  (backend: {})...",
        file_path.display(),
        args.backend
    );
    let now = Instant::now();
    let road_network = RoadNetwork::new(args.backend, &road_network_properties)?;
    let load_time = now.elapsed();
    eprintln!("Loaded in {:.2?}.", load_time);

    // ── Enter TUI ──
    enable_raw_mode()?;
    io::stdout().execute(EnterAlternateScreen)?;
    let mut terminal = ratatui::init();

    let mut app = App::new(
        &road_network,
        file_path.to_str().unwrap_or("unknown"),
        &args.backend.to_string(),
        load_time,
    );

    let result = run_app(&mut terminal, &mut app, &road_network);

    // ── Restore terminal ──
    ratatui::restore();
    disable_raw_mode()?;
    io::stdout().execute(LeaveAlternateScreen)?;

    result?;
    Ok(())
}

fn run_app(terminal: &mut DefaultTerminal, app: &mut App, rn: &RoadNetwork) -> Result<(), Box<dyn std::error::Error>> {
    while !app.should_quit {
        terminal.draw(|frame| draw(frame, app))?;
        handle_events(app, rn)?;
    }
    Ok(())
}
