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

//! Integration tests for the `maliput_query` tool.
//!
//! The binary uses a ratatui TUI, so interactive stdin/stdout testing is not
//! possible.  CLI-level tests (help, version, bad args) are gated behind
//! `#[cfg(feature = "tui")]` because the binary requires that feature.
//!
//! Query-level tests exercise the same maliput API that the TUI calls
//! internally, verifying the results directly through the library.

mod common;

use maliput::api::{InertialPosition, RoadNetwork, RoadNetworkBackend};
use std::collections::HashMap;

/// Helper: get the path to a test xodr file shipped with the crate.
fn xodr_path(name: &str) -> String {
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    format!("{}/data/xodr/{}", manifest_dir, name)
}

// ═══════════════════════════════════════════════════════════════════════════
//  CLI argument validation (requires the `tui` feature so the binary exists)
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(feature = "tui")]
mod cli {
    use assert_cmd::Command;
    use predicates::prelude::*;

    /// Helper: build a `Command` for the `maliput_query` binary.
    #[allow(deprecated)]
    fn maliput_query_cmd() -> Command {
        Command::cargo_bin("maliput_query").expect("binary maliput_query should be built")
    }

    #[test]
    fn no_args_shows_error() {
        maliput_query_cmd()
            .assert()
            .failure()
            .stderr(predicate::str::contains("Usage"));
    }

    #[test]
    fn help_flag_shows_usage() {
        maliput_query_cmd()
            .arg("--help")
            .assert()
            .success()
            .stdout(predicate::str::contains("Usage"))
            .stdout(predicate::str::contains("ROAD_NETWORK_FILE_PATH"));
    }

    #[test]
    fn version_flag() {
        maliput_query_cmd()
            .arg("--version")
            .assert()
            .success()
            .stdout(predicate::str::is_empty().not());
    }

    #[test]
    fn invalid_file_path_fails() {
        maliput_query_cmd()
            .arg("/nonexistent/path/fake.xodr")
            .assert()
            .failure();
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Query-level tests — exercise the maliput API the TUI relies on
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn loads_road_network() {
    let rn = common::create_t_shape_road_network(true);
    let rg = rn.road_geometry();
    assert!(!rg.id().is_empty(), "Road geometry should have an id");
    assert!(
        !rg.get_lanes().is_empty(),
        "Road geometry should have at least one lane"
    );
}

#[test]
fn get_number_of_lanes() {
    let rn = common::create_t_shape_road_network(true);
    let rg = rn.road_geometry();
    let num_lanes = rg.get_lanes().len();
    assert!(num_lanes > 0, "Expected at least one lane");
}

#[test]
fn get_linear_tolerance() {
    let rn = common::create_t_shape_road_network(true);
    let rg = rn.road_geometry();
    let tol = rg.linear_tolerance();
    assert!(tol > 0.0, "Linear tolerance should be positive");
}

#[test]
fn get_angular_tolerance() {
    let rn = common::create_t_shape_road_network(true);
    let rg = rn.road_geometry();
    let tol = rg.angular_tolerance();
    assert!(tol > 0.0, "Angular tolerance should be positive");
}

#[test]
fn get_total_length() {
    let rn = common::create_t_shape_road_network(true);
    let rg = rn.road_geometry();
    let lanes = rg.get_lanes();
    let total: f64 = lanes.iter().map(|l| l.length()).sum();
    assert!(total > 0.0, "Total lane length should be positive");
}

#[test]
fn print_all_lanes() {
    let rn = common::create_t_shape_road_network(true);
    let rg = rn.road_geometry();
    let lanes = rg.get_lanes();
    for lane in &lanes {
        assert!(!lane.id().is_empty(), "Lane should have an id");
        assert!(lane.length() > 0.0, "Lane should have positive length");
    }
}

#[test]
fn to_road_position() {
    let rn = common::create_t_shape_road_network(true);
    let rg = rn.road_geometry();
    let result = rg
        .to_road_position(&InertialPosition::new(0.0, 0.0, 0.0))
        .expect("to_road_position should succeed");
    assert!(
        !result.road_position.lane().id().is_empty(),
        "Result should have a lane id"
    );
    assert!(result.distance >= 0.0, "Distance should be non-negative");
}

#[test]
fn multiple_queries_in_sequence() {
    let rn = common::create_t_shape_road_network(true);
    let rg = rn.road_geometry();

    // Query 1: number of lanes
    let num_lanes = rg.get_lanes().len();
    assert!(num_lanes > 0);

    // Query 2: linear tolerance
    let lin_tol = rg.linear_tolerance();
    assert!(lin_tol > 0.0);

    // Query 3: angular tolerance
    let ang_tol = rg.angular_tolerance();
    assert!(ang_tol > 0.0);
}

// ── CLI options (verified at the library level) ─────────────────────────────

#[test]
fn custom_linear_tolerance() {
    let xodr = xodr_path("TShapeRoad.xodr");
    let props = HashMap::from([
        ("road_geometry_id", "custom_tol_rg"),
        ("opendrive_file", xodr.as_str()),
        ("linear_tolerance", "0.5"),
    ]);
    let rn = RoadNetwork::new(RoadNetworkBackend::MaliputMalidrive, &props).expect("should load road network");
    let rg = rn.road_geometry();
    assert!(
        (rg.linear_tolerance() - 0.5).abs() < 1e-9,
        "Linear tolerance should be 0.5, got {}",
        rg.linear_tolerance()
    );
}

#[test]
fn allow_non_drivable_lanes_flag_is_applied() {
    // Default: non-drivable lanes are omitted.
    let rn_default = common::create_t_shape_road_network(true);
    let default_count = rn_default.road_geometry().get_lanes().len();

    // With non-drivable lanes included.
    let rn_all = common::create_t_shape_road_network(false);
    let all_count = rn_all.road_geometry().get_lanes().len();

    assert!(
        all_count > default_count,
        "Including non-drivable lanes ({}) should yield more lanes than the default ({})",
        all_count,
        default_count
    );
}

#[test]
fn sequential_build_policy_produces_same_road_network() {
    let xodr = xodr_path("TShapeRoad.xodr");

    // Parallel builder.
    let props_parallel = HashMap::from([
        ("road_geometry_id", "parallel_rg"),
        ("opendrive_file", xodr.as_str()),
        ("linear_tolerance", "0.001"),
        ("build_policy", "parallel"),
    ]);
    let rn_parallel =
        RoadNetwork::new(RoadNetworkBackend::MaliputMalidrive, &props_parallel).expect("parallel build should succeed");

    // Sequential builder.
    let props_sequential = HashMap::from([
        ("road_geometry_id", "sequential_rg"),
        ("opendrive_file", xodr.as_str()),
        ("linear_tolerance", "0.001"),
        ("build_policy", "sequential"),
    ]);
    let rn_sequential = RoadNetwork::new(RoadNetworkBackend::MaliputMalidrive, &props_sequential)
        .expect("sequential build should succeed");

    assert_eq!(
        rn_parallel.road_geometry().get_lanes().len(),
        rn_sequential.road_geometry().get_lanes().len(),
        "Parallel and sequential builders should produce the same number of lanes"
    );
}
