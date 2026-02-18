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

//! Integration tests for the `maliput_query` binary.
//!
//! These tests run the compiled binary as a subprocess and verify its
//! stdout/stderr output and exit code for various scenarios.

use assert_cmd::Command;
use predicates::prelude::*;

/// Helper: get the path to a test xodr file shipped with the crate.
fn xodr_path(name: &str) -> String {
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    format!("{}/data/xodr/{}", manifest_dir, name)
}

/// Helper: build a `Command` for the `maliput_query` binary.
#[allow(deprecated)]
fn maliput_query_cmd() -> Command {
    Command::cargo_bin("maliput_query").expect("binary maliput_query should be built")
}

// ── CLI argument validation ─────────────────────────────────────────────────

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
        .stdout(predicate::str::contains("ROAD_NETWORK_XODR_FILE_PATH"));
}

#[test]
fn version_flag() {
    maliput_query_cmd()
        .arg("--version")
        .assert()
        .success()
        .stdout(predicate::str::is_empty().not());
}

// ── Road network loading ────────────────────────────────────────────────────

#[test]
fn loads_road_network_and_responds_to_exit() {
    let xodr = xodr_path("TShapeRoad.xodr");
    maliput_query_cmd()
        .arg(&xodr)
        .write_stdin("exit\n")
        .assert()
        .success()
        .stdout(predicate::str::contains("Road network loaded"))
        .stdout(predicate::str::contains("Available commands"));
}

#[test]
fn invalid_file_path_fails() {
    maliput_query_cmd()
        .arg("/nonexistent/path/fake.xodr")
        .write_stdin("exit\n")
        .assert()
        .failure();
}

// ── Query commands via stdin ────────────────────────────────────────────────

#[test]
fn get_number_of_lanes() {
    let xodr = xodr_path("TShapeRoad.xodr");
    maliput_query_cmd()
        .arg(&xodr)
        .write_stdin("GetNumberOfLanes\nexit\n")
        .assert()
        .success()
        .stdout(predicate::str::contains("Number of lanes:"));
}

#[test]
fn get_linear_tolerance() {
    let xodr = xodr_path("TShapeRoad.xodr");
    maliput_query_cmd()
        .arg(&xodr)
        .write_stdin("GetLinearTolerance\nexit\n")
        .assert()
        .success()
        .stdout(predicate::str::contains("Linear tolerance"));
}

#[test]
fn get_angular_tolerance() {
    let xodr = xodr_path("TShapeRoad.xodr");
    maliput_query_cmd()
        .arg(&xodr)
        .write_stdin("GetAngularTolerance\nexit\n")
        .assert()
        .success()
        .stdout(predicate::str::contains("Angular tolerance"));
}

#[test]
fn get_total_length() {
    let xodr = xodr_path("TShapeRoad.xodr");
    maliput_query_cmd()
        .arg(&xodr)
        .write_stdin("GetTotalLengthOfTheRoadGeometry\nexit\n")
        .assert()
        .success()
        .stdout(predicate::str::contains("Total length of the road geometry"));
}

#[test]
fn print_all_lanes() {
    let xodr = xodr_path("TShapeRoad.xodr");
    maliput_query_cmd()
        .arg(&xodr)
        .write_stdin("PrintAllLanes\nexit\n")
        .assert()
        .success()
        .stdout(predicate::str::contains("All lanes in the road geometry"));
}

#[test]
fn to_road_position() {
    let xodr = xodr_path("TShapeRoad.xodr");
    maliput_query_cmd()
        .arg(&xodr)
        .write_stdin("ToRoadPosition 0 0 0\nexit\n")
        .assert()
        .success()
        .stdout(predicate::str::contains("Road Position Result"));
}

#[test]
fn invalid_query_shows_available_commands() {
    let xodr = xodr_path("TShapeRoad.xodr");
    maliput_query_cmd()
        .arg(&xodr)
        .write_stdin("FooBarBaz\nexit\n")
        .assert()
        .success()
        .stdout(predicate::str::contains("Invalid query command"))
        .stdout(predicate::str::contains("Available commands"));
}

// ── CLI options ─────────────────────────────────────────────────────────────

#[test]
fn custom_linear_tolerance() {
    let xodr = xodr_path("TShapeRoad.xodr");
    maliput_query_cmd()
        .args([&xodr, "--linear-tolerance", "0.5"])
        .write_stdin("GetLinearTolerance\nexit\n")
        .assert()
        .success()
        .stdout(predicate::str::contains("0.5"));
}

#[test]
fn multiple_queries_in_sequence() {
    let xodr = xodr_path("TShapeRoad.xodr");
    maliput_query_cmd()
        .arg(&xodr)
        .write_stdin("GetNumberOfLanes\nGetLinearTolerance\nGetAngularTolerance\nexit\n")
        .assert()
        .success()
        .stdout(predicate::str::contains("Number of lanes"))
        .stdout(predicate::str::contains("Linear tolerance"))
        .stdout(predicate::str::contains("Angular tolerance"));
}

#[test]
fn allow_non_drivable_lanes_flag_is_applied() {
    let xodr = xodr_path("TShapeRoad.xodr");

    // Default: non-drivable lanes are omitted (omit_nondrivable_lanes = "true").
    let default_output = maliput_query_cmd()
        .arg(&xodr)
        .write_stdin("GetNumberOfLanes\nexit\n")
        .assert()
        .success()
        .stdout(predicate::str::contains("\"omit_nondrivable_lanes\": \"true\""))
        .get_output()
        .stdout
        .clone();
    let default_stdout = String::from_utf8(default_output).unwrap();

    // With --allow-non-drivable-lanes: omit_nondrivable_lanes = "false", so more lanes are included.
    let with_flag_output = maliput_query_cmd()
        .args([&xodr, "--allow-non-drivable-lanes"])
        .write_stdin("GetNumberOfLanes\nexit\n")
        .assert()
        .success()
        .stdout(predicate::str::contains("\"omit_nondrivable_lanes\": \"false\""))
        .get_output()
        .stdout
        .clone();
    let with_flag_stdout = String::from_utf8(with_flag_output).unwrap();

    // Extract the lane count from each run.
    let re = regex::Regex::new(r"Number of lanes:\s*(\d+)").unwrap();
    let default_count: u32 = re
        .captures(&default_stdout)
        .expect("should find lane count in default output")[1]
        .parse()
        .unwrap();
    let with_flag_count: u32 = re
        .captures(&with_flag_stdout)
        .expect("should find lane count in --allow-non-drivable-lanes output")[1]
        .parse()
        .unwrap();

    assert!(
        with_flag_count > default_count,
        "With --allow-non-drivable-lanes the lane count ({}) should be greater than the default ({})",
        with_flag_count,
        default_count
    );
}

#[test]
fn disable_parallel_builder_policy_flag_is_applied() {
    let xodr = xodr_path("TShapeRoad.xodr");

    // Default: parallel build policy.
    maliput_query_cmd()
        .arg(&xodr)
        .write_stdin("exit\n")
        .assert()
        .success()
        .stdout(predicate::str::contains("\"build_policy\": \"parallel\""));

    // With --disable-parallel-builder-policy: sequential build policy.
    maliput_query_cmd()
        .args([&xodr, "--disable-parallel-builder-policy"])
        .write_stdin("exit\n")
        .assert()
        .success()
        .stdout(predicate::str::contains("\"build_policy\": \"sequential\""));
}
