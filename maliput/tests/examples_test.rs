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

//! Integration tests that verify every example compiles and runs successfully.
//!
//! Each test runs `cargo run --example <name>` as a subprocess and asserts
//! that the process exits with code 0.  This catches compilation errors,
//! runtime panics, and assertion failures inside the examples.

use std::process::Command;

/// Run a single example by name and assert it succeeds.
fn run_example(name: &str) {
    let output = Command::new(env!("CARGO"))
        .args(["run", "--example", name])
        .output()
        .unwrap_or_else(|e| panic!("Failed to launch `cargo run --example {name}`: {e}"));

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    assert!(
        output.status.success(),
        "Example `{name}` failed (exit code: {:?}).\n--- stdout ---\n{stdout}\n--- stderr ---\n{stderr}",
        output.status.code()
    );
}

#[test]
fn example_01_road_geometry() {
    run_example("01_road_geometry");
}

#[test]
fn example_02_backend_custom_command() {
    run_example("02_backend_custom_command");
}

#[test]
fn example_03_maliput_malidrive_resources() {
    run_example("03_maliput_malidrive_resources");
}

#[test]
fn example_04_math() {
    run_example("04_math");
}

#[test]
fn example_05_multi_road_geometry() {
    run_example("05_multi_road_geometry");
}

#[test]
fn example_06_road_rulebook() {
    run_example("06_road_rulebook");
}

#[test]
fn example_07_error_handling() {
    run_example("07_error_handling");
}

#[test]
fn example_08_direction_usage_rule() {
    run_example("08_direction_usage_rule");
}

#[test]
fn example_09_phase_ring() {
    run_example("09_phase_ring");
}

#[test]
fn example_10_lane_boundary() {
    run_example("10_lane_boundary");
}

#[test]
fn example_11_lane_boundary_complex_marking() {
    run_example("11_lane_boundary_complex_marking");
}
