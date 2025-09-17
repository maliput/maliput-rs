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

use maliput::api::rules::RuleType;

#[test]
fn rule_type_api() {
    let rule_type = RuleType::DirectionUsage;
    assert_eq!(rule_type.to_string(), "Direction-Usage Rule Type");
    assert_eq!(rule_type.get_rule_id("lane_id"), "Direction-Usage Rule Type/lane_id");
    let rule_type = RuleType::RightOfWay;
    assert_ne!(rule_type.to_string(), "Direction-Usage Rule Type");
    assert_eq!(rule_type.to_string(), "Right-Of-Way Rule Type");
    assert_eq!(rule_type.get_rule_id("lane_id"), "Right-Of-Way Rule Type/lane_id");
    let rule_type = RuleType::VehicleStopInZoneBehavior;
    assert_ne!(rule_type.to_string(), "Right-Of-Way Rule Type");
    assert_eq!(rule_type.to_string(), "Vehicle-Stop-In-Zone-Behavior Rule Type");
    assert_eq!(
        rule_type.get_rule_id("lane_id"),
        "Vehicle-Stop-In-Zone-Behavior Rule Type/lane_id"
    );
    let rule_type = RuleType::SpeedLimit;
    assert_ne!(rule_type.to_string(), "Vehicle-Stop-In-Zone-Behavior Rule Type");
    assert_eq!(rule_type.to_string(), "Speed-Limit Rule Type");
    assert_eq!(rule_type.get_rule_id("lane_id"), "Speed-Limit Rule Type/lane_id");
    let rule_type = RuleType::DirectionUsage;
    assert_ne!(rule_type.to_string(), "Speed-Limit Rule Type");
}
