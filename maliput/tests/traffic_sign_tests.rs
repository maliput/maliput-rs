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
mod common;

// TwoRoadsWithTrafficSigns.xodr is used for all traffic sign tests.
//
// Road layout (viewed from above):
//
//     Road 1 (100m, hdg=0)          Road 2 (80m, hdg=0)
//   ========================      ========================
//        lane 1_0_1                    lane 2_0_1
//   ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─    ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─
//   (0,0) ──────── (100,0)       (110,0) ──────── (190,0)
//   ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─    ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─
//        lane 1_0_-1                   lane 2_0_-1
//   ========================      ========================
//       SS1 at s=50                   SS2 at s=40
//
// SS1 (name="StopSign_SS1"): type=206/Stop, s=50, t=2, zOffset=1, validity fromLane=-1 toLane=-1.
//   Road 1 also carries a signalReference to SS2 (no validity → all lanes of road 1).
//   related_lanes(SS1): {1_0_-1, 2_0_-1, 2_0_1}
//
// SS2 (name="StopSign_SS2"): type=206/Stop, s=40, t=-2, zOffset=1, validity: empty (all lanes of road 2).
//   Road 2 also carries a signalReference to SS1 (no validity → all lanes of road 2).
//   related_lanes(SS2): {1_0_-1, 1_0_1, 2_0_-1, 2_0_1}

const SS1_ID: &str = "SS1";
const SS2_ID: &str = "SS2";

#[test]
fn traffic_sign_book_api() {
    let road_network = common::create_malidrive_road_network(
        "TwoRoadsWithTrafficSigns.xodr",
        None,
        Some("traffic_signal_db_example.yaml"),
    );
    let book = road_network.traffic_sign_book();

    let signs = book.traffic_signs();
    assert_eq!(signs.len(), 2, "Expected 2 traffic signs");

    let ids: Vec<String> = signs.iter().map(|s| s.id()).collect();
    assert!(ids.contains(&SS1_ID.to_string()), "Missing sign {SS1_ID}");
    assert!(ids.contains(&SS2_ID.to_string()), "Missing sign {SS2_ID}");

    // get_traffic_sign returns Some for known IDs.
    assert!(book.get_traffic_sign(&SS1_ID.to_string()).is_some());
    assert!(book.get_traffic_sign(&SS2_ID.to_string()).is_some());

    // get_traffic_sign returns None for an unknown ID.
    assert!(book.get_traffic_sign(&String::from("nonexistent_id")).is_none());
}

#[test]
fn traffic_sign_test_api() {
    let road_network = common::create_malidrive_road_network(
        "TwoRoadsWithTrafficSigns.xodr",
        None,
        Some("traffic_signal_db_example.yaml"),
    );
    let book = road_network.traffic_sign_book();
    let tol = 1e-1;

    let ss1 = book.get_traffic_sign(&SS1_ID.to_string()).expect("SS1 not found");
    assert_eq!(ss1.id(), SS1_ID);
    assert_eq!(ss1.sign_type(), maliput::api::rules::TrafficSignType::Stop);
    // SS1: s=50 on road 1 (x=0..100, hdg=0) with t=2, zOffset=1 → position ≈ (50, 2, 1).
    let pos1 = ss1.position_road_network();
    assert!((pos1.x() - 50.0).abs() < tol, "SS1 x: got {}", pos1.x());
    assert!((pos1.y() - 2.0).abs() < tol, "SS1 y: got {}", pos1.y());
    assert!((pos1.z() - 1.0).abs() < tol, "SS1 z: got {}", pos1.z());
    // Bounding box has 8 vertices.
    assert_eq!(ss1.bounding_box().get_vertices().len(), 8);

    let ss2 = book.get_traffic_sign(&SS2_ID.to_string()).expect("SS2 not found");
    assert_eq!(ss2.id(), SS2_ID);
    assert_eq!(ss2.sign_type(), maliput::api::rules::TrafficSignType::Stop);
    // SS2: s=40 on road 2 (x=110..190, hdg=0) with t=-2, zOffset=1 → position ≈ (150, -2, 1).
    let pos2 = ss2.position_road_network();
    assert!((pos2.x() - 150.0).abs() < tol, "SS2 x: got {}", pos2.x());
    assert!((pos2.y() - (-2.0)).abs() < tol, "SS2 y: got {}", pos2.y());
    assert!((pos2.z() - 1.0).abs() < tol, "SS2 z: got {}", pos2.z());
    assert_eq!(ss2.bounding_box().get_vertices().len(), 8);
}

#[test]
fn traffic_sign_related_lanes_test() {
    let road_network = common::create_malidrive_road_network(
        "TwoRoadsWithTrafficSigns.xodr",
        None,
        Some("traffic_signal_db_example.yaml"),
    );
    let book = road_network.traffic_sign_book();

    let ss1 = book.get_traffic_sign(&SS1_ID.to_string()).expect("SS1 not found");
    let ss1_lanes = ss1.related_lanes();
    // SS1 validity covers lane 1_0_-1; signalRef on road 2 covers 2_0_1 and 2_0_-1.
    assert_eq!(ss1_lanes.len(), 3, "SS1 expected 3 related lanes, got {:?}", ss1_lanes);
    assert!(ss1_lanes.contains(&"1_0_-1".to_string()));
    assert!(ss1_lanes.contains(&"2_0_1".to_string()));
    assert!(ss1_lanes.contains(&"2_0_-1".to_string()));

    let ss2 = book.get_traffic_sign(&SS2_ID.to_string()).expect("SS2 not found");
    let ss2_lanes = ss2.related_lanes();
    // SS2 validity covers all lanes of road 2 (2_0_1, 2_0_-1); signalRef on road 1 covers 1_0_1 and 1_0_-1.
    assert_eq!(ss2_lanes.len(), 4, "SS2 expected 4 related lanes, got {:?}", ss2_lanes);
    assert!(ss2_lanes.contains(&"2_0_1".to_string()));
    assert!(ss2_lanes.contains(&"2_0_-1".to_string()));
    assert!(ss2_lanes.contains(&"1_0_1".to_string()));
    assert!(ss2_lanes.contains(&"1_0_-1".to_string()));
}

#[test]
fn traffic_sign_book_find_by_lane_test() {
    let road_network = common::create_malidrive_road_network(
        "TwoRoadsWithTrafficSigns.xodr",
        None,
        Some("traffic_signal_db_example.yaml"),
    );
    let book = road_network.traffic_sign_book();

    // Lane "1_0_-1" is related to both SS1 (own validity) and SS2 (via signalRef).
    let signs_1_0_minus1 = book.find_by_lane(&String::from("1_0_-1"));
    assert_eq!(signs_1_0_minus1.len(), 2);
    let ids: Vec<String> = signs_1_0_minus1.iter().map(|s| s.id()).collect();
    assert!(ids.contains(&SS1_ID.to_string()));
    assert!(ids.contains(&SS2_ID.to_string()));

    // Lane "1_0_1" is related only to SS2 (via signalRef on road 1, no validity → all lanes).
    let signs_1_0_1 = book.find_by_lane(&String::from("1_0_1"));
    assert_eq!(signs_1_0_1.len(), 1);
    assert_eq!(signs_1_0_1[0].id(), SS2_ID);

    // A nonexistent lane returns an empty vector.
    let signs_none = book.find_by_lane(&String::from("nonexistent_lane"));
    assert!(signs_none.is_empty());
}

#[test]
fn traffic_sign_book_find_by_type_test() {
    let road_network = common::create_malidrive_road_network(
        "TwoRoadsWithTrafficSigns.xodr",
        None,
        Some("traffic_signal_db_example.yaml"),
    );
    let book = road_network.traffic_sign_book();

    // Both signs are Stop signs.
    let stop_signs = book.find_by_type(&maliput::api::rules::TrafficSignType::Stop);
    assert_eq!(stop_signs.len(), 2);
    let ids: Vec<String> = stop_signs.iter().map(|s| s.id()).collect();
    assert!(ids.contains(&SS1_ID.to_string()));
    assert!(ids.contains(&SS2_ID.to_string()));

    // No Yield signs in this map.
    let yield_signs = book.find_by_type(&maliput::api::rules::TrafficSignType::Yield);
    assert!(yield_signs.is_empty());
}
