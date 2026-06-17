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

// RoadWithAllDeviceTypes.xodr is used for all road marking tests. It is paired
// with the all_device_types_test_db.yaml traffic-control-device database which
// is required by maliput_malidrive's TrafficControlDeviceBooksBuilder to
// populate the RoadMarkingBook (see
// maliput_malidrive/builder/traffic_control_device_books_builder.cc).
//
// Road layout (viewed from above):
//
//             Road 1 (200m, hdg=0)
//   ============================================
//                    lane 1_0_1
//   ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─
//   (0,0) ───────────────────────────── (200,0)
//   ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─
//                    lane 1_0_-1
//   ============================================
//
// Objects on Road 1 (classified by all_device_types_test_db.yaml):
//   RM1         (Crosswalk1): type=crosswalk, s=120, t=0, zOffset=0
//                             → device_type=road_marking, semantics=crosswalk
//                             → kCrosswalk RoadMarking
//   RO1         (Barrier1):   type=barrier,   s=160 → device_type=road_object  (NOT a RoadMarking)
//   UNMATCHED1  (Tree1):      type=tree,      s=180 → unmatched               (NOT a RoadMarking)

const RM1_ID: &str = "RM1";

#[test]
fn road_marking_book_api() {
    let road_network = common::create_malidrive_road_network(
        "RoadWithAllDeviceTypes.xodr",
        None,
        Some("all_device_types_test_db.yaml"),
    );
    let book = road_network.road_marking_book();

    let markings = book.road_markings();
    assert_eq!(
        markings.len(),
        1,
        "Expected exactly 1 road marking, got {}",
        markings.len()
    );

    let ids: Vec<String> = markings.iter().map(|m| m.id()).collect();
    assert!(ids.contains(&RM1_ID.to_string()), "Missing road marking {RM1_ID}");

    // get_road_marking returns Some for the known id.
    assert!(book.get_road_marking(&RM1_ID.to_string()).is_some());

    // get_road_marking returns None for an unknown id.
    assert!(book.get_road_marking(&String::from("nonexistent_id")).is_none());
    // Objects routed to the RoadObjectBook (or unmatched) must not appear here.
    assert!(book.get_road_marking(&String::from("RO1")).is_none());
    assert!(book.get_road_marking(&String::from("UNMATCHED1")).is_none());
}

#[test]
fn road_marking_api() {
    let road_network = common::create_malidrive_road_network(
        "RoadWithAllDeviceTypes.xodr",
        None,
        Some("all_device_types_test_db.yaml"),
    );
    let book = road_network.road_marking_book();
    let tol = 1e-3;

    let rm1 = book.get_road_marking(&RM1_ID.to_string()).expect("RM1 not found");
    assert_eq!(rm1.id(), RM1_ID);
    assert_eq!(rm1.name(), Some("Crosswalk1".to_string()));
    assert_eq!(rm1.marking_type(), maliput::api::objects::RoadMarkingType::Crosswalk);

    // RM1: s=120 on road 1 (x=0..200, hdg=0) with t=0, zOffset=0 → inertial ≈ (120, 0, 0).
    let pos1 = rm1.position();
    assert!(
        (pos1.inertial_position.x() - 120.0).abs() < tol,
        "RM1 x: got {}",
        pos1.inertial_position.x()
    );
    assert!(
        (pos1.inertial_position.y() - 0.0).abs() < tol,
        "RM1 y: got {}",
        pos1.inertial_position.y()
    );
    assert!(
        (pos1.inertial_position.z() - 0.0).abs() < tol,
        "RM1 z: got {}",
        pos1.inertial_position.z()
    );

    // Bounding box has 8 vertices.
    assert_eq!(rm1.bounding_box().get_vertices().len(), 8);

    // num_outlines() must agree with outlines().len(). RM1 has no <outline> XML elements,
    // so we only require the two accessors to be consistent.
    let outlines = rm1.outlines();
    assert_eq!(outlines.len() as i32, rm1.num_outlines());

    // RM1 carries no <speed> child → no value attached.
    assert!(rm1.value().is_none());
}

#[test]
fn road_marking_related_lanes_test() {
    let road_network = common::create_malidrive_road_network(
        "RoadWithAllDeviceTypes.xodr",
        None,
        Some("all_device_types_test_db.yaml"),
    );
    let book = road_network.road_marking_book();

    let rm1 = book.get_road_marking(&RM1_ID.to_string()).expect("RM1 not found");
    let rm1_lanes = rm1.related_lanes();
    // RM1 has <validity fromLane="-1" toLane="1"/> on road 1 → covers lanes 1_0_-1 and 1_0_1
    // (lane 1_0_0 is the center reference lane and is non-drivable).
    assert_eq!(rm1_lanes.len(), 2, "RM1 expected 2 related lanes, got {:?}", rm1_lanes);
    assert!(rm1_lanes.contains(&"1_0_-1".to_string()));
    assert!(rm1_lanes.contains(&"1_0_1".to_string()));
}

#[test]
fn road_marking_book_find_by_lane_test() {
    let road_network = common::create_malidrive_road_network(
        "RoadWithAllDeviceTypes.xodr",
        None,
        Some("all_device_types_test_db.yaml"),
    );
    let book = road_network.road_marking_book();

    // Lane "1_0_-1" is related to RM1 via its validity range.
    let markings_minus1 = book.find_by_lane(&String::from("1_0_-1"));
    assert_eq!(markings_minus1.len(), 1);
    assert_eq!(markings_minus1[0].id(), RM1_ID);

    // Lane "1_0_1" is also related to RM1 via the same validity range.
    let markings_plus1 = book.find_by_lane(&String::from("1_0_1"));
    assert_eq!(markings_plus1.len(), 1);
    assert_eq!(markings_plus1[0].id(), RM1_ID);

    // A nonexistent lane returns an empty vector.
    let markings_none = book.find_by_lane(&String::from("nonexistent_lane"));
    assert!(markings_none.is_empty());
}

#[test]
fn road_marking_book_find_by_type_test() {
    let road_network = common::create_malidrive_road_network(
        "RoadWithAllDeviceTypes.xodr",
        None,
        Some("all_device_types_test_db.yaml"),
    );
    let book = road_network.road_marking_book();

    // RM1 is a crosswalk → find_by_type(Crosswalk) returns exactly RM1.
    let crosswalks = book.find_by_type(&maliput::api::objects::RoadMarkingType::Crosswalk);
    assert_eq!(crosswalks.len(), 1);
    assert_eq!(crosswalks[0].id(), RM1_ID);

    // No stop lines, arrows, or other markings in this map.
    let stop_lines = book.find_by_type(&maliput::api::objects::RoadMarkingType::StopLine);
    assert!(stop_lines.is_empty());
    let arrows = book.find_by_type(&maliput::api::objects::RoadMarkingType::PrescribedStraight);
    assert!(arrows.is_empty());
    let unknown = book.find_by_type(&maliput::api::objects::RoadMarkingType::Unknown);
    assert!(unknown.is_empty());
}

#[test]
fn road_marking_book_is_empty_without_tcd_database() {
    // Without a traffic_control_device_db, the malidrive backend cannot classify
    // any XODR object as a RoadMarking, so the book must be empty.
    let road_network = common::create_malidrive_road_network("RoadWithAllDeviceTypes.xodr", None, None);
    let book = road_network.road_marking_book();
    assert!(book.road_markings().is_empty());
}
