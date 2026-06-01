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

use maliput::api::objects::RoadMarkingType;

// These tests exercise the `RoadMarkingBook` / `RoadMarking` bindings end-to-end
// via the malidrive backend. They intentionally avoid asserting exact contents
// produced by the backend (which depends on the XODR-to-RoadMarking mapper) and
// instead validate that:
//   * `RoadNetwork::road_marking_book()` is reachable and returns a usable book,
//   * each accessor on the book and on individual `RoadMarking`s can be invoked
//     without panicking, and
//   * lookups for unknown ids / lanes / types return empty results.

#[test]
fn road_marking_book_accessor_is_reachable() {
    let road_network = common::create_malidrive_road_network("TwoRoadsWithRoadObjects.xodr", None, None);
    let book = road_network.road_marking_book();
    // road_markings() must return a Vec (length is backend-defined; we only
    // require the call succeed and the type be correct).
    let markings = book.road_markings();
    let _len: usize = markings.len();
}

#[test]
fn road_marking_book_lookup_unknown_returns_none_or_empty() {
    let road_network = common::create_malidrive_road_network("TwoRoadsWithRoadObjects.xodr", None, None);
    let book = road_network.road_marking_book();

    assert!(
        book.get_road_marking(&String::from("nonexistent_road_marking_id"))
            .is_none(),
        "Expected None for an unknown RoadMarking id",
    );
    assert!(
        book.find_by_lane(&String::from("nonexistent_lane_id")).is_empty(),
        "Expected an empty list for an unknown lane id",
    );
}

#[test]
fn road_marking_book_find_by_type_returns_consistent_results() {
    let road_network = common::create_malidrive_road_network("TwoRoadsWithRoadObjects.xodr", None, None);
    let book = road_network.road_marking_book();

    // Exercise every variant of RoadMarkingType so the enum conversion path is
    // covered. We do not require any particular variant to be populated by the
    // backend; we only require the call to succeed and return a Vec.
    for marking_type in [
        RoadMarkingType::Stop,
        RoadMarkingType::StopLine,
        RoadMarkingType::Crosswalk,
        RoadMarkingType::ParkingSpace,
        RoadMarkingType::EmergencyLane,
        RoadMarkingType::SpeedLimit,
        RoadMarkingType::DoNotStop,
        RoadMarkingType::RailRoad,
        RoadMarkingType::GiveWay,
        RoadMarkingType::ArrowTurnRight,
        RoadMarkingType::ArrowTurnLeft,
        RoadMarkingType::ArrowForwardTurnRight,
        RoadMarkingType::ArrowForwardTurnLeft,
        RoadMarkingType::ArrowForward,
        RoadMarkingType::ArrowForwardTurnRightTurnLeft,
        RoadMarkingType::ArrowTurnRightTurnLeft,
        RoadMarkingType::ArrowUTurnRight,
        RoadMarkingType::ArrowUTurnLeft,
        RoadMarkingType::Unknown,
    ] {
        let by_type = book.find_by_type(&marking_type);
        // Every reported marking must agree on its type.
        for marking in &by_type {
            assert_eq!(
                marking.marking_type(),
                marking_type,
                "find_by_type returned a marking whose marking_type() disagrees",
            );
        }
    }
}

#[test]
fn road_marking_accessors_dont_panic() {
    let road_network = common::create_malidrive_road_network("TwoRoadsWithRoadObjects.xodr", None, None);
    let book = road_network.road_marking_book();

    for marking in book.road_markings() {
        // Smoke-call every accessor on the wrapper. Values are not asserted
        // because they depend on the backend's XODR-to-RoadMarking mapping.
        let _id: String = marking.id();
        let _name: Option<String> = marking.name();
        let _t = marking.marking_type();
        let _pos = marking.position();
        let _rot = marking.orientation();
        let _bb = marking.bounding_box();
        let _related: Vec<String> = marking.related_lanes();
        let n = marking.num_outlines();
        let outlines = marking.outlines();
        assert_eq!(outlines.len() as i32, n, "num_outlines must match outlines().len()");
        let _v = marking.value();

        // Lookup by id must round-trip.
        let by_id = book.get_road_marking(&marking.id());
        assert!(
            by_id.is_some(),
            "RoadMarking present in road_markings() must be findable by id"
        );
        assert_eq!(by_id.unwrap().id(), marking.id());
    }
}
