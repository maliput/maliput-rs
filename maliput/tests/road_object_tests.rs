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

// TwoRoadsWithRoadObjects.xodr road layout (viewed from above):
//
//     Road 1 (100m, hdg=0)              Road 2 (100m, hdg=0)
//   ============================      ============================
//           lane 1_0_1                        lane 2_0_1
//   ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─    ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─
//   (0,0) ──────────── (100,0)       (100,0) ──────────── (200,0)
//   ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─    ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─
//           lane 1_0_-1                       lane 2_0_-1
//   ============================      ============================
//
// Objects on Road 1:
//   obj_vegetation  (StarBush)        : type=vegetation, s=0,   t=4,  zOffset=0.0, star outline (6 corners, closed)
//   obj_barrier     (GuardRail)       : type=barrier,    s=20,  t=3,  zOffset=0.5, validity fromLane=-1 toLane=-1
//   obj_building    (Warehouse)       : type=building,   s=50,  t=-5, zOffset=0.0, radius=4.0
//   obj_crosswalk   (PedestrianCrossing): type=crosswalk, s=100, t=0, zOffset=0.0, validity fromLane=-1 toLane=1, rect outline (4 corners, closed)
//
// RoadWithStopLine.xodr road layout:
//
//     Road 1 (100m, hdg=0)
//   ============================
//           lane 1_0_1
//   ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─
//   (0,0) ──────────── (100,0)
//   ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─
//           lane 1_0_-1
//   ============================
//
// Objects on Road 1:
//   obj_stop_line_name   : type=roadMark, name=stopLine   → kStopLine
//   obj_stop_line_subtype: type=roadMark, subtype=stopLine → kStopLine
//   obj_road_mark        : type=roadMark, name=TurnArrow  → kRoadMark

const OBJ_VEGETATION_ID: &str = "obj_vegetation";
const OBJ_BARRIER_ID: &str = "obj_barrier";
const OBJ_BUILDING_ID: &str = "obj_building";
const OBJ_CROSSWALK_ID: &str = "obj_crosswalk";
const OBJ_STOP_LINE_NAME_ID: &str = "obj_stop_line_name";
const OBJ_STOP_LINE_SUBTYPE_ID: &str = "obj_stop_line_subtype";
const OBJ_ROAD_MARK_ID: &str = "obj_road_mark";

#[test]
fn road_object_book_api() {
    let road_network = common::create_malidrive_road_network("TwoRoadsWithRoadObjects.xodr", None, None);
    let book = road_network.road_object_book();

    let objects = book.road_objects();
    assert_eq!(objects.len(), 4, "Expected 4 road objects");

    let ids: Vec<String> = objects.iter().map(|o| o.id()).collect();
    assert!(
        ids.contains(&OBJ_VEGETATION_ID.to_string()),
        "Missing {OBJ_VEGETATION_ID}"
    );
    assert!(ids.contains(&OBJ_BARRIER_ID.to_string()), "Missing {OBJ_BARRIER_ID}");
    assert!(ids.contains(&OBJ_BUILDING_ID.to_string()), "Missing {OBJ_BUILDING_ID}");
    assert!(
        ids.contains(&OBJ_CROSSWALK_ID.to_string()),
        "Missing {OBJ_CROSSWALK_ID}"
    );

    // get_road_object returns Some for known IDs.
    assert!(book.get_road_object(&OBJ_VEGETATION_ID.to_string()).is_some());
    assert!(book.get_road_object(&OBJ_BARRIER_ID.to_string()).is_some());
    assert!(book.get_road_object(&OBJ_BUILDING_ID.to_string()).is_some());
    assert!(book.get_road_object(&OBJ_CROSSWALK_ID.to_string()).is_some());

    // get_road_object returns None for an unknown ID.
    assert!(book.get_road_object(&String::from("nonexistent_id")).is_none());
}

#[test]
fn road_object_api() {
    let road_network = common::create_malidrive_road_network("TwoRoadsWithRoadObjects.xodr", None, None);
    let book = road_network.road_object_book();

    let vegetation = book
        .get_road_object(&OBJ_VEGETATION_ID.to_string())
        .expect("obj_vegetation not found");
    assert_eq!(vegetation.id(), OBJ_VEGETATION_ID);
    assert_eq!(vegetation.name(), Some("StarBush".to_string()));
    assert_eq!(
        vegetation.object_type(),
        maliput::api::objects::RoadObjectType::Vegetation
    );
    assert_eq!(vegetation.subtype(), None);
    assert!(!vegetation.is_dynamic());

    let barrier = book
        .get_road_object(&OBJ_BARRIER_ID.to_string())
        .expect("obj_barrier not found");
    assert_eq!(barrier.id(), OBJ_BARRIER_ID);
    assert_eq!(barrier.name(), Some("GuardRail".to_string()));
    assert_eq!(barrier.object_type(), maliput::api::objects::RoadObjectType::Barrier);
    assert_eq!(barrier.subtype(), Some("guardRail".to_string()));
    assert!(!barrier.is_dynamic());

    let building = book
        .get_road_object(&OBJ_BUILDING_ID.to_string())
        .expect("obj_building not found");
    assert_eq!(building.id(), OBJ_BUILDING_ID);
    assert_eq!(building.name(), Some("Warehouse".to_string()));
    assert_eq!(building.object_type(), maliput::api::objects::RoadObjectType::Building);
    assert_eq!(building.subtype(), None);
    assert!(!building.is_dynamic());

    let crosswalk = book
        .get_road_object(&OBJ_CROSSWALK_ID.to_string())
        .expect("obj_crosswalk not found");
    assert_eq!(crosswalk.id(), OBJ_CROSSWALK_ID);
    assert_eq!(crosswalk.name(), Some("PedestrianCrossing".to_string()));
    assert_eq!(
        crosswalk.object_type(),
        maliput::api::objects::RoadObjectType::Crosswalk
    );
    assert_eq!(crosswalk.subtype(), None);
    assert!(!crosswalk.is_dynamic());
}

#[test]
fn road_object_position() {
    let road_network = common::create_malidrive_road_network("TwoRoadsWithRoadObjects.xodr", None, None);
    let book = road_network.road_object_book();
    let tol = 1e-2; // Equal to the linear_tolerance set in [common::create_malidrive_road_network].

    // obj_vegetation: s=0, t=4, zOffset=0 on hdg=0 road → inertial ≈ (0, 4, 0).
    let veg = book.get_road_object(&OBJ_VEGETATION_ID.to_string()).unwrap();
    let veg_pos = veg.position();
    assert!(
        (veg_pos.inertial_position.x() - 0.0).abs() < tol,
        "veg x: got {}",
        veg_pos.inertial_position.x()
    );
    assert!(
        (veg_pos.inertial_position.y() - 4.0).abs() < tol,
        "veg y: got {}",
        veg_pos.inertial_position.y()
    );
    assert!(
        (veg_pos.inertial_position.z() - 0.0).abs() < tol,
        "veg z: got {}",
        veg_pos.inertial_position.z()
    );

    // obj_barrier: s=20, t=3, zOffset=0.5 → inertial ≈ (20, 3, 0.5).
    let barrier = book.get_road_object(&OBJ_BARRIER_ID.to_string()).unwrap();
    let barrier_pos = barrier.position();
    assert!(
        (barrier_pos.inertial_position.x() - 20.0).abs() < tol,
        "barrier x: got {}",
        barrier_pos.inertial_position.x()
    );
    assert!(
        (barrier_pos.inertial_position.y() - 3.0).abs() < tol,
        "barrier y: got {}",
        barrier_pos.inertial_position.y()
    );
    assert!(
        (barrier_pos.inertial_position.z() - 0.5).abs() < tol,
        "barrier z: got {}",
        barrier_pos.inertial_position.z()
    );

    // obj_building: s=50, t=-5, zOffset=0 → inertial ≈ (50, -5, 0).
    let building = book.get_road_object(&OBJ_BUILDING_ID.to_string()).unwrap();
    let building_pos = building.position();
    assert!(
        (building_pos.inertial_position.x() - 50.0).abs() < tol,
        "building x: got {}",
        building_pos.inertial_position.x()
    );
    assert!(
        (building_pos.inertial_position.y() - (-5.0)).abs() < tol,
        "building y: got {}",
        building_pos.inertial_position.y()
    );
    assert!(
        (building_pos.inertial_position.z() - 0.0).abs() < tol,
        "building z: got {}",
        building_pos.inertial_position.z()
    );
}

#[test]
fn road_object_bounding_box() {
    let road_network = common::create_malidrive_road_network("TwoRoadsWithRoadObjects.xodr", None, None);
    let book = road_network.road_object_book();

    // Every RoadObject should have a valid bounding box with 8 vertices.
    let barrier = book.get_road_object(&OBJ_BARRIER_ID.to_string()).unwrap();
    assert_eq!(barrier.bounding_box().get_vertices().len(), 8);

    let building = book.get_road_object(&OBJ_BUILDING_ID.to_string()).unwrap();
    assert_eq!(building.bounding_box().get_vertices().len(), 8);

    let crosswalk = book.get_road_object(&OBJ_CROSSWALK_ID.to_string()).unwrap();
    assert_eq!(crosswalk.bounding_box().get_vertices().len(), 8);
}

#[test]
fn road_object_outlines() {
    let road_network = common::create_malidrive_road_network("TwoRoadsWithRoadObjects.xodr", None, None);
    let book = road_network.road_object_book();

    // obj_vegetation has 1 outline with 6 corners (star shape, closed).
    let veg = book.get_road_object(&OBJ_VEGETATION_ID.to_string()).unwrap();
    assert_eq!(veg.num_outlines(), 1);
    let veg_outlines = veg.outlines();
    assert_eq!(veg_outlines.len(), 1);
    assert!(veg_outlines[0].is_closed());
    assert_eq!(veg_outlines[0].num_corners(), 6);
    assert_eq!(veg_outlines[0].corners().len(), 6);
    // All corners have a height attribute (height="2.0" in the XODR cornerLocal).
    assert!(
        veg_outlines[0].corners().iter().all(|c| c.height.is_some()),
        "All vegetation corners should have height"
    );

    // obj_crosswalk has 1 outline with 4 corners (closed).
    let crosswalk = book.get_road_object(&OBJ_CROSSWALK_ID.to_string()).unwrap();
    assert_eq!(crosswalk.num_outlines(), 1);
    let crosswalk_outlines = crosswalk.outlines();
    assert_eq!(crosswalk_outlines.len(), 1);
    assert!(crosswalk_outlines[0].is_closed());
    assert_eq!(crosswalk_outlines[0].num_corners(), 4);
    assert_eq!(crosswalk_outlines[0].corners().len(), 4);

    // obj_barrier has no outlines.
    let barrier = book.get_road_object(&OBJ_BARRIER_ID.to_string()).unwrap();
    assert_eq!(barrier.num_outlines(), 0);
    assert!(barrier.outlines().is_empty());

    // obj_building has no outlines.
    let building = book.get_road_object(&OBJ_BUILDING_ID.to_string()).unwrap();
    assert_eq!(building.num_outlines(), 0);
    assert!(building.outlines().is_empty());
}

#[test]
fn road_object_related_lanes() {
    let road_network = common::create_malidrive_road_network("TwoRoadsWithRoadObjects.xodr", None, None);
    let book = road_network.road_object_book();

    // obj_barrier: validity fromLane=-1 toLane=-1 → related to lane "1_0_-1" only.
    let barrier = book.get_road_object(&OBJ_BARRIER_ID.to_string()).unwrap();
    let barrier_lanes = barrier.related_lanes();
    assert!(
        barrier_lanes.contains(&"1_0_-1".to_string()),
        "obj_barrier related lanes: {:?}",
        barrier_lanes
    );
    assert!(
        !barrier_lanes.contains(&"1_0_1".to_string()),
        "obj_barrier should not be related to 1_0_1: {:?}",
        barrier_lanes
    );

    // obj_crosswalk: validity fromLane=-1 toLane=1 → related to lanes "1_0_-1" and "1_0_1".
    let crosswalk = book.get_road_object(&OBJ_CROSSWALK_ID.to_string()).unwrap();
    let crosswalk_lanes = crosswalk.related_lanes();
    assert!(
        crosswalk_lanes.contains(&"1_0_-1".to_string()),
        "obj_crosswalk related lanes: {:?}",
        crosswalk_lanes
    );
    assert!(
        crosswalk_lanes.contains(&"1_0_1".to_string()),
        "obj_crosswalk related lanes: {:?}",
        crosswalk_lanes
    );
}

#[test]
fn road_object_book_find_by_type() {
    let road_network = common::create_malidrive_road_network("TwoRoadsWithRoadObjects.xodr", None, None);
    let book = road_network.road_object_book();

    let vegetation_objects = book.find_by_type(&maliput::api::objects::RoadObjectType::Vegetation);
    assert_eq!(vegetation_objects.len(), 1);
    assert_eq!(vegetation_objects[0].id(), OBJ_VEGETATION_ID);

    let barrier_objects = book.find_by_type(&maliput::api::objects::RoadObjectType::Barrier);
    assert_eq!(barrier_objects.len(), 1);
    assert_eq!(barrier_objects[0].id(), OBJ_BARRIER_ID);

    let building_objects = book.find_by_type(&maliput::api::objects::RoadObjectType::Building);
    assert_eq!(building_objects.len(), 1);
    assert_eq!(building_objects[0].id(), OBJ_BUILDING_ID);

    let crosswalk_objects = book.find_by_type(&maliput::api::objects::RoadObjectType::Crosswalk);
    assert_eq!(crosswalk_objects.len(), 1);
    assert_eq!(crosswalk_objects[0].id(), OBJ_CROSSWALK_ID);

    // No Pole objects in this map.
    let pole_objects = book.find_by_type(&maliput::api::objects::RoadObjectType::Pole);
    assert!(
        pole_objects.is_empty(),
        "Expected no Pole objects, got {:?}",
        pole_objects.len()
    );
}

#[test]
fn road_object_book_find_by_lane() {
    let road_network = common::create_malidrive_road_network("TwoRoadsWithRoadObjects.xodr", None, None);
    let book = road_network.road_object_book();

    // Lane "1_0_-1": obj_barrier (validity -1 to -1) and obj_crosswalk (validity -1 to 1).
    let objects_1_0_minus1 = book.find_by_lane(&String::from("1_0_-1"));
    let ids: Vec<String> = objects_1_0_minus1.iter().map(|o| o.id()).collect();
    assert!(
        ids.contains(&OBJ_BARRIER_ID.to_string()),
        "Expected obj_barrier in lane 1_0_-1, got: {:?}",
        ids
    );
    assert!(
        ids.contains(&OBJ_CROSSWALK_ID.to_string()),
        "Expected obj_crosswalk in lane 1_0_-1, got: {:?}",
        ids
    );

    // Lane "1_0_1": obj_crosswalk only (validity -1 to 1 includes +1, barrier is -1 only).
    let objects_1_0_1 = book.find_by_lane(&String::from("1_0_1"));
    let ids: Vec<String> = objects_1_0_1.iter().map(|o| o.id()).collect();
    assert!(
        ids.contains(&OBJ_CROSSWALK_ID.to_string()),
        "Expected obj_crosswalk in lane 1_0_1, got: {:?}",
        ids
    );
    assert!(
        !ids.contains(&OBJ_BARRIER_ID.to_string()),
        "obj_barrier should not be in lane 1_0_1, got: {:?}",
        ids
    );

    // Unknown lane returns empty.
    let none_objects = book.find_by_lane(&String::from("nonexistent_lane"));
    assert!(none_objects.is_empty());
}

#[test]
fn road_object_book_find_in_radius() {
    let road_network = common::create_malidrive_road_network("TwoRoadsWithRoadObjects.xodr", None, None);
    let book = road_network.road_object_book();

    // obj_building is at approximately (50, -5, 0); radius 2.0 should find it.
    let results = book.find_in_radius(50.0, -5.0, 0.0, 2.0);
    let ids: Vec<String> = results.iter().map(|o| o.id()).collect();
    assert!(
        ids.contains(&OBJ_BUILDING_ID.to_string()),
        "Expected obj_building near (50, -5, 0), got: {:?}",
        ids
    );

    // Searching far away with a tiny radius returns nothing.
    let far_results = book.find_in_radius(500.0, 500.0, 500.0, 0.1);
    assert!(
        far_results.is_empty(),
        "Expected empty result far away, got: {:?}",
        far_results.len()
    );
}

#[test]
fn road_object_stop_line_type() {
    let road_network = common::create_malidrive_road_network("RoadWithStopLine.xodr", None, None);
    let book = road_network.road_object_book();

    let objects = book.road_objects();
    assert_eq!(objects.len(), 3, "Expected 3 road objects in RoadWithStopLine.xodr");

    // obj_stop_line_name: roadMark with name="stopLine" → kStopLine.
    let stop_line_name = book
        .get_road_object(&OBJ_STOP_LINE_NAME_ID.to_string())
        .expect("obj_stop_line_name not found");
    assert_eq!(
        stop_line_name.object_type(),
        maliput::api::objects::RoadObjectType::StopLine,
        "Expected StopLine for obj_stop_line_name"
    );
    assert_eq!(stop_line_name.name(), Some("stopLine".to_string()));
    assert!(!stop_line_name.is_dynamic());

    // obj_stop_line_subtype: roadMark with subtype="stopLine" → kStopLine.
    let stop_line_subtype = book
        .get_road_object(&OBJ_STOP_LINE_SUBTYPE_ID.to_string())
        .expect("obj_stop_line_subtype not found");
    assert_eq!(
        stop_line_subtype.object_type(),
        maliput::api::objects::RoadObjectType::StopLine,
        "Expected StopLine for obj_stop_line_subtype"
    );
    assert_eq!(stop_line_subtype.subtype(), Some("stopLine".to_string()));

    // obj_road_mark: regular roadMark → kRoadMark.
    let road_mark = book
        .get_road_object(&OBJ_ROAD_MARK_ID.to_string())
        .expect("obj_road_mark not found");
    assert_eq!(
        road_mark.object_type(),
        maliput::api::objects::RoadObjectType::RoadMark,
        "Expected RoadMark for obj_road_mark"
    );
    assert_eq!(road_mark.name(), Some("TurnArrow".to_string()));
}

#[test]
fn road_object_stop_line_related_lanes() {
    let road_network = common::create_malidrive_road_network("RoadWithStopLine.xodr", None, None);
    let book = road_network.road_object_book();

    // obj_stop_line_name: validity fromLane=-1 toLane=1 → related to both driving lanes.
    let stop_line = book.get_road_object(&OBJ_STOP_LINE_NAME_ID.to_string()).unwrap();
    let lanes = stop_line.related_lanes();
    assert!(
        lanes.contains(&"1_0_-1".to_string()),
        "stop_line_name related lanes: {:?}",
        lanes
    );
    assert!(
        lanes.contains(&"1_0_1".to_string()),
        "stop_line_name related lanes: {:?}",
        lanes
    );
}

#[test]
fn road_object_properties() {
    let road_network = common::create_malidrive_road_network("TwoRoadsWithRoadObjects.xodr", None, None);
    let book = road_network.road_object_book();

    // Verify properties() is callable and returns a HashMap without panicking.
    for obj in book.road_objects() {
        let _props = obj.properties();
    }
}
