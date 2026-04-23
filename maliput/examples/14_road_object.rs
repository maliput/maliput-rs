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

// This example demonstrates how to use the RoadObject API.
//
// The TwoRoadsWithRoadObjects.xodr map contains two roads, each with a set of
// road objects defined as OpenDRIVE `<object>` elements. Road objects model
// physical features adjacent to or along the road — barriers, buildings,
// vegetation, crosswalks, etc.
//
// Road layout (viewed from above):
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
// Objects placed on Road 1:
//   obj_vegetation  (StarBush)           : type=vegetation, s=0,   t=4,  zOffset=0.0
//                                          star outline (6 corners, closed)
//   obj_barrier     (GuardRail)          : type=barrier,    s=20,  t=3,  zOffset=0.5
//                                          validity fromLane=-1 toLane=-1
//   obj_building    (Warehouse)          : type=building,   s=50,  t=-5, zOffset=0.0
//                                          radius=4.0
//   obj_crosswalk   (PedestrianCrossing) : type=crosswalk,  s=100, t=0,  zOffset=0.0
//                                          validity fromLane=-1 toLane=1
//                                          rect outline (4 corners, closed)
//
// Topics covered:
//   - Loading a road network that contains road objects.
//   - Accessing all RoadObjects via RoadObjectBook::road_objects().
//   - Looking up a RoadObject by ID with RoadObjectBook::get_road_object().
//   - Inspecting object properties: id, name, type, subtype, dynamic flag.
//   - Querying the inertial position and optional lane position.
//   - Accessing the bounding box and its vertices.
//   - Iterating outlines and corners (for objects with polygon shapes).
//   - Reading key-value properties attached to an object.
//   - Filtering objects by type with RoadObjectBook::find_by_type().
//   - Filtering objects by lane with RoadObjectBook::find_by_lane().
//   - Spatial queries with RoadObjectBook::find_in_radius().

fn main() -> Result<(), Box<dyn std::error::Error>> {
    use maliput::api::{objects::RoadObjectType, RoadNetwork, RoadNetworkBackend};
    use std::collections::HashMap;

    // Use the ResourceManager to locate the XODR file supplied by the
    // maliput_malidrive backend.  This avoids hard-coding installation paths.
    let rm = maliput::ResourceManager::new();
    let xodr_path = rm
        .get_resource_path_by_name("maliput_malidrive", "TwoRoadsWithRoadObjects.xodr")
        .unwrap();

    let road_network_properties = HashMap::from([
        ("road_geometry_id", "two_roads_with_road_objects"),
        ("opendrive_file", xodr_path.to_str().unwrap()),
        ("linear_tolerance", "0.01"),
    ]);

    let road_network = RoadNetwork::new(RoadNetworkBackend::MaliputMalidrive, &road_network_properties)?;

    // --- RoadObjectBook overview ---
    //
    // RoadObjectBook is the central registry for all RoadObjects in the network.
    // It is obtained directly from the RoadNetwork.
    let book = road_network.road_object_book();

    let all_objects = book.road_objects();
    println!("Total road objects: {}", all_objects.len());

    for obj in &all_objects {
        // Every RoadObject has a mandatory unique ID and type.
        // The name and subtype are optional attributes that come from the XODR
        // `name` and `subtype` attributes of the `<object>` element.
        let name_str = obj.name().unwrap_or_else(|| "(none)".to_string());
        let subtype_str = obj.subtype().unwrap_or_else(|| "(none)".to_string());
        println!(
            "\n  Object '{}': name='{}' type={:?} subtype='{}'",
            obj.id(),
            name_str,
            obj.object_type(),
            subtype_str
        );

        // is_dynamic indicates whether this object may change its position or
        // state during a simulation.  Static physical features (barriers,
        // buildings, …) return false.
        println!("    is_dynamic: {}", obj.is_dynamic());

        // The position contains the inertial-frame coordinates of the object
        // origin.  When the underlying XODR representation provides a lane
        // association, the position also contains lane-local (s, r, h)
        // coordinates expressed as (lane_id, s, r, h).
        let pos = obj.position();
        let ip = &pos.inertial_position;
        println!(
            "    inertial position: (x={:.3}, y={:.3}, z={:.3})",
            ip.x(),
            ip.y(),
            ip.z()
        );
        if let Some((lane_id, s, r, h)) = &pos.lane_position {
            println!("    lane position: lane='{}' s={:.3} r={:.3} h={:.3}", lane_id, s, r, h);
        }

        // The orientation of the object in the inertial frame is represented as
        // a Rotation (roll-pitch-yaw).
        let orient = obj.orientation();
        println!(
            "    orientation (roll={:.3}, pitch={:.3}, yaw={:.3})",
            orient.roll(),
            orient.pitch(),
            orient.yaw()
        );

        // The bounding box describes the object's oriented bounding volume.
        // get_vertices() always returns 8 corner points.
        let bb = obj.bounding_box();
        println!("    bounding box vertices: {}", bb.get_vertices().len());

        // related_lanes lists the lane IDs that this object is associated with,
        // as derived from the XODR `<validity>` element (or all lanes of the
        // road when no validity is specified).
        let lanes = obj.related_lanes();
        println!("    related_lanes: {:?}", lanes);

        // --- Outlines ---
        //
        // Some objects (e.g. vegetation with a star shape, crosswalks with a
        // rectangular footprint) carry one or more polygon outlines that describe
        // their boundary in the object-local coordinate frame.
        //
        // Objects without an explicit outline (barriers, buildings with only a
        // radius) have num_outlines() == 0.
        let n_outlines = obj.num_outlines();
        println!("    num_outlines: {}", n_outlines);

        if n_outlines > 0 {
            for outline in obj.outlines() {
                println!(
                    "      outline '{}': {} corners, closed={}",
                    outline.id(),
                    outline.num_corners(),
                    outline.is_closed()
                );
                for (i, corner) in outline.corners().iter().enumerate() {
                    // Each OutlineCorner stores its local (x, y, z) coordinates.
                    // The `height` field is Some when the XODR `<cornerLocal>`
                    // element carries an explicit `height` attribute.
                    let h_str = corner
                        .height
                        .map(|h| format!("{:.3}", h))
                        .unwrap_or_else(|| "(none)".to_string());
                    println!(
                        "        corner {}: ({:.3}, {:.3}, {:.3})  height={}",
                        i, corner.x, corner.y, corner.z, h_str
                    );
                }
            }
        }

        // --- Key-value properties ---
        //
        // Properties are arbitrary string–string pairs attached to an object via
        // XODR `<userData>` elements.  Most objects have no properties.
        let props = obj.properties();
        if !props.is_empty() {
            println!("    properties:");
            for (key, val) in &props {
                println!("      '{}' = '{}'", key, val);
            }
        }
    }

    // --- Look up a specific object by ID ---
    //
    // get_road_object() returns Some(&RoadObject) when the ID is found,
    // None otherwise.
    println!("\n--- Look up 'obj_barrier' by ID ---");
    match book.get_road_object(&"obj_barrier".to_string()) {
        Some(barrier) => {
            println!("Found '{}' (type={:?})", barrier.id(), barrier.object_type());
            let pos = barrier.position();
            println!(
                "  position: ({:.3}, {:.3}, {:.3})",
                pos.inertial_position.x(),
                pos.inertial_position.y(),
                pos.inertial_position.z()
            );
        }
        None => println!("Object not found."),
    }

    // --- Filter by type ---
    //
    // find_by_type() returns all objects whose object_type() matches.
    println!("\n--- Objects of type Barrier ---");
    let barriers = book.find_by_type(&RoadObjectType::Barrier);
    println!("  count: {}", barriers.len());
    for b in &barriers {
        println!("  '{}'", b.id());
    }

    println!("\n--- Objects of type Crosswalk ---");
    let crosswalks = book.find_by_type(&RoadObjectType::Crosswalk);
    println!("  count: {}", crosswalks.len());
    for c in &crosswalks {
        println!("  '{}' (related_lanes: {:?})", c.id(), c.related_lanes());
    }

    // --- Filter by lane ---
    //
    // find_by_lane() returns all objects whose related_lanes() contains the
    // given lane ID.
    //
    // obj_barrier has validity fromLane=-1 toLane=-1, so it is only related to
    // lane "1_0_-1".
    println!("\n--- Objects related to lane '1_0_-1' ---");
    let objects_for_lane = book.find_by_lane(&"1_0_-1".to_string());
    println!("  count: {}", objects_for_lane.len());
    for o in &objects_for_lane {
        println!("  '{}' (type={:?})", o.id(), o.object_type());
    }

    // --- Spatial query ---
    //
    // find_in_radius() returns all objects whose inertial position is within
    // `radius` metres of the query point (x, y, z).
    //
    // Querying near (0.0, 4.0, 0.0) with radius 5.0 should find obj_vegetation
    // which sits at approximately (0, 4, 0).
    println!("\n--- Objects within 5.0 m of (0.0, 4.0, 0.0) ---");
    let nearby = book.find_in_radius(0.0, 4.0, 0.0, 5.0);
    println!("  count: {}", nearby.len());
    for o in &nearby {
        let pos = o.position();
        println!(
            "  '{}' at ({:.3}, {:.3}, {:.3})",
            o.id(),
            pos.inertial_position.x(),
            pos.inertial_position.y(),
            pos.inertial_position.z()
        );
    }

    Ok(())
}
