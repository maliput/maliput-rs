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

// This example demonstrates how to use the TrafficSign API.
//
// A TrafficSign is a static, passive signaling device placed along or above the
// road to convey regulatory, warning, or informational messages (e.g. stop
// signs, yield signs, speed limit plates).  Unlike TrafficLights, traffic signs
// carry no dynamic state — they simply exist at a position with an orientation
// and a semantic type.
//
// The RoadWithTrafficSigns.xodr map defines two roads, each carrying one
// stop sign as an OpenDRIVE `<signal>` element.  Road 1 also carries a custom
// sign (type 12345).  The TrafficSignalDatabase.yaml file (located in the
// `data/traffic_signal_db/` subdirectory of the crate) describes the physical
// geometry (bounding box dimensions) of each signal type so that TrafficSign
// objects can be constructed with a proper bounding box.
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
//
// Signs defined in the map:
//   SS1 (name="StopSign_SS1") : type=Stop, s=50, t=2, zOffset=1
//                               validity fromLane=-1 toLane=-1
//                               Road 2 also holds a signalReference to SS1
//                               (no validity → all lanes of road 2)
//                               → related_lanes(SS1): {1_0_-1, 2_0_1, 2_0_-1}
//
//   SS2 (name="StopSign_SS2") : type=Stop, s=40, t=-2, zOffset=1
//                               validity: none (all lanes of road 2)
//                               Road 1 also holds a signalReference to SS2
//                               (no validity → all lanes of road 1)
//                               → related_lanes(SS2): {2_0_1, 2_0_-1, 1_0_1, 1_0_-1}
//
//   CS1 (name="CustomSign_CS1") : type=Unknown, s=75, t=-3, zOffset=1
//                                 validity: none (all lanes of road 1)
//                                 → related_lanes(CS1): {1_0_1, 1_0_-1}
//
// Topics covered:
//   - Loading a road network that populates the TrafficSignBook from XODR
//     signal elements via a traffic_signal_db YAML file.
//   - Listing all TrafficSigns with TrafficSignBook::traffic_signs().
//   - Looking up a specific sign by ID with TrafficSignBook::get_traffic_sign().
//   - Inspecting sign properties: id, type, position, orientation, message,
//     related_lanes and bounding box.
//   - Filtering signs by lane with TrafficSignBook::find_by_lane().
//   - Filtering signs by type with TrafficSignBook::find_by_type().

fn main() -> Result<(), Box<dyn std::error::Error>> {
    use maliput::api::{rules::TrafficSignType, RoadNetwork, RoadNetworkBackend};
    use std::collections::HashMap;

    // Use CARGO_MANIFEST_DIR to locate the local XODR and traffic_signal_db files.
    let package_location = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let xodr_path = format!("{}/data/xodr/RoadWithTrafficSigns.xodr", package_location);

    // The traffic_signal_db key points to a YAML file that defines signal type
    // templates (bounding box dimensions, rule mappings, …).  The backend uses
    // this file together with the `<signal>` descriptions in the XODR to build
    // TrafficSign objects.
    let db_path = format!("{}/data/traffic_signal_db/TrafficSignalDatabase.yaml", package_location);

    let road_network_properties = HashMap::from([
        ("road_geometry_id", "two_roads_with_traffic_signs"),
        ("opendrive_file", xodr_path.as_str()),
        ("traffic_signal_db", db_path.as_str()),
        ("linear_tolerance", "0.01"),
    ]);

    let road_network = RoadNetwork::new(RoadNetworkBackend::MaliputMalidrive, &road_network_properties)?;

    // --- TrafficSignBook overview ---
    //
    // TrafficSignBook is the central registry for all TrafficSigns in the
    // network.  It is obtained directly from the RoadNetwork.
    let book = road_network.traffic_sign_book();

    let all_signs = book.traffic_signs();
    println!("Total traffic signs: {}", all_signs.len());

    for sign in &all_signs {
        // Every TrafficSign has a unique ID derived from the XODR signal's `id`
        // attribute, and a semantic type encoded as a TrafficSignType enum value.
        println!("\n  Sign '{}' (type={:?}):", sign.id(), sign.sign_type());

        // The position is expressed in the road network's inertial frame.
        // For a sign at XODR (s=50, t=2, zOffset=1) on a straight road
        // starting at (0,0), the inertial position is approximately (50, 2, 1).
        let pos = sign.position_road_network();
        println!("    position (x={:.3}, y={:.3}, z={:.3})", pos.x(), pos.y(), pos.z());

        // The orientation is also in the inertial frame, expressed as a
        // Rotation (roll-pitch-yaw).
        let orient = sign.orientation_road_network();
        println!(
            "    orientation (roll={:.3}, pitch={:.3}, yaw={:.3})",
            orient.roll(),
            orient.pitch(),
            orient.yaw()
        );

        // The optional message field carries any text printed on the sign face
        // (e.g. a speed value for a speed limit sign).  It is None for signs
        // that carry no textual content.
        let msg = sign.message().unwrap_or_else(|| "(none)".to_string());
        println!("    message: {}", msg);

        // related_lanes lists every lane ID that this sign is physically
        // relevant to.  This is derived from the XODR `<validity>` element on
        // the originating `<signal>` and any `<signalReference>` elements on
        // other roads that point to the same sign ID.
        println!("    related_lanes: {:?}", sign.related_lanes());

        // The bounding box describes the sign's oriented bounding volume in the
        // inertial frame.  get_vertices() always returns 8 corner points.
        let bb = sign.bounding_box();
        println!("    bounding box vertices: {}", bb.get_vertices().len());
    }

    // --- Look up a specific sign by ID ---
    //
    // get_traffic_sign() returns Some(&TrafficSign) when the ID is found,
    // None otherwise.
    println!("\n--- Look up 'SS1' by ID ---");
    match book.get_traffic_sign(&"SS1".to_string()) {
        Some(ss1) => {
            let pos = ss1.position_road_network();
            println!(
                "Found '{}' at ({:.3}, {:.3}, {:.3})",
                ss1.id(),
                pos.x(),
                pos.y(),
                pos.z()
            );
            println!("  type: {:?}", ss1.sign_type());
        }
        None => println!("Sign not found."),
    }

    // --- Filter by lane ---
    //
    // find_by_lane() returns all signs whose related_lanes() contains the given
    // lane ID.
    //
    // Lane "1_0_-1" is directly covered by SS1's validity element and is also
    // reached by SS2 via a signalReference on road 1, so both signs are
    // returned.
    println!("\n--- Signs related to lane '1_0_-1' ---");
    let signs_for_lane = book.find_by_lane(&"1_0_-1".to_string());
    println!("  count: {}", signs_for_lane.len());
    for s in &signs_for_lane {
        println!("  '{}' (type={:?})", s.id(), s.sign_type());
    }

    // Lane "1_0_1" is only reached by SS2 (via the signalReference on road 1,
    // which has no validity restriction → all lanes of road 1).
    println!("\n--- Signs related to lane '1_0_1' ---");
    let signs_1_0_1 = book.find_by_lane(&"1_0_1".to_string());
    println!("  count: {}", signs_1_0_1.len());
    for s in &signs_1_0_1 {
        println!("  '{}' (type={:?})", s.id(), s.sign_type());
    }

    // A lane that does not exist in the map returns an empty vector.
    let signs_nonexistent = book.find_by_lane(&"nonexistent_lane".to_string());
    println!(
        "\n--- Signs related to 'nonexistent_lane': {} ---",
        signs_nonexistent.len()
    );

    // --- Filter by type ---
    //
    // find_by_type() returns all signs whose sign_type() matches the given
    // TrafficSignType variant.
    //
    // Both SS1 and SS2 are Stop signs.
    println!("\n--- Stop signs ---");
    let stop_signs = book.find_by_type(&TrafficSignType::Stop);
    println!("  count: {}", stop_signs.len());
    for s in &stop_signs {
        println!("  '{}' related_lanes={:?}", s.id(), s.related_lanes());
    }

    // No Yield signs are defined in this map.
    let yield_signs = book.find_by_type(&TrafficSignType::Yield);
    println!("\n--- Yield signs: {} ---", yield_signs.len());

    // CS1 is a custom sign (type 12345) which maps to TrafficSignType::Unknown.
    println!("\n--- Unknown signs ---");
    let unknown_signs = book.find_by_type(&TrafficSignType::Unknown);
    println!("  count: {}", unknown_signs.len());
    for s in &unknown_signs {
        println!("  '{}' related_lanes={:?}", s.id(), s.related_lanes());
    }

    Ok(())
}
