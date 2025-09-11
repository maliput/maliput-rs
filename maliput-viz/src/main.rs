use maliput::api::{InertialPosition, Lane, LanePosition};
use rerun as rr;
use rerun::datatypes::Vec3D;

enum BoundSide {
    Left,
    Right,
}

fn get_bound_points(lane: &Lane, num_points: usize, sampling_step: f64, side: BoundSide) -> Vec<Vec3D> {
    (0..num_points)
        .map(|i| {
            let s = ((i as f64) * sampling_step).min(lane.length());
            let rbounds = lane.lane_bounds(s).unwrap_or_else(|_| {
                panic!("Failed to get lane bounds for s = {}", s);
            });
            let r = match side {
                BoundSide::Left => rbounds.max(),
                BoundSide::Right => rbounds.min(),
            };
            let lane_pos = LanePosition::new(s, r, 0.0);
            // Convert lane-relative position to world (inertial) position
            let inertial_pos: InertialPosition = lane
                .to_inertial_position(&lane_pos)
                .expect("Failed to convert lane position to inertial position");

            // Convert to a Rerun Vec3D for logging
            Vec3D::new(
                inertial_pos.x() as f32,
                inertial_pos.y() as f32,
                inertial_pos.z() as f32,
            )
        })
        .collect::<Vec<_>>()
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    use maliput::api::RoadNetwork;
    use std::collections::HashMap;

    // Get location of odr resources
    let package_location = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let xodr_path = format!("{}/data/xodr/Town01.xodr", package_location);

    let road_network_properties = HashMap::from([
        ("road_geometry_id", "my_rg_from_rust"),
        ("opendrive_file", xodr_path.as_str()),
        ("linear_tolerance", "0.1"),
        ("build_policy", "parallel"),
    ]);

    let now = std::time::SystemTime::now();
    let road_network = RoadNetwork::new("maliput_malidrive", &road_network_properties)?;
    println!("RoadNetwork creation took: {:?}", now.elapsed().unwrap());
    let rg = road_network.road_geometry();
    let rec = rerun::RecordingStreamBuilder::new("maliput-viz").spawn()?;

    let sampling_step = 0.2; // meters
    let lanes = rg.get_lanes();

    let now = std::time::SystemTime::now();
    for lane in &lanes {
        println!(
            "Lane ID: {}, Length: {}, Num Points: {}",
            lane.id(),
            lane.length(),
            (lane.length() / sampling_step) as usize
        );

        let num_points = (lane.length() / sampling_step).ceil() as usize + 1;
        let left_bound_points = get_bound_points(lane, num_points, sampling_step, BoundSide::Left);
        let right_bound_points = get_bound_points(lane, num_points, sampling_step, BoundSide::Right);

        // Log the left and right bounds as 3D line strips
        rec.log(
            format!("world/road_network/{}/left_bound", lane.id()),
            &rr::LineStrips3D::new([left_bound_points]),
        )?;
        rec.log(
            format!("world/road_network/{}/right_bound", lane.id()),
            &rr::LineStrips3D::new([right_bound_points]),
        )?;
    }
    println!("Visualization logging took: {:?}", now.elapsed().unwrap());

    Ok(())
}
