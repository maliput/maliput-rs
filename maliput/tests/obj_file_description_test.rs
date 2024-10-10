mod common;

use maliput::utility::{get_obj_description_from_road_network, ObjFeatures};

#[test]
fn get_obj_description_from_road_network_test() {
    let road_network = common::create_t_shape_road_network();
    let features = ObjFeatures {
        max_grid_unit: 1.0,
        min_grid_resolution: 5.0,
        draw_stripes: true,
        draw_arrows: true,
        draw_lane_haze: true,
        draw_branch_points: true,
        draw_elevation_bounds: true,
        off_grid_mesh_generation: false,
        simplify_mesh_threshold: 0.,
        stripe_width: 0.25,
        stripe_elevation: 0.05,
        arrow_elevation: 0.05,
        lane_haze_elevation: 0.02,
        branch_point_elevation: 0.5,
        branch_point_height: 0.5,
        origin: [0.; 3],
        highlighted_segments: Vec::new(),
    };
    let obj_description = get_obj_description_from_road_network(&road_network, &features);
    assert!(obj_description.is_ok());
    let obj_description = obj_description.unwrap();
    assert_ne!(obj_description.len(), 0);
}
