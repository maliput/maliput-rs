// BSD 3-Clause License
//
// Copyright (c) 2024, Woven by Toyota.
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

// TShapeRoad maps is being used to test the traffic light API and its components.
// YAML information about the TrafficLightBook can be found at:
// https://github.com/maliput/maliput_malidrive/blob/7136af97ae5415077c49ad955098610abd3a7f04/resources/TShapeRoad.yaml

#[test]
fn traffic_light_test_api() {
    let road_network = common::create_t_shape_road_network_with_books();

    let road_geometry = road_network.road_geometry();
    assert_eq!(road_geometry.id(), "my_rg_from_rust");

    let book = road_network.traffic_light_book();
    let expected_traffic_light_id = String::from("EastFacing");
    let traffic_lights = book.traffic_lights();
    assert_eq!(traffic_lights.len(), 1);
    let only_traffic_light = traffic_lights.first().expect("No traffic lights found");
    assert_eq!(only_traffic_light.id(), expected_traffic_light_id);

    let traffic_light = book.get_traffic_light(&String::from("wrong_traffic_light_id"));
    assert!(traffic_light.is_none());

    let traffic_light = book.get_traffic_light(&expected_traffic_light_id);
    assert!(traffic_light.is_some());
    let traffic_light = traffic_light.unwrap();
    assert_eq!(traffic_light.id(), expected_traffic_light_id);

    let position = traffic_light.position_road_network();
    assert_eq!(position.x(), 46.0);
    assert_eq!(position.y(), -5.0);
    assert_eq!(position.z(), 2.0);

    let orientation = traffic_light.orientation_road_network();
    use std::f64::consts::PI;
    assert_eq!(orientation.roll(), -PI);
    assert_eq!(orientation.pitch(), 0.0);
    assert_eq!(orientation.yaw(), PI);

    let bulb_groups = traffic_light.bulb_groups();
    assert_eq!(bulb_groups.len(), 1);
    let bulb_group = traffic_light.get_bulb_group(&String::from("EastFacingBulbs"));
    assert!(bulb_group.is_some());
}

#[test]
fn bulb_group_test_api() {
    let road_network = common::create_t_shape_road_network_with_books();
    let book = road_network.traffic_light_book();
    let traffic_light = book.get_traffic_light(&String::from("EastFacing")).unwrap();
    let bulb_groups = traffic_light.bulb_groups();
    assert_eq!(bulb_groups.len(), 1);
    let bulb_group = bulb_groups.first().expect("No bulb groups found");
    assert_eq!(bulb_group.id(), "EastFacingBulbs");

    // UniqueBulbGroupId tests.
    let unique_id = bulb_group.unique_id();
    let traffic_light_id = unique_id.traffic_light_id();
    assert_eq!(traffic_light_id, traffic_light.id());
    let bulb_group_id = unique_id.bulb_group_id();
    assert_eq!(bulb_group_id, bulb_group.id());
    let bulb_group_unique_id = unique_id.string();
    assert_eq!(bulb_group_unique_id, "EastFacing-EastFacingBulbs");

    // BulbGroups pose tests.
    let position_traffic_light = bulb_group.position_traffic_light();
    assert_eq!(position_traffic_light.x(), 0.0);
    assert_eq!(position_traffic_light.y(), 0.0);
    assert_eq!(position_traffic_light.z(), 0.0);
    let orientation_traffic_light = bulb_group.orientation_traffic_light();
    assert_eq!(orientation_traffic_light.roll(), 0.0);
    assert_eq!(orientation_traffic_light.pitch(), 0.0);
    assert_eq!(orientation_traffic_light.yaw(), 0.0);
    let bulbs = bulb_group.bulbs();
    assert_eq!(bulbs.len(), 4);

    // BulbGroup get api.
    let bulb = bulb_group.get_bulb(&String::from("RedBulb"));
    assert!(bulb.is_some());
    let position_bulb = bulb.unwrap().position_bulb_group();
    assert_eq!(position_bulb.z(), 0.3937);
    let bulb = bulb_group.get_bulb(&String::from("YellowBulb"));
    assert!(bulb.is_some());
    let position_bulb = bulb.unwrap().position_bulb_group();
    assert_eq!(position_bulb.z(), 0.);
    let bulb = bulb_group.get_bulb(&String::from("GreenBulb"));
    assert!(bulb.is_some());
    let position_bulb = bulb.unwrap().position_bulb_group();
    assert_eq!(position_bulb.z(), -0.3937);
    let bulb = bulb_group.get_bulb(&String::from("YellowLeftArrowBulb"));
    assert!(bulb.is_some());
    let position_bulb = bulb.unwrap().position_bulb_group();
    assert_eq!(position_bulb.y(), -0.3937);
    assert_eq!(position_bulb.z(), -0.3937);
    let bulb = bulb_group.get_bulb(&String::from("wrong_bulb_id"));
    assert!(bulb.is_none());

    let traffic_light_from_bulb_group = bulb_group.traffic_light();
    assert_eq!(traffic_light_from_bulb_group.id(), traffic_light.id());
}

#[test]
fn bulb_test_api() {
    let road_network = common::create_t_shape_road_network_with_books();
    let book = road_network.traffic_light_book();
    let traffic_light = book.get_traffic_light(&String::from("EastFacing")).unwrap();
    let bulb_group = traffic_light.get_bulb_group(&String::from("EastFacingBulbs")).unwrap();
    let bulb = bulb_group.get_bulb(&String::from("RedBulb")).unwrap();

    // UniqueBulbId tests.
    let unique_id = bulb.unique_id();
    let traffic_light_id = unique_id.traffic_light_id();
    assert_eq!(traffic_light_id, traffic_light.id());
    let bulb_group_id = unique_id.bulb_group_id();
    assert_eq!(bulb_group_id, bulb_group.id());
    let bulb_id = unique_id.bulb_id();
    assert_eq!(bulb_id, bulb.id());
    let bulb_unique_id = unique_id.string();
    assert_eq!(bulb_unique_id, "EastFacing-EastFacingBulbs-RedBulb");

    // Test on red bulb.
    assert_eq!(bulb.id(), "RedBulb");
    let position = bulb.position_bulb_group();
    assert_eq!(position.x(), 0.0);
    assert_eq!(position.y(), 0.0);
    assert_eq!(position.z(), 0.3937);
    let orientation = bulb.orientation_bulb_group();
    assert_eq!(orientation.roll(), 0.0);
    assert_eq!(orientation.pitch(), 0.0);
    assert_eq!(orientation.yaw(), 0.0);
    let color = bulb.color();
    assert_eq!(color, maliput::api::rules::BulbColor::Red);
    let bulb_type = bulb.bulb_type();
    assert_eq!(bulb_type, maliput::api::rules::BulbType::Round);
    let arrow_orientation = bulb.arrow_orientation_rad();
    assert!(arrow_orientation.is_none());
    let states = bulb.states();
    assert_eq!(states.len(), 2);
    let default_state = bulb.get_default_state();
    assert_eq!(default_state, maliput::api::rules::BulbState::Off);
    let is_valid_state = bulb.is_valid_state(&maliput::api::rules::BulbState::On);
    assert!(is_valid_state);
    let bounding_box = bulb.bounding_box();
    let p_min = bounding_box.0;
    assert_eq!(p_min.x(), -0.0889);
    assert_eq!(p_min.y(), -0.1778);
    assert_eq!(p_min.z(), -0.1778);
    let p_max = bounding_box.1;
    assert_eq!(p_max.x(), 0.0889);
    assert_eq!(p_max.y(), 0.1778);
    assert_eq!(p_max.z(), 0.1778);
    let bulb_group_from_bulb = bulb.bulb_group();
    assert_eq!(bulb_group_from_bulb.id(), bulb_group.id());

    // Extends tests with arrow bulb to the left.
    let bulb = bulb_group.get_bulb(&String::from("YellowLeftArrowBulb")).unwrap();
    let arrow_orientation = bulb.arrow_orientation_rad();
    assert!(arrow_orientation.is_some());
    #[allow(clippy::approx_constant)]
    let expected_orientation = 3.14;
    assert_eq!(arrow_orientation.unwrap(), expected_orientation);
}
