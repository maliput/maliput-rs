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
}
