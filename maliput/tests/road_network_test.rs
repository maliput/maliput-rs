// BSD 3-Clause License
//
// Copyright (c) 2025, Woven by Toyota.
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

#[cfg(test)]
mod road_network_test {
    use maliput::common::MaliputError;
    use std::{any::Any, collections::HashMap};
    #[test]
    fn road_network_new() {
        let package_location = std::env::var("CARGO_MANIFEST_DIR").unwrap();
        let xodr_path = format!("{}/data/xodr/TShapeRoad.xodr", package_location);

        let road_network_properties = HashMap::from([
            ("road_geometry_id", "my_rg_from_rust"),
            ("opendrive_file", xodr_path.as_str()),
            ("linear_tolerance", "0.01"),
        ]);
        let rn_res = maliput::api::RoadNetwork::new("maliput_malidrive", &road_network_properties);
        assert!(
            rn_res.is_ok(),
            "Expected RoadNetwork to be created successfully with TShapeRoad.xodr"
        );
        let rn = rn_res.unwrap();
        let rg = rn.road_geometry();
        assert_eq!(rg.id(), "my_rg_from_rust");
    }

    #[test]
    fn road_network_new_error() {
        let invalid_xodr_path = "/hopefully/this/path/does/not/exist.xodr";
        let road_network = maliput::api::RoadNetwork::new(
            "maliput_malidrive",
            &HashMap::from([
                ("road_geometry_id", "my_rg_from_rust"),
                ("opendrive_file", invalid_xodr_path),
            ]),
        );
        assert!(
            road_network.is_err(),
            "Expected an error when creating RoadNetwork with an invalid xodr_path"
        );
        match road_network {
            Ok(_) => panic!("Expected RoadNetwork creation to fail with an invalid xodr_path"),
            Err(e) => {
                assert!(
                    e.type_id() == std::any::TypeId::of::<MaliputError>(),
                    "Expected MaliputError, got: {:?}",
                    e.type_id()
                );
                if let maliput::common::MaliputError::AssertionError(_) = e {
                    // This is the expected error type.
                } else {
                    panic!("Expected MaliputError::AssertionError, got: {:?}", e);
                }
            }
        }
    }

    #[test]
    fn road_network_description_parser_error_test() {
        let package_location = std::env::var("CARGO_MANIFEST_DIR").unwrap();
        let xodr_path = format!("{}/data/xodr/ArcLane-IllFormed.xodr", package_location);

        let road_network_properties = HashMap::from([
            ("road_geometry_id", "my_rg_from_rust"),
            ("opendrive_file", xodr_path.as_str()),
            ("linear_tolerance", "0.01"),
        ]);
        let rn_res = maliput::api::RoadNetwork::new("maliput_malidrive", &road_network_properties);
        assert!(
            rn_res.is_err(),
            "Expected RoadNetworkDescriptionParserError with ArcLane-IllFormed.xodr"
        );
        match rn_res {
            Ok(_) => panic!("Expected RoadNetwork creation to fail with an ill formed XODR."),
            Err(e) => {
                assert!(
                    e.type_id() == std::any::TypeId::of::<MaliputError>(),
                    "Expected MaliputError, got: {:?}",
                    e.type_id()
                );
                if let maliput::common::MaliputError::RoadNetworkDescriptionParserError(_) = e {
                    // This is the expected error type.
                } else {
                    panic!("Expected MaliputError::RoadNetworkDescriptionParserError, got: {:?}", e);
                }
            }
        }
    }
}
