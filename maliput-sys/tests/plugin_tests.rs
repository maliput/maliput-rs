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

#[cfg(test)]
mod plugin_tests {
    use maliput_sys::plugin::ffi::CreateRoadNetwork;
    use std::any::Any;
    #[test]
    fn create_valid_road_network_test() {
        std::env::set_var("MALIPUT_PLUGIN_PATH", maliput_sdk::get_maliput_malidrive_plugin_path());
        let road_network_loader_id = String::from("maliput_malidrive");
        let package_location = env!("CARGO_MANIFEST_DIR");
        let xodr_path = format!("opendrive_file:{}/tests/resources/ArcLane.xodr", package_location);
        let road_network_properties = vec![xodr_path];
        let rn_res = CreateRoadNetwork(&road_network_loader_id, &road_network_properties);
        assert!(
            rn_res.is_ok(),
            "Expected RoadNetwork to be created successfully with ArcLane.xodr"
        );
    }
    #[test]
    fn create_invalid_road_network_test() {
        std::env::set_var("MALIPUT_PLUGIN_PATH", maliput_sdk::get_maliput_malidrive_plugin_path());
        let road_network_loader_id = String::from("maliput_malidrive");
        let invalid_xodr_path = "/hopefully/this/path/does/not/exist.xodr".to_string();
        let road_network_properties = vec![invalid_xodr_path];
        let rn_res = CreateRoadNetwork(&road_network_loader_id, &road_network_properties);
        assert!(
            rn_res.is_err(),
            "Expected an error when creating RoadNetwork with an invalid xodr_path"
        );
        let e: cxx::Exception = rn_res.err().unwrap();
        assert!(
            e.type_id() == std::any::TypeId::of::<cxx::Exception>(),
            "Expected cxx::Exception, got: {:?}",
            e.type_id()
        );
    }
}
