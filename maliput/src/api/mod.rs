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

/// A RoadGeometry.
/// Wrapper around C++ implementation `maliput::api::RoadGeometry`.
/// See RoadNetwork for an example of how to get a RoadGeometry.
pub struct RoadGeometry<'a> {
    rg: &'a maliput_sys::api::ffi::RoadGeometry,
}

impl<'a> RoadGeometry<'a> {
    pub fn num_junctions(&self) -> i32 {
        self.rg.num_junctions()
    }
    pub fn linear_tolerance(&self) -> f64 {
        self.rg.linear_tolerance()
    }
    pub fn angular_tolerance(&self) -> f64 {
        self.rg.angular_tolerance()
    }
    pub fn num_branch_points(&self) -> i32 {
        self.rg.num_branch_points()
    }
}

/// A RoadNetwork.
/// Wrapper around C++ implementation `maliput::api::RoadNetwork`.
///
/// ## Example
///
/// ```rust, no_run
/// use maliput::api::RoadNetwork;
/// use std::collections::HashMap;
///
/// let maliput_malidrive_plugin_path = maliput_sdk::get_maliput_malidrive_plugin_path();
/// std::env::set_var("MALIPUT_PLUGIN_PATH", maliput_malidrive_plugin_path);
/// let package_location = std::env::var("CARGO_MANIFEST_DIR").unwrap();
/// let xodr_path = format!("{}/data/xodr/TShapeRoad.xodr", package_location);
/// let mut properties = HashMap::new();
/// properties.insert("road_geometry_id", "my_rg_from_rust");
/// properties.insert("opendrive_file", xodr_path.as_str());
/// let road_network = RoadNetwork::new("maliput_malidrive", &properties);
/// let road_geometry = road_network.road_geometry();
/// println!("num_junctions: {}", road_geometry.num_junctions());
/// ```
pub struct RoadNetwork {
    rn: cxx::UniquePtr<maliput_sys::api::ffi::RoadNetwork>,
}

impl RoadNetwork {
    /// Create a new `RoadNetwork` with the given `road_network_loader_id` and `properties`.
    ///
    /// # Requisites
    /// The environment variable `MALIPUT_PLUGIN_PATH` must be set so maliput discovers the plugins.
    ///
    /// # Arguments
    ///
    /// * `road_network_loader_id` - The id of the road network loader.
    /// * `properties` - The properties of the road network.
    ///
    /// # Details
    /// It relies on `maliput_sys::plugin::ffi::CreateRoadNetwork` to create a new `RoadNetwork`.
    pub fn new(road_network_loader_id: &str, properties: &std::collections::HashMap<&str, &str>) -> RoadNetwork {
        // Translate the properties to ffi types
        let mut properties_vec = Vec::new();
        for (key, value) in properties.iter() {
            properties_vec.push(format!("{}:{}", key, value));
        }

        RoadNetwork {
            rn: maliput_sys::plugin::ffi::CreateRoadNetwork(&road_network_loader_id.to_string(), &properties_vec),
        }
    }

    /// Get the `RoadGeometry` of the `RoadNetwork`.
    pub fn road_geometry(&self) -> RoadGeometry {
        unsafe {
            RoadGeometry {
                rg: self.rn.road_geometry().as_ref().expect(""),
            }
        }
    }
}

/// A Lane Position.
/// Wrapper around C++ implementation `maliput::api::LanePosition`.
///
/// ## Example
///
/// ```rust, no_run
/// use maliput::api::LanePosition;
///
/// let lane_pos = LanePosition::new(1.0, 2.0, 3.0);
/// println!("lane_pos = {}", lane_pos);
/// assert_eq!(lane_pos.s(), 1.0);
/// assert_eq!(lane_pos.r(), 2.0);
/// assert_eq!(lane_pos.h(), 3.0);
/// ```
pub struct LanePosition {
    lp: cxx::UniquePtr<maliput_sys::api::ffi::LanePosition>,
}

use crate::math::Vector3;

impl LanePosition {
    /// Create a new `LanePosition` with the given `s`, `r`, and `h` components.
    pub fn new(s: f64, r: f64, h: f64) -> LanePosition {
        LanePosition {
            lp: maliput_sys::api::ffi::LanePosition_new(s, r, h),
        }
    }
    /// Get the `s` component of the `LanePosition`.
    pub fn s(&self) -> f64 {
        self.lp.s()
    }
    /// Get the `r` component of the `LanePosition`.
    pub fn r(&self) -> f64 {
        self.lp.r()
    }
    /// Get the `h` component of the `LanePosition`.
    pub fn h(&self) -> f64 {
        self.lp.h()
    }

    /// Returns all components as 3-vector `[s, r, h]`.
    pub fn srh(&self) -> Vector3 {
        let srh = self.lp.srh();
        Vector3::new(srh.x(), srh.y(), srh.z())
    }

    /// Set the `s` component of the `LanePosition`.
    pub fn set_s(&mut self, s: f64) {
        self.lp.as_mut().expect("Underlying LanePosition is null").set_s(s);
    }

    /// Set the `r` component of the `LanePosition`.
    pub fn set_r(&mut self, r: f64) {
        self.lp.as_mut().expect("Underlying LanePosition is null").set_r(r);
    }

    /// Set the `h` component of the `LanePosition`.
    pub fn set_h(&mut self, h: f64) {
        self.lp.as_mut().expect("Underlying LanePosition is null").set_h(h);
    }

    /// Set all components from 3-vector `[s, r, h]`.
    pub fn set_srh(&mut self, srh: &Vector3) {
        let ffi_vec = maliput_sys::math::ffi::Vector3_new(srh.x(), srh.y(), srh.z());
        self.lp
            .as_mut()
            .expect("Underlying LanePosition is null")
            .set_srh(&ffi_vec);
    }
}

impl PartialEq for LanePosition {
    fn eq(&self, other: &Self) -> bool {
        self.srh() == other.srh()
    }
}

impl Eq for LanePosition {}

impl std::fmt::Display for LanePosition {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{}", maliput_sys::api::ffi::LanePosition_to_str(&self.lp))
    }
}

impl std::fmt::Debug for LanePosition {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        f.debug_struct("LanePosition")
            .field("s", &self.s())
            .field("r", &self.r())
            .field("h", &self.h())
            .finish()
    }
}

/// An Inertial Position.
/// Wrapper around C++ implementation `maliput::api::InertialPosition`.
///
/// ## Example
///
/// ```rust, no_run
/// use maliput::api::InertialPosition;
///
/// let inertial_pos = InertialPosition::new(1.0, 2.0, 3.0);
/// println!("inertial_pos = {}", inertial_pos);
/// assert_eq!(inertial_pos.x(), 1.0);
/// assert_eq!(inertial_pos.y(), 2.0);
/// assert_eq!(inertial_pos.z(), 3.0);
/// ```
pub struct InertialPosition {
    ip: cxx::UniquePtr<maliput_sys::api::ffi::InertialPosition>,
}

impl InertialPosition {
    /// Create a new `InertialPosition` with the given `x`, `y`, and `z` components.
    pub fn new(x: f64, y: f64, z: f64) -> InertialPosition {
        InertialPosition {
            ip: maliput_sys::api::ffi::InertialPosition_new(x, y, z),
        }
    }
    /// Get the `x` component of the `InertialPosition`.
    pub fn x(&self) -> f64 {
        self.ip.x()
    }
    /// Get the `y` component of the `InertialPosition`.
    pub fn y(&self) -> f64 {
        self.ip.y()
    }
    /// Get the `z` component of the `InertialPosition`.
    pub fn z(&self) -> f64 {
        self.ip.z()
    }

    /// Returns all components as 3-vector `[x, y, z]`.
    pub fn xyz(&self) -> Vector3 {
        let xyz = self.ip.xyz();
        Vector3::new(xyz.x(), xyz.y(), xyz.z())
    }

    /// Set the `x` component of the `InertialPosition`.
    pub fn set_x(&mut self, x: f64) {
        self.ip.as_mut().expect("Underlying InertialPosition is null").set_x(x);
    }

    /// Set the `y` component of the `InertialPosition`.
    pub fn set_y(&mut self, y: f64) {
        self.ip.as_mut().expect("Underlying InertialPosition is null").set_y(y);
    }

    /// Set the `z` component of the `InertialPosition`.
    pub fn set_z(&mut self, z: f64) {
        self.ip.as_mut().expect("Underlying InertialPosition is null").set_z(z);
    }

    /// Set all components from 3-vector `[x, y, z]`.
    pub fn set_xyz(&mut self, xyz: &Vector3) {
        let ffi_vec = maliput_sys::math::ffi::Vector3_new(xyz.x(), xyz.y(), xyz.z());
        self.ip
            .as_mut()
            .expect("Underlying InertialPosition is null")
            .set_xyz(&ffi_vec);
    }

    /// Get the length of `InertialPosition`.
    pub fn length(&self) -> f64 {
        self.ip.length()
    }

    /// Get the distance between two `InertialPosition`.
    pub fn distance(&self, other: &InertialPosition) -> f64 {
        self.ip.Distance(&other.ip)
    }
}

impl PartialEq for InertialPosition {
    fn eq(&self, other: &Self) -> bool {
        maliput_sys::api::ffi::InertialPosition_operator_eq(&self.ip, &other.ip)
    }
}

impl Eq for InertialPosition {}

impl std::fmt::Display for InertialPosition {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{}", maliput_sys::api::ffi::InertialPosition_to_str(&self.ip))
    }
}

impl std::fmt::Debug for InertialPosition {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        f.debug_struct("InertialPosition")
            .field("x", &self.x())
            .field("y", &self.y())
            .field("z", &self.z())
            .finish()
    }
}

impl std::ops::Add for InertialPosition {
    type Output = InertialPosition;

    fn add(self, other: InertialPosition) -> InertialPosition {
        InertialPosition {
            ip: maliput_sys::api::ffi::InertialPosition_operator_sum(&self.ip, &other.ip),
        }
    }
}

impl std::ops::Sub for InertialPosition {
    type Output = InertialPosition;

    fn sub(self, other: InertialPosition) -> InertialPosition {
        InertialPosition {
            ip: maliput_sys::api::ffi::InertialPosition_operator_sub(&self.ip, &other.ip),
        }
    }
}

impl std::ops::Mul<f64> for InertialPosition {
    type Output = InertialPosition;

    fn mul(self, scalar: f64) -> InertialPosition {
        InertialPosition {
            ip: maliput_sys::api::ffi::InertialPosition_operator_mul_scalar(&self.ip, scalar),
        }
    }
}

mod tests {
    mod lane_position {
        #[test]
        fn lane_position_new() {
            let lane_pos = crate::api::LanePosition::new(1.0, 2.0, 3.0);
            assert_eq!(lane_pos.s(), 1.0);
            assert_eq!(lane_pos.r(), 2.0);
            assert_eq!(lane_pos.h(), 3.0);
        }

        #[test]
        fn equality() {
            let v = crate::api::LanePosition::new(1.0, 2.0, 3.0);
            let w = crate::api::LanePosition::new(1.0, 2.0, 3.0);
            assert_eq!(v, w);
            let z = crate::api::LanePosition::new(4.0, 5.0, 6.0);
            assert_ne!(v, z);
        }

        #[test]
        fn set_s() {
            let mut lane_pos = crate::api::LanePosition::new(1.0, 2.0, 3.0);
            lane_pos.set_s(4.0);
            assert_eq!(lane_pos.s(), 4.0);
        }

        #[test]
        fn set_r() {
            let mut lane_pos = crate::api::LanePosition::new(1.0, 2.0, 3.0);
            lane_pos.set_r(4.0);
            assert_eq!(lane_pos.r(), 4.0);
        }

        #[test]
        fn set_h() {
            let mut lane_pos = crate::api::LanePosition::new(1.0, 2.0, 3.0);
            lane_pos.set_h(4.0);
            assert_eq!(lane_pos.h(), 4.0);
        }

        #[test]
        fn set_srh() {
            use crate::math::Vector3;
            let mut lane_pos = crate::api::LanePosition::new(1.0, 2.0, 3.0);
            let vector = Vector3::new(4.0, 5.0, 6.0);
            lane_pos.set_srh(&vector);
            assert_eq!(lane_pos.s(), 4.0);
            assert_eq!(lane_pos.r(), 5.0);
            assert_eq!(lane_pos.h(), 6.0);
        }
    }

    mod inertial_position {

        #[test]
        fn inertial_position_new() {
            let inertial_pos = crate::api::InertialPosition::new(1.0, 2.0, 3.0);
            assert_eq!(inertial_pos.x(), 1.0);
            assert_eq!(inertial_pos.y(), 2.0);
            assert_eq!(inertial_pos.z(), 3.0);
        }

        #[test]
        fn equality() {
            let v = crate::api::InertialPosition::new(1.0, 2.0, 3.0);
            let w = crate::api::InertialPosition::new(1.0, 2.0, 3.0);
            assert_eq!(v, w);
            let z = crate::api::InertialPosition::new(4.0, 5.0, 6.0);
            assert_ne!(v, z);
        }

        #[test]
        fn set_x() {
            let mut inertial_pos = crate::api::InertialPosition::new(1.0, 2.0, 3.0);
            inertial_pos.set_x(4.0);
            assert_eq!(inertial_pos.x(), 4.0);
        }

        #[test]
        fn set_y() {
            let mut inertial_pos = crate::api::InertialPosition::new(1.0, 2.0, 3.0);
            inertial_pos.set_y(4.0);
            assert_eq!(inertial_pos.y(), 4.0);
        }

        #[test]
        fn set_z() {
            let mut inertial_pos = crate::api::InertialPosition::new(1.0, 2.0, 3.0);
            inertial_pos.set_z(4.0);
            assert_eq!(inertial_pos.z(), 4.0);
        }

        #[test]
        fn set_xyz() {
            use crate::math::Vector3;
            let mut inertial_pos = crate::api::InertialPosition::new(1.0, 2.0, 3.0);
            let vector = Vector3::new(4.0, 5.0, 6.0);
            inertial_pos.set_xyz(&vector);
            assert_eq!(inertial_pos.x(), 4.0);
            assert_eq!(inertial_pos.y(), 5.0);
            assert_eq!(inertial_pos.z(), 6.0);
        }

        #[test]
        fn xyz() {
            let inertial_pos = crate::api::InertialPosition::new(1.0, 2.0, 3.0);
            assert_eq!(inertial_pos.xyz(), crate::math::Vector3::new(1.0, 2.0, 3.0));
        }

        #[test]
        fn length() {
            let inertial_pos = crate::api::InertialPosition::new(3.0, 0.0, 4.0);
            assert_eq!(inertial_pos.length(), 5.0);
        }

        #[test]
        fn distance() {
            let inertial_pos1 = crate::api::InertialPosition::new(1.0, 1.0, 1.0);
            let inertial_pos2 = crate::api::InertialPosition::new(5.0, 1.0, 1.0);
            assert_eq!(inertial_pos1.distance(&inertial_pos2), 4.0);
        }

        #[test]
        fn str() {
            let inertial_pos = crate::api::InertialPosition::new(1.0, 2.0, 3.0);
            assert_eq!(inertial_pos.to_string(), "(x = 1, y = 2, z = 3)");
        }

        #[test]
        fn add_operation() {
            let inertial_pos1 = crate::api::InertialPosition::new(1.0, 2.0, 3.0);
            let inertial_pos2 = crate::api::InertialPosition::new(4.0, 5.0, 6.0);
            let inertial_pos3 = inertial_pos1 + inertial_pos2;
            assert_eq!(inertial_pos3.x(), 5.0);
            assert_eq!(inertial_pos3.y(), 7.0);
            assert_eq!(inertial_pos3.z(), 9.0);
        }

        #[test]
        fn sub_operation() {
            let inertial_pos1 = crate::api::InertialPosition::new(1.0, 2.0, 3.0);
            let inertial_pos2 = crate::api::InertialPosition::new(4.0, 5.0, 6.0);
            let inertial_pos3 = inertial_pos1 - inertial_pos2;
            assert_eq!(inertial_pos3.x(), -3.0);
            assert_eq!(inertial_pos3.y(), -3.0);
            assert_eq!(inertial_pos3.z(), -3.0);
        }

        #[test]
        fn mul_scalar_operation() {
            let inertial_pos1 = crate::api::InertialPosition::new(1.0, 2.0, 3.0);
            let inertial_pos2 = inertial_pos1 * 2.0;
            assert_eq!(inertial_pos2.x(), 2.0);
            assert_eq!(inertial_pos2.y(), 4.0);
            assert_eq!(inertial_pos2.z(), 6.0);
        }
    }
}
