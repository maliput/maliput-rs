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

use crate::math::Matrix3;
use crate::math::Quaternion;
use crate::math::RollPitchYaw;
use crate::math::Vector3;

/// A RoadGeometry.
/// Wrapper around C++ implementation `maliput::api::RoadGeometry`.
/// See RoadNetwork for an example of how to get a RoadGeometry.
pub struct RoadGeometry<'a> {
    rg: &'a maliput_sys::api::ffi::RoadGeometry,
}

impl<'a> RoadGeometry<'a> {
    /// Returns the number of Junctions in the RoadGeometry.
    ///
    /// Return value is non-negative.
    pub fn num_junctions(&self) -> i32 {
        self.rg.num_junctions()
    }
    /// Returns the tolerance guaranteed for linear measurements (positions).
    pub fn linear_tolerance(&self) -> f64 {
        self.rg.linear_tolerance()
    }
    /// Returns the tolerance guaranteed for angular measurements (orientations).
    pub fn angular_tolerance(&self) -> f64 {
        self.rg.angular_tolerance()
    }
    /// Returns the number of BranchPoints in the RoadGeometry.
    ///
    /// Return value is non-negative.
    pub fn num_branch_points(&self) -> i32 {
        self.rg.num_branch_points()
    }
    /// Determines the RoadPosition corresponding to InertialPosition `inertial_position`.
    ///
    /// Returns a RoadPositionResult. Its RoadPosition is the point in the
    /// RoadGeometry's manifold which is, in the `Inertial`-frame, closest to
    /// `inertial_position`. Its InertialPosition is the `Inertial`-frame equivalent of the
    /// RoadPosition and its distance is the Cartesian distance from
    /// `inertial_position` to the nearest point.
    ///
    /// This method guarantees that its result satisfies the condition that
    /// `result.lane.to_lane_position(result.pos)` is within `linear_tolerance()`
    /// of the returned InertialPosition.
    ///
    /// The map from RoadGeometry to the `Inertial`-frame is not onto (as a bounded
    /// RoadGeometry cannot completely cover the unbounded Cartesian universe).
    /// If `inertial_position` does represent a point contained within the volume
    /// of the RoadGeometry, then result distance is guaranteed to be less
    /// than or equal to `linear_tolerance()`.
    ///
    /// The map from RoadGeometry to `Inertial`-frame is not necessarily one-to-one.
    /// Different `(s,r,h)` coordinates from different Lanes, potentially from
    /// different Segments, may map to the same `(x,y,z)` `Inertial`-frame location.
    ///
    /// If `inertial_position` is contained within the volumes of multiple Segments,
    /// then ToRoadPosition() will choose a Segment which yields the minimum
    /// height `h` value in the result.  If the chosen Segment has multiple
    /// Lanes, then ToRoadPosition() will choose a Lane which contains
    /// `inertial_position` within its `lane_bounds()` if possible, and if that is
    /// still ambiguous, it will further select a Lane which minimizes the
    /// absolute value of the lateral `r` coordinate in the result.
    ///
    /// Wrapper around C++ implementation `maliput::api::RoadGeometry::ToRoadPosition`.
    pub fn to_road_position(&self, inertial_position: &InertialPosition) -> RoadPositionResult {
        let rpr = maliput_sys::api::ffi::RoadGeometry_ToRoadPosition(self.rg, &inertial_position.ip);
        RoadPositionResult {
            road_position: RoadPosition {
                rp: maliput_sys::api::ffi::RoadPositionResult_road_position(&rpr),
            },
            nearest_position: InertialPosition {
                ip: maliput_sys::api::ffi::RoadPositionResult_nearest_position(&rpr),
            },
            distance: maliput_sys::api::ffi::RoadPositionResult_distance(&rpr),
        }
    }
    /// Get the lane matching given `lane_id`.
    pub fn get_lane(&self, lane_id: &String) -> Lane {
        unsafe {
            Lane {
                lane: maliput_sys::api::ffi::RoadGeometry_GetLane(self.rg, lane_id)
                    .lane
                    .as_ref()
                    .expect(""),
            }
        }
    }
    /// Get all lanes of the `RoadGeometry`.
    /// Returns a vector of `Lane`.
    /// # Example
    /// ```rust, no_run
    /// use maliput::api::RoadNetwork;
    /// use std::collections::HashMap;
    ///
    /// let maliput_malidrive_plugin_path = maliput_sdk::get_maliput_malidrive_plugin_path();
    /// std::env::set_var("MALIPUT_PLUGIN_PATH", maliput_malidrive_plugin_path);
    /// let package_location = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    /// let xodr_path = format!("{}/data/xodr/TShapeRoad.xodr", package_location);
    /// let road_network_properties = HashMap::from([("road_geometry_id", "my_rg_from_rust"), ("opendrive_file", xodr_path.as_str())]);
    /// let road_network = RoadNetwork::new("maliput_malidrive", &road_network_properties);
    /// let road_geometry = road_network.road_geometry();
    /// let lanes = road_geometry.get_lanes();
    /// for lane in lanes {
    ///    println!("lane_id: {}", lane.id());
    /// }
    /// ```
    pub fn get_lanes(&self) -> Vec<Lane> {
        let lanes = maliput_sys::api::ffi::RoadGeometry_GetLanes(self.rg);
        lanes
            .into_iter()
            .map(|l| Lane {
                lane: unsafe { l.lane.as_ref().expect("") },
            })
            .collect::<Vec<Lane>>()
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
/// let road_network_properties = HashMap::from([("road_geometry_id", "my_rg_from_rust"), ("opendrive_file", xodr_path.as_str())]);
/// let road_network = RoadNetwork::new("maliput_malidrive", &road_network_properties);
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

/// Bounds in the lateral dimension (r component) of a `Lane`-frame, consisting
/// of a pair of minimum and maximum r value.  The bounds must straddle r = 0,
/// i.e., the minimum must be <= 0 and the maximum must be >= 0.
pub struct RBounds {
    min: f64,
    max: f64,
}

impl RBounds {
    pub fn new(min: f64, max: f64) -> RBounds {
        RBounds { min, max }
    }
    pub fn min(&self) -> f64 {
        self.min
    }
    pub fn max(&self) -> f64 {
        self.max
    }
    pub fn set_min(&mut self, min: f64) {
        self.min = min;
    }
    pub fn set_max(&mut self, max: f64) {
        self.max = max;
    }
}

/// Bounds in the elevation dimension (`h` component) of a `Lane`-frame,
/// consisting of a pair of minimum and maximum `h` value.  The bounds
/// must straddle `h = 0`, i.e., the minimum must be `<= 0` and the
/// maximum must be `>= 0`.
pub struct HBounds {
    min: f64,
    max: f64,
}

impl HBounds {
    pub fn new(min: f64, max: f64) -> HBounds {
        HBounds { min, max }
    }
    pub fn min(&self) -> f64 {
        self.min
    }
    pub fn max(&self) -> f64 {
        self.max
    }
    pub fn set_min(&mut self, min: f64) {
        self.min = min;
    }
    pub fn set_max(&mut self, max: f64) {
        self.max = max;
    }
}

/// A maliput::api::Lane
/// Wrapper around C++ implementation `maliput::api::Lane`.
pub struct Lane<'a> {
    lane: &'a maliput_sys::api::ffi::Lane,
}

impl<'a> Lane<'a> {
    /// Returns the index of this Lane within the Segment which owns it.    
    pub fn index(&self) -> i32 {
        self.lane.index()
    }
    /// Get the left lane of the `Lane`.
    pub fn to_left(&self) -> Option<Lane> {
        let lane = self.lane.to_left();
        if lane.is_null() {
            None
        } else {
            unsafe {
                Some(Lane {
                    lane: lane.as_ref().expect(""),
                })
            }
        }
    }
    /// Get the right lane of the `Lane`.
    pub fn to_right(&self) -> Option<Lane> {
        let lane = self.lane.to_right();
        if lane.is_null() {
            None
        } else {
            unsafe {
                Some(Lane {
                    lane: lane.as_ref().expect(""),
                })
            }
        }
    }
    /// Get the length of the `Lane`.
    pub fn length(&self) -> f64 {
        self.lane.length()
    }
    /// Get the id of the `Lane` as a string.
    pub fn id(&self) -> String {
        maliput_sys::api::ffi::Lane_id(self.lane)
    }
    /// Get the orientation of the `Lane` at the given `LanePosition`.
    pub fn get_orientation(&self, lane_position: &LanePosition) -> Rotation {
        Rotation {
            r: maliput_sys::api::ffi::Lane_GetOrientation(self.lane, lane_position.lp.as_ref().expect("")),
        }
    }
    /// Get the inertial position of the `Lane` at the given `LanePosition`.
    pub fn to_inertial_position(&self, lane_position: &LanePosition) -> InertialPosition {
        InertialPosition {
            ip: maliput_sys::api::ffi::Lane_ToInertialPosition(self.lane, lane_position.lp.as_ref().expect("")),
        }
    }
    /// Get the lane bounds of the `Lane` at the given `s`.
    pub fn lane_bounds(&self, s: f64) -> RBounds {
        let bounds = maliput_sys::api::ffi::Lane_lane_bounds(self.lane, s);
        RBounds::new(bounds.min(), bounds.max())
    }
    /// Get the segment bounds of the `Lane` at the given `s`.
    pub fn segment_bounds(&self, s: f64) -> RBounds {
        let bounds = maliput_sys::api::ffi::Lane_segment_bounds(self.lane, s);
        RBounds::new(bounds.min(), bounds.max())
    }
    /// Get the elevation bounds of the `Lane` at the given `s` and `r`.
    pub fn elevation_bounds(&self, s: f64, r: f64) -> HBounds {
        let bounds = maliput_sys::api::ffi::Lane_elevation_bounds(self.lane, s, r);
        HBounds::new(bounds.min(), bounds.max())
    }
    /// Check if the `Lane` contains the given `LanePosition`.
    pub fn contains(&self, lane_position: &LanePosition) -> bool {
        self.lane.Contains(lane_position.lp.as_ref().expect(""))
    }
}

/// A maliput::api::RoadPosition
/// Wrapper around C++ implementation `maliput::api::RoadPosition`.
pub struct RoadPosition {
    rp: cxx::UniquePtr<maliput_sys::api::ffi::RoadPosition>,
}

impl RoadPosition {
    /// Create a new `RoadPosition` with the given `lane` and `lane_pos`.
    pub fn new(lane: &Lane, lane_pos: &LanePosition) -> RoadPosition {
        unsafe {
            RoadPosition {
                rp: maliput_sys::api::ffi::RoadPosition_new(lane.lane, &lane_pos.lp),
            }
        }
    }
    /// Get the inertial position of the `RoadPosition` via doing a Lane::ToInertialPosition query call.
    pub fn to_inertial_position(&self) -> InertialPosition {
        InertialPosition {
            ip: maliput_sys::api::ffi::RoadPosition_ToInertialPosition(&self.rp),
        }
    }
    /// Get the lane of the `RoadPosition`.
    pub fn lane(&self) -> Lane {
        unsafe {
            Lane {
                lane: maliput_sys::api::ffi::RoadPosition_lane(&self.rp).as_ref().expect(""),
            }
        }
    }
    /// Get the lane position of the `RoadPosition`.
    pub fn pos(&self) -> LanePosition {
        LanePosition {
            lp: maliput_sys::api::ffi::RoadPosition_pos(&self.rp),
        }
    }
}

/// Represents the result of a RoadPosition query.
pub struct RoadPositionResult {
    pub road_position: RoadPosition,
    pub nearest_position: InertialPosition,
    pub distance: f64,
}

impl RoadPositionResult {
    /// Create a new `RoadPositionResult` with the given `road_position`, `nearest_position`, and `distance`.
    pub fn new(road_position: RoadPosition, nearest_position: InertialPosition, distance: f64) -> RoadPositionResult {
        RoadPositionResult {
            road_position,
            nearest_position,
            distance,
        }
    }
}

/// A maliput::api::Rotation
/// A wrapper around C++ implementation `maliput::api::Rotation`.
pub struct Rotation {
    r: cxx::UniquePtr<maliput_sys::api::ffi::Rotation>,
}

impl Default for Rotation {
    fn default() -> Self {
        Self::new()
    }
}

impl Rotation {
    /// Create a new `Rotation`.
    pub fn new() -> Rotation {
        Rotation {
            r: maliput_sys::api::ffi::Rotation_new(),
        }
    }
    /// Create a new `Rotation` from a `Quaternion`.
    pub fn from_quat(q: &Quaternion) -> Rotation {
        let q_ffi = maliput_sys::math::ffi::Quaternion_new(q.w(), q.x(), q.y(), q.z());
        Rotation {
            r: maliput_sys::api::ffi::Rotation_from_quat(&q_ffi),
        }
    }
    /// Create a new `Rotation` from a `RollPitchYaw`.
    pub fn from_rpy(rpy: &RollPitchYaw) -> Rotation {
        let rpy_ffi = maliput_sys::math::ffi::RollPitchYaw_new(rpy.roll_angle(), rpy.pitch_angle(), rpy.yaw_angle());
        Rotation {
            r: maliput_sys::api::ffi::Rotation_from_rpy(&rpy_ffi),
        }
    }
    /// Get the roll of the `Rotation`.
    pub fn roll(&self) -> f64 {
        self.r.roll()
    }
    /// Get the pitch of the `Rotation`.
    pub fn pitch(&self) -> f64 {
        self.r.pitch()
    }
    /// Get the yaw of the `Rotation`.
    pub fn yaw(&self) -> f64 {
        self.r.yaw()
    }
    /// Get a quaternion representation of the `Rotation`.
    pub fn quat(&self) -> Quaternion {
        let q_ffi = self.r.quat();
        Quaternion::new(q_ffi.w(), q_ffi.x(), q_ffi.y(), q_ffi.z())
    }
    /// Get a roll-pitch-yaw representation of the `Rotation`.
    pub fn rpy(&self) -> RollPitchYaw {
        let rpy_ffi = maliput_sys::api::ffi::Rotation_rpy(&self.r);
        RollPitchYaw::new(rpy_ffi.roll_angle(), rpy_ffi.pitch_angle(), rpy_ffi.yaw_angle())
    }
    /// Set the `Rotation` from a `Quaternion`.
    pub fn set_quat(&mut self, q: &Quaternion) {
        let q_ffi = maliput_sys::math::ffi::Quaternion_new(q.w(), q.x(), q.y(), q.z());
        maliput_sys::api::ffi::Rotation_set_quat(self.r.pin_mut(), &q_ffi);
    }
    /// Get the matrix representation of the `Rotation`.
    pub fn matrix(&self) -> Matrix3 {
        let matrix_ffi: cxx::UniquePtr<maliput_sys::math::ffi::Matrix3> =
            maliput_sys::api::ffi::Rotation_matrix(&self.r);
        let row_0 = maliput_sys::math::ffi::Matrix3_row(matrix_ffi.as_ref().expect(""), 0);
        let row_1 = maliput_sys::math::ffi::Matrix3_row(matrix_ffi.as_ref().expect(""), 1);
        let row_2 = maliput_sys::math::ffi::Matrix3_row(matrix_ffi.as_ref().expect(""), 2);
        Matrix3::new(
            Vector3::new(row_0.x(), row_0.y(), row_0.z()),
            Vector3::new(row_1.x(), row_1.y(), row_1.z()),
            Vector3::new(row_2.x(), row_2.y(), row_2.z()),
        )
    }
    /// Get the distance between two `Rotation`.
    pub fn distance(&self, other: &Rotation) -> f64 {
        self.r.Distance(&other.r)
    }
    /// Apply the `Rotation` to an `InertialPosition`.
    pub fn apply(&self, v: &InertialPosition) -> InertialPosition {
        InertialPosition {
            ip: maliput_sys::api::ffi::Rotation_Apply(&self.r, &v.ip),
        }
    }
    /// Get the reverse of the `Rotation`.
    pub fn reverse(&self) -> Rotation {
        Rotation {
            r: maliput_sys::api::ffi::Rotation_Reverse(&self.r),
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
    mod rotation {
        #[test]
        fn rotation_new() {
            let rotation = crate::api::Rotation::new();
            assert_eq!(rotation.roll(), 0.0);
            assert_eq!(rotation.pitch(), 0.0);
            assert_eq!(rotation.yaw(), 0.0);
        }

        #[test]
        fn from_quat() {
            let quat = crate::math::Quaternion::new(1.0, 0.0, 0.0, 0.0);
            let rotation = crate::api::Rotation::from_quat(&quat);
            assert_eq!(rotation.roll(), 0.0);
            assert_eq!(rotation.pitch(), 0.0);
            assert_eq!(rotation.yaw(), 0.0);
        }

        #[test]
        fn from_rpy() {
            let rpy = crate::math::RollPitchYaw::new(0.0, 0.0, 0.0);
            let rotation = crate::api::Rotation::from_rpy(&rpy);
            assert_eq!(rotation.roll(), 0.0);
            assert_eq!(rotation.pitch(), 0.0);
            assert_eq!(rotation.yaw(), 0.0);
        }

        #[test]
        fn set_quat() {
            let mut rotation = crate::api::Rotation::new();
            let quat = crate::math::Quaternion::new(1.0, 0.0, 0.0, 0.0);
            rotation.set_quat(&quat);
            assert_eq!(rotation.roll(), 0.0);
            assert_eq!(rotation.pitch(), 0.0);
            assert_eq!(rotation.yaw(), 0.0);
        }

        #[test]
        fn matrix() {
            let rotation = crate::api::Rotation::new();
            let matrix = rotation.matrix();
            assert_eq!(matrix.row(0), crate::math::Vector3::new(1.0, 0.0, 0.0));
            assert_eq!(matrix.row(1), crate::math::Vector3::new(0.0, 1.0, 0.0));
            assert_eq!(matrix.row(2), crate::math::Vector3::new(0.0, 0.0, 1.0));
        }
    }
}
