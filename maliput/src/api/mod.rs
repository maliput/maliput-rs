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

use crate::common::MaliputError;
use crate::math::Matrix3;
use crate::math::Quaternion;
use crate::math::RollPitchYaw;
use crate::math::Vector3;

pub mod rules;

/// A RoadGeometry.
/// Wrapper around C++ implementation `maliput::api::RoadGeometry`.
/// See RoadNetwork for an example of how to get a RoadGeometry.
pub struct RoadGeometry<'a> {
    rg: &'a maliput_sys::api::ffi::RoadGeometry,
}

impl<'a> RoadGeometry<'a> {
    /// Returns the id of the RoadGeometry.
    pub fn id(&self) -> String {
        maliput_sys::api::ffi::RoadGeometry_id(self.rg)
    }
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
    /// ### Arguments
    /// * `lane_id` - The id of the lane.
    /// ### Return
    /// The lane with the given id.
    /// If no lane is found with the given id, return None.
    pub fn get_lane(&self, lane_id: &String) -> Option<Lane> {
        let lane = maliput_sys::api::ffi::RoadGeometry_GetLane(self.rg, lane_id);
        if lane.lane.is_null() {
            return None;
        }
        Some(Lane {
            lane: unsafe { lane.lane.as_ref().expect("") },
        })
    }
    /// Get all lanes of the `RoadGeometry`.
    /// Returns a vector of `Lane`.
    /// # Example
    /// ```rust, no_run
    /// use maliput::api::RoadNetwork;
    /// use std::collections::HashMap;
    ///
    /// let package_location = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    /// let xodr_path = format!("{}/data/xodr/TShapeRoad.xodr", package_location);
    /// let road_network_properties = HashMap::from([("road_geometry_id", "my_rg_from_rust"), ("opendrive_file", xodr_path.as_str())]);
    /// let road_network = RoadNetwork::new("maliput_malidrive", &road_network_properties)?;
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
    /// Get the segment matching given `segment_id`.
    pub fn get_segment(&self, segment_id: &String) -> Segment {
        unsafe {
            Segment {
                segment: maliput_sys::api::ffi::RoadGeometry_GetSegment(self.rg, segment_id)
                    .as_ref()
                    .expect(""),
            }
        }
    }
    /// Get the junction matching given `junction_id`.
    pub fn get_junction(&self, junction_id: &String) -> Junction {
        unsafe {
            Junction {
                junction: maliput_sys::api::ffi::RoadGeometry_GetJunction(self.rg, junction_id)
                    .as_ref()
                    .expect(""),
            }
        }
    }
    /// Get the branch point matching given `branch_point_id`.
    pub fn get_branch_point(&self, branch_point_id: &String) -> BranchPoint {
        unsafe {
            BranchPoint {
                branch_point: maliput_sys::api::ffi::RoadGeometry_GetBranchPoint(self.rg, branch_point_id)
                    .as_ref()
                    .expect("Underlying BranchPoint is null"),
            }
        }
    }
    /// Execute a custom command on the backend.
    /// ### Details
    /// The documentation of the custom command should be provided by the backend: https://github.com/maliput/maliput_malidrive/blob/main/src/maliput_malidrive/base/road_geometry.h
    ///
    /// ### Arguments
    /// * `command` - The command to execute.
    ///
    /// ### Return
    /// The result of the command.
    pub fn backend_custom_command(&self, command: &String) -> String {
        maliput_sys::api::ffi::RoadGeometry_BackendCustomCommand(self.rg, command)
    }
    /// Obtains the Geo Reference info of this RoadGeometry.
    ///
    /// ### Return
    /// A string containing the Geo Reference projection, if any.
    pub fn geo_reference_info(&self) -> String {
        maliput_sys::api::ffi::RoadGeometry_GeoReferenceInfo(self.rg)
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
/// let package_location = std::env::var("CARGO_MANIFEST_DIR").unwrap();
/// let xodr_path = format!("{}/data/xodr/TShapeRoad.xodr", package_location);
/// let road_network_properties = HashMap::from([("road_geometry_id", "my_rg_from_rust"), ("opendrive_file", xodr_path.as_str())]);
/// let road_network = RoadNetwork::new("maliput_malidrive", &road_network_properties)?;
/// let road_geometry = road_network.road_geometry();
/// println!("num_junctions: {}", road_geometry.num_junctions());
/// ```
pub struct RoadNetwork {
    pub(crate) rn: cxx::UniquePtr<maliput_sys::api::ffi::RoadNetwork>,
}

impl RoadNetwork {
    /// Create a new `RoadNetwork` with the given `road_network_loader_id` and `properties`.
    ///
    /// # Arguments
    ///
    /// * `road_network_loader_id` - The id of the road network loader.
    /// * `properties` - The properties of the road network.
    ///
    /// # Details
    /// It relies on `maliput_sys::plugin::ffi::CreateRoadNetwork` to create a new `RoadNetwork`.
    ///
    /// # Returns
    /// A result containing the `RoadNetwork` or a `MaliputError` if the creation fails.
    pub fn new(
        road_network_loader_id: &str,
        properties: &std::collections::HashMap<&str, &str>,
    ) -> Result<RoadNetwork, MaliputError> {
        // Translate the properties to ffi types
        let mut properties_vec = Vec::new();
        for (key, value) in properties.iter() {
            properties_vec.push(format!("{}:{}", key, value));
        }
        std::env::set_var("MALIPUT_PLUGIN_PATH", maliput_sdk::get_maliput_malidrive_plugin_path());
        let rn = maliput_sys::plugin::ffi::CreateRoadNetwork(&road_network_loader_id.to_string(), &properties_vec)?;
        Ok(RoadNetwork { rn })
    }

    /// Get the `RoadGeometry` of the `RoadNetwork`.
    pub fn road_geometry(&self) -> RoadGeometry {
        unsafe {
            RoadGeometry {
                rg: self.rn.road_geometry().as_ref().expect(""),
            }
        }
    }
    /// Get the `IntersectionBook` of the `RoadNetwork`.
    pub fn intersection_book(&mut self) -> IntersectionBook {
        let intersection_book_ffi = self
            .rn
            .as_mut()
            .expect("Underlying RoadNetwork is null")
            .intersection_book();
        IntersectionBook {
            intersection_book: unsafe {
                intersection_book_ffi
                    .as_mut()
                    .expect("Underlying IntersectionBook is null")
            },
        }
    }
    /// Get the `TrafficLightBook` of the `RoadNetwork`.
    pub fn traffic_light_book(&self) -> rules::TrafficLightBook {
        let traffic_light_book_ffi = self.rn.traffic_light_book();
        rules::TrafficLightBook {
            traffic_light_book: unsafe {
                traffic_light_book_ffi
                    .as_ref()
                    .expect("Underlying TrafficLightBook is null")
            },
        }
    }
    /// Get the `RoadRulebook` of the `RoadNetwork`.
    pub fn rulebook(&self) -> rules::RoadRulebook {
        let rulebook_ffi = self.rn.rulebook();
        rules::RoadRulebook {
            road_rulebook: unsafe { rulebook_ffi.as_ref().expect("Underlying RoadRulebook is null") },
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

/// Isometric velocity vector in a `Lane`-frame.
///
/// sigma_v, rho_v, and eta_v are the components of velocity in a
/// (sigma, rho, eta) coordinate system.  (sigma, rho, eta) have the same
/// orientation as the (s, r, h) at any given point in space, however they
/// form an isometric system with a Cartesian distance metric.  Hence,
/// IsoLaneVelocity represents a "real" physical velocity vector (albeit
/// with an orientation relative to the road surface).
#[derive(Default, Copy, Clone, Debug, PartialEq)]
pub struct IsoLaneVelocity {
    pub sigma_v: f64,
    pub rho_v: f64,
    pub eta_v: f64,
}

impl IsoLaneVelocity {
    /// Create a new `IsoLaneVelocity` with the given `sigma_v`, `rho_v`, and `eta_v` components.
    pub fn new(sigma_v: f64, rho_v: f64, eta_v: f64) -> IsoLaneVelocity {
        IsoLaneVelocity { sigma_v, rho_v, eta_v }
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
    /// Returns the Segment to which this Lane belongs.
    pub fn segment(&self) -> Segment<'a> {
        unsafe {
            Segment {
                segment: self.lane.segment().as_ref().expect(""),
            }
        }
    }
    /// Get the orientation of the `Lane` at the given `LanePosition`.
    pub fn get_orientation(&self, lane_position: &LanePosition) -> Rotation {
        Rotation {
            r: maliput_sys::api::ffi::Lane_GetOrientation(self.lane, lane_position.lp.as_ref().expect("")),
        }
    }
    /// ## Brief
    /// Get the [InertialPosition] of the [Lane] at the given [LanePosition].
    ///
    /// ## Notes
    /// Note there is no constraint for the `r` coordinate, as it can be outside the lane boundaries.
    /// In that scenario, the resultant inertial position represents a point in the `s-r` plane at the given `s` and `h`
    /// coordinates. It's on the user side to verify, if needed, that the lane position is within lane boundaries.
    /// Bare in mind that the inertial position will be a point in the `s-r` plane, but *not* necessarily on the road surface.
    ///
    /// ## Arguments
    /// * `lane_position` - A maliput [LanePosition].
    ///
    /// ## Precondition
    /// The s component of `lane_position` must be in domain [0, Lane::length()].
    ///
    /// ## Return
    /// The [InertialPosition] corresponding to the input [LanePosition].
    pub fn to_inertial_position(&self, lane_position: &LanePosition) -> InertialPosition {
        InertialPosition {
            ip: maliput_sys::api::ffi::Lane_ToInertialPosition(self.lane, lane_position.lp.as_ref().expect("")),
        }
    }
    /// Determines the LanePosition corresponding to InertialPosition `inertial_position`.
    /// The LanePosition is expected to be contained within the lane's boundaries.
    /// See [Lane::to_segment_position] method.
    ///
    /// This method guarantees that its result satisfies the condition that
    /// `to_inertial_position(result.lane_position)` is within `linear_tolerance()`
    ///  of `result.nearest_position`.
    pub fn to_lane_position(&self, inertial_position: &InertialPosition) -> LanePositionResult {
        let lpr = maliput_sys::api::ffi::Lane_ToLanePosition(self.lane, inertial_position.ip.as_ref().expect(""));
        LanePositionResult {
            lane_position: LanePosition {
                lp: maliput_sys::api::ffi::LanePositionResult_road_position(&lpr),
            },
            nearest_position: InertialPosition {
                ip: maliput_sys::api::ffi::LanePositionResult_nearest_position(&lpr),
            },
            distance: maliput_sys::api::ffi::LanePositionResult_distance(&lpr),
        }
    }
    /// Determines the LanePosition corresponding to InertialPosition `inertial_position`.
    /// The LanePosition is expected to be contained within the segment's boundaries.
    /// See [Lane::to_lane_position] method.
    ///
    /// This method guarantees that its result satisfies the condition that
    /// `to_inertial_position(result.lane_position)` is within `linear_tolerance()`
    ///  of `result.nearest_position`.
    pub fn to_segment_position(&self, inertial_position: &InertialPosition) -> LanePositionResult {
        let spr = maliput_sys::api::ffi::Lane_ToSegmentPosition(self.lane, inertial_position.ip.as_ref().expect(""));
        LanePositionResult {
            lane_position: LanePosition {
                lp: maliput_sys::api::ffi::LanePositionResult_road_position(&spr),
            },
            nearest_position: InertialPosition {
                ip: maliput_sys::api::ffi::LanePositionResult_nearest_position(&spr),
            },
            distance: maliput_sys::api::ffi::LanePositionResult_distance(&spr),
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
    /// Computes derivatives of [LanePosition] given a velocity vector `velocity`.
    /// `velocity` is a isometric velocity vector oriented in the `Lane`-frame
    /// at `position`.
    ///
    /// Returns `Lane`-frame derivatives packed into a [LanePosition] struct.
    pub fn eval_motion_derivatives(&self, lane_position: &LanePosition, velocity: &IsoLaneVelocity) -> LanePosition {
        LanePosition {
            lp: maliput_sys::api::ffi::Lane_EvalMotionDerivatives(
                self.lane,
                lane_position.lp.as_ref().expect(""),
                velocity.sigma_v,
                velocity.rho_v,
                velocity.eta_v,
            ),
        }
    }
    /// Returns the lane's BranchPoint for the specified end.
    pub fn get_branch_point(&self, end: &LaneEnd) -> BranchPoint {
        assert! {
            end == &LaneEnd::Start(self.clone()) || end == &LaneEnd::Finish(self.clone()),
            "LaneEnd must be an end of this lane {:?}",
            end
        }
        BranchPoint {
            branch_point: unsafe {
                maliput_sys::api::ffi::Lane_GetBranchPoint(self.lane, end == &LaneEnd::Start(self.clone()))
                    .as_ref()
                    .expect("Underlying BranchPoint is null")
            },
        }
    }
    /// Returns the set of LaneEnd's which connect with this lane on the
    /// same side of the BranchPoint at `end`. At a minimum,
    /// this set will include this Lane.
    pub fn get_confluent_branches(&self, end: &LaneEnd) -> LaneEndSet {
        assert! {
            end == &LaneEnd::Start(self.clone()) || end == &LaneEnd::Finish(self.clone()),
            "LaneEnd must be an end of this lane {:?}",
            end
        }
        LaneEndSet {
            lane_end_set: unsafe {
                maliput_sys::api::ffi::Lane_GetConfluentBranches(self.lane, end == &LaneEnd::Start(self.clone()))
                    .as_ref()
                    .expect("Underlying LaneEndSet is null")
            },
        }
    }
    /// Returns the set of LaneEnd's which continue onward from this lane at the
    /// BranchPoint at `end`.
    pub fn get_ongoing_branches(&self, end: &LaneEnd) -> LaneEndSet {
        assert! {
            end == &LaneEnd::Start(self.clone()) || end == &LaneEnd::Finish(self.clone()),
            "LaneEnd must be an end of this lane {:?}",
            end
        }
        LaneEndSet {
            lane_end_set: unsafe {
                maliput_sys::api::ffi::Lane_GetOngoingBranches(self.lane, end == &LaneEnd::Start(self.clone()))
                    .as_ref()
                    .expect("Underlying LaneEndSet is null")
            },
        }
    }
    /// Returns the default ongoing LaneEnd connected at `end`,
    /// or None if no default branch has been established.
    pub fn get_default_branch(&self, end: &LaneEnd) -> Option<LaneEnd> {
        assert! {
            end == &LaneEnd::Start(self.clone()) || end == &LaneEnd::Finish(self.clone()),
            "LaneEnd must be an end of this lane {:?}",
            end
        }
        let lane_end = maliput_sys::api::ffi::Lane_GetDefaultBranch(self.lane, end == &LaneEnd::Start(self.clone()));
        match lane_end.is_null() {
            true => None,
            false => {
                let lane_end_ref: &maliput_sys::api::ffi::LaneEnd =
                    lane_end.as_ref().expect("Underlying LaneEnd is null");
                let is_start = maliput_sys::api::ffi::LaneEnd_is_start(lane_end_ref);
                let lane_ref = unsafe {
                    maliput_sys::api::ffi::LaneEnd_lane(lane_end_ref)
                        .as_ref()
                        .expect("Underlying LaneEnd is null")
                };
                match is_start {
                    true => Some(LaneEnd::Start(Lane { lane: lane_ref })),
                    false => Some(LaneEnd::Finish(Lane { lane: lane_ref })),
                }
            }
        }
    }
    /// Check if the `Lane` contains the given `LanePosition`.
    pub fn contains(&self, lane_position: &LanePosition) -> bool {
        self.lane.Contains(lane_position.lp.as_ref().expect(""))
    }
}

/// Copy trait for Lane.
/// A reference to the Lane is copied.
impl Clone for Lane<'_> {
    fn clone(&self) -> Self {
        Lane { lane: self.lane }
    }
}

/// A Segment represents a bundle of adjacent Lanes which share a
/// continuously traversable road surface. Every [LanePosition] on a
/// given [Lane] of a Segment has a corresponding [LanePosition] on each
/// other [Lane], all with the same height-above-surface h, that all
/// map to the same GeoPoint in 3-space.
///
/// Segments are grouped by [Junction].
///
/// Wrapper around C++ implementation `maliput::api::Segment`.
pub struct Segment<'a> {
    segment: &'a maliput_sys::api::ffi::Segment,
}

impl<'a> Segment<'a> {
    /// Get the id of the `Segment` as a string.
    pub fn id(&self) -> String {
        maliput_sys::api::ffi::Segment_id(self.segment)
    }
    /// Returns the [Junction] to which this Segment belongs.
    pub fn junction(&self) -> Junction {
        unsafe {
            Junction {
                junction: self.segment.junction().as_ref().expect(""),
            }
        }
    }
    /// Get the number of lanes in the `Segment`.
    pub fn num_lanes(&self) -> i32 {
        self.segment.num_lanes()
    }
    /// Get the lane at the given `index`.
    pub fn lane(&self, index: i32) -> Lane {
        unsafe {
            Lane {
                lane: self.segment.lane(index).as_ref().expect(""),
            }
        }
    }
}

/// A Junction is a closed set of [Segment]s which have physically
/// coplanar road surfaces, in the sense that [RoadPosition]s with the
/// same h value (height above surface) in the domains of two [Segment]s
/// map to the same [InertialPosition].  The [Segment]s need not be directly
/// connected to one another in the network topology.
///
/// Junctions are grouped by [RoadGeometry].
///
/// Wrapper around C++ implementation `maliput::api::Segment`.
pub struct Junction<'a> {
    junction: &'a maliput_sys::api::ffi::Junction,
}

impl<'a> Junction<'a> {
    /// Get the id of the `Junction` as a string.
    pub fn id(&self) -> String {
        maliput_sys::api::ffi::Junction_id(self.junction)
    }
    /// Get the road geometry of the `Junction`.
    pub fn road_geometry(&self) -> RoadGeometry {
        unsafe {
            RoadGeometry {
                rg: self.junction.road_geometry().as_ref().expect(""),
            }
        }
    }
    /// Get the number of segments in the `Junction`.
    pub fn num_segments(&self) -> i32 {
        self.junction.num_segments()
    }
    /// Get the segment at the given `index`.
    pub fn segment(&self, index: i32) -> Segment {
        unsafe {
            Segment {
                segment: self.junction.segment(index).as_ref().expect(""),
            }
        }
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

/// Represents the result of a LanePosition query.
pub struct LanePositionResult {
    pub lane_position: LanePosition,
    pub nearest_position: InertialPosition,
    pub distance: f64,
}

impl LanePositionResult {
    /// Create a new `LanePositionResult` with the given `lane_position`, `nearest_position`, and `distance`.
    pub fn new(lane_position: LanePosition, nearest_position: InertialPosition, distance: f64) -> LanePositionResult {
        LanePositionResult {
            lane_position,
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

/// Directed, inclusive longitudinal (s value) range from s0 to s1.
/// Wrapper around C++ implementation `maliput::api::SRange`.
pub struct SRange {
    s_range: cxx::UniquePtr<maliput_sys::api::ffi::SRange>,
}

impl SRange {
    /// Create a new `SRange` with the given `s0` and `s1`.
    pub fn new(s0: f64, s1: f64) -> SRange {
        SRange {
            s_range: maliput_sys::api::ffi::SRange_new(s0, s1),
        }
    }
    /// Get the s0 of the `SRange`.
    pub fn s0(&self) -> f64 {
        self.s_range.s0()
    }
    /// Get the s1 of the `SRange`.
    pub fn s1(&self) -> f64 {
        self.s_range.s1()
    }
    /// Set the s0 of the `SRange`.
    pub fn set_s0(&mut self, s0: f64) {
        self.s_range.as_mut().expect("Underlying SRange is null").set_s0(s0);
    }
    /// Set the s1 of the `SRange`.
    pub fn set_s1(&mut self, s1: f64) {
        self.s_range.as_mut().expect("Underlying SRange is null").set_s1(s1);
    }
    /// Get the size of the `SRange`.
    pub fn size(&self) -> f64 {
        self.s_range.size()
    }
    /// Returns true When this SRange is in the direction of +s.
    pub fn with_s(&self) -> bool {
        self.s_range.WithS()
    }
    /// Determines whether this SRange intersects with `s_range`.
    pub fn intersects(&self, s_range: &SRange, tolerance: f64) -> bool {
        self.s_range.Intersects(&s_range.s_range, tolerance)
    }
    /// Determines whether this SRange contains `s_range`.
    pub fn contains(&self, s_range: &SRange, tolerance: f64) -> bool {
        self.s_range.Contains(&s_range.s_range, tolerance)
    }
    /// Get the intersection of this SRange with `s_range`.
    /// Returns None if the intersection is empty.
    pub fn get_intersection(&self, s_range: &SRange, tolerance: f64) -> Option<SRange> {
        let intersection = maliput_sys::api::ffi::SRange_GetIntersection(&self.s_range, &s_range.s_range, tolerance);
        match intersection.is_null() {
            true => None,
            false => Some(SRange { s_range: intersection }),
        }
    }
}

/// Directed longitudinal range of a specific Lane, identified by a LaneId.
/// Wrapper around C++ implementation `maliput::api::LaneSRange`.
pub struct LaneSRange {
    pub(crate) lane_s_range: cxx::UniquePtr<maliput_sys::api::ffi::LaneSRange>,
}

impl LaneSRange {
    /// Create a new `LaneSRange` with the given `lane_id` and `s_range`.
    pub fn new(lane_id: &String, s_range: &SRange) -> LaneSRange {
        LaneSRange {
            lane_s_range: maliput_sys::api::ffi::LaneSRange_new(lane_id, &s_range.s_range),
        }
    }
    /// Get the lane id of the `LaneSRange`.
    pub fn lane_id(&self) -> String {
        maliput_sys::api::ffi::LaneSRange_lane_id(&self.lane_s_range)
    }
    /// Get the s range of the `LaneSRange`.
    pub fn s_range(&self) -> SRange {
        SRange {
            s_range: maliput_sys::api::ffi::LaneSRange_s_range(&self.lane_s_range),
        }
    }
    /// Get the length of the `LaneSRange`.
    pub fn length(&self) -> f64 {
        self.lane_s_range.length()
    }
    /// Determines whether this LaneSRange intersects with `lane_s_range`.
    pub fn intersects(&self, lane_s_range: &LaneSRange, tolerance: f64) -> bool {
        self.lane_s_range.Intersects(&lane_s_range.lane_s_range, tolerance)
    }
    /// Determines whether this LaneSRange contains `lane_s_range`.
    pub fn contains(&self, lane_s_range: &LaneSRange, tolerance: f64) -> bool {
        self.lane_s_range.Contains(&lane_s_range.lane_s_range, tolerance)
    }
    /// Get the intersection of this LaneSRange with `lane_s_range`.
    pub fn get_intersection(&self, lane_s_range: &LaneSRange, tolerance: f64) -> Option<LaneSRange> {
        let intersection = maliput_sys::api::ffi::LaneSRange_GetIntersection(
            &self.lane_s_range,
            &lane_s_range.lane_s_range,
            tolerance,
        );
        match intersection.is_null() {
            true => None,
            false => Some(LaneSRange {
                lane_s_range: intersection,
            }),
        }
    }
}

/// A route, possibly spanning multiple (end-to-end) lanes.
///
/// The sequence of [LaneSRange]s should be contiguous by either presenting
/// laterally adjacent [LaneSRange]s, or consecutive [LaneSRange]s. (In other words,
/// taken as a Lane-space path with r=0 and h=0, it should present a
/// G1-continuous curve.)
pub struct LaneSRoute {
    lane_s_route: cxx::UniquePtr<maliput_sys::api::ffi::LaneSRoute>,
}

impl LaneSRoute {
    /// Create a new `LaneSRoute` with the given `ranges`.
    ///
    /// ## Arguments
    /// * `ranges` - A vector of [LaneSRange] to create the [LaneSRoute].
    pub fn new(ranges: Vec<LaneSRange>) -> LaneSRoute {
        let mut lane_s_ranges_cpp = cxx::CxxVector::new();
        for range in &ranges {
            lane_s_ranges_cpp
                .as_mut()
                .unwrap()
                .push(maliput_sys::api::ffi::ConstLaneSRangeRef {
                    lane_s_range: &range.lane_s_range,
                });
        }
        LaneSRoute {
            lane_s_route: maliput_sys::api::ffi::LaneSRoute_new(&lane_s_ranges_cpp),
        }
    }

    /// Returns the sequence of [LaneSRange]s.
    pub fn ranges(&self) -> Vec<LaneSRange> {
        let mut ranges = Vec::new();
        let lane_s_ranges = self.lane_s_route.ranges();
        for range in lane_s_ranges {
            ranges.push(LaneSRange {
                lane_s_range: maliput_sys::api::ffi::LaneSRange_new(
                    &maliput_sys::api::ffi::LaneSRange_lane_id(range),
                    maliput_sys::api::ffi::LaneSRange_s_range(range).as_ref().expect(""),
                ),
            })
        }
        ranges
    }

    /// Computes the accumulated length of all [LaneSRange]s.
    pub fn length(&self) -> f64 {
        self.lane_s_route.length()
    }

    /// Determines whether this LaneSRoute intersects with `other`.
    ///
    /// ## Arguments
    /// * `other` - The other LaneSRoute to check for intersection.
    /// * `tolerance` - The tolerance to use for intersection checks.
    ///
    /// ## Returns
    /// * `true` if the two LaneSRoute intersect, `false` otherwise.
    pub fn intersects(&self, other: &LaneSRoute, tolerance: f64) -> bool {
        self.lane_s_route.Intersects(&other.lane_s_route, tolerance)
    }
}

/// A specific endpoint of a specific Lane.
/// This is analogous to the C++ maliput::api::LaneEnd implementation.
pub enum LaneEnd<'a> {
    /// The start of the Lane. ("s == 0")
    Start(Lane<'a>),
    /// The end of the Lane. ("s == length")
    Finish(Lane<'a>),
}

impl LaneEnd<'_> {
    /// Get the Lane of the `LaneEnd`.
    pub fn lane(&self) -> &Lane {
        match self {
            LaneEnd::Start(lane) => lane,
            LaneEnd::Finish(lane) => lane,
        }
    }
}

impl PartialEq for LaneEnd<'_> {
    fn eq(&self, other: &Self) -> bool {
        match self {
            LaneEnd::Start(lane) => match other {
                LaneEnd::Start(other_lane) => lane.id() == other_lane.id(),
                _ => false,
            },
            LaneEnd::Finish(lane) => match other {
                LaneEnd::Finish(other_lane) => lane.id() == other_lane.id(),
                _ => false,
            },
        }
    }
}

impl Eq for LaneEnd<'_> {}

impl std::fmt::Display for LaneEnd<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            LaneEnd::Start(lane) => write!(f, "LaneEnd::Start({})", lane.id()),
            LaneEnd::Finish(lane) => write!(f, "LaneEnd::Finish({})", lane.id()),
        }
    }
}

impl std::fmt::Debug for LaneEnd<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            LaneEnd::Start(lane) => write!(f, "LaneEnd::Start({})", lane.id()),
            LaneEnd::Finish(lane) => write!(f, "LaneEnd::Finish({})", lane.id()),
        }
    }
}

/// A set of LaneEnds.
pub struct LaneEndSet<'a> {
    lane_end_set: &'a maliput_sys::api::ffi::LaneEndSet,
}

impl<'a> LaneEndSet<'a> {
    /// Obtain the size of the LaneEndSet.
    pub fn size(&self) -> i32 {
        self.lane_end_set.size()
    }
    /// Get the LaneEnd at the given index.
    pub fn get(&self, index: i32) -> LaneEnd {
        let lane_end = self.lane_end_set.get(index);
        // Obtain end type and lane reference.
        let is_start = maliput_sys::api::ffi::LaneEnd_is_start(lane_end);
        let lane_ref = unsafe {
            maliput_sys::api::ffi::LaneEnd_lane(lane_end)
                .as_ref()
                .expect("Underlying LaneEnd is null")
        };
        // Create a LaneEnd enum variant.
        match is_start {
            true => LaneEnd::Start(Lane { lane: lane_ref }),
            false => LaneEnd::Finish(Lane { lane: lane_ref }),
        }
    }

    /// Convert the LaneEndSet to a map of lane-id to LaneEnd.
    pub fn to_lane_map(&self) -> std::collections::HashMap<String, LaneEnd> {
        (0..self.size())
            .map(|i| {
                let end = self.get(i);
                (end.lane().id(), end)
            })
            .collect()
    }
}

/// A BranchPoint is a node in the network of a RoadGeometry at which
/// Lanes connect to one another.  A BranchPoint is a collection of LaneEnds
/// specifying the Lanes (and, in particular, which ends of the Lanes) are
/// connected at the BranchPoint.
///
/// LaneEnds participating in a BranchPoint are grouped into two sets,
/// arbitrarily named "A-side" and "B-side". LaneEnds on the same "side"
/// have coincident into-the-lane tangent vectors, which are anti-parallel
/// to those of LaneEnds on the other side.
pub struct BranchPoint<'a> {
    branch_point: &'a maliput_sys::api::ffi::BranchPoint,
}

impl<'a> BranchPoint<'a> {
    /// Get the id of the `BranchPoint` as a string.
    pub fn id(&self) -> String {
        maliput_sys::api::ffi::BranchPoint_id(self.branch_point)
    }
    pub fn road_geometry(&self) -> RoadGeometry {
        unsafe {
            RoadGeometry {
                rg: self.branch_point.road_geometry().as_ref().expect(""),
            }
        }
    }
    /// Returns the set of LaneEnds on the same side as the given LaneEnd.
    /// E.g: For a T-junction, this would return the set of LaneEnds on the merging side.
    pub fn get_confluent_branches(&self, end: &LaneEnd) -> LaneEndSet {
        let lane_end_set_ptr = self.branch_point.GetConfluentBranches(
            BranchPoint::from_lane_end_to_ffi(end)
                .as_ref()
                .expect("Underlying LaneEnd is null"),
        );
        LaneEndSet {
            lane_end_set: unsafe { lane_end_set_ptr.as_ref().expect("Underlying LaneEndSet is null") },
        }
    }
    /// Returns the set of LaneEnds on the opposite side as the given LaneEnd.
    /// E.g: For a T-junction, this would return the LaneEnds which end flows into the junction.
    pub fn get_ongoing_branches(&self, end: &LaneEnd) -> LaneEndSet {
        let lane_end_set_ptr = self.branch_point.GetOngoingBranches(
            BranchPoint::from_lane_end_to_ffi(end)
                .as_ref()
                .expect("Underlying LaneEnd is null"),
        );
        LaneEndSet {
            lane_end_set: unsafe { lane_end_set_ptr.as_ref().expect("Underlying LaneEndSet is null") },
        }
    }
    /// Returns the default ongoing branch (if any) for the given `end`.
    /// This typically represents what would be considered "continuing
    /// through-traffic" from `end` (e.g., as opposed to a branch executing
    /// a turn).
    ///
    /// If `end` has no default-branch at this BranchPoint, the return
    /// value will be None.
    pub fn get_default_branch(&self, end: &LaneEnd) -> Option<LaneEnd> {
        let lane_end = maliput_sys::api::ffi::BranchPoint_GetDefaultBranch(
            self.branch_point,
            BranchPoint::from_lane_end_to_ffi(end)
                .as_ref()
                .expect("Underlying LaneEnd is null"),
        );
        match lane_end.is_null() {
            true => None,
            false => {
                let lane_end_ref: &maliput_sys::api::ffi::LaneEnd =
                    lane_end.as_ref().expect("Underlying LaneEnd is null");
                let is_start = maliput_sys::api::ffi::LaneEnd_is_start(lane_end_ref);
                let lane_ref = unsafe {
                    maliput_sys::api::ffi::LaneEnd_lane(lane_end_ref)
                        .as_ref()
                        .expect("Underlying LaneEnd is null")
                };
                match is_start {
                    true => Some(LaneEnd::Start(Lane { lane: lane_ref })),
                    false => Some(LaneEnd::Finish(Lane { lane: lane_ref })),
                }
            }
        }
    }
    /// Returns the set of LaneEnds grouped together on the "A-side".
    pub fn get_a_side(&self) -> LaneEndSet {
        let lane_end_set_ptr = self.branch_point.GetASide();
        LaneEndSet {
            lane_end_set: unsafe { lane_end_set_ptr.as_ref().expect("Underlying LaneEndSet is null") },
        }
    }
    /// Returns the set of LaneEnds grouped together on the "B-side".
    pub fn get_b_side(&self) -> LaneEndSet {
        let lane_end_set_ptr = self.branch_point.GetBSide();
        LaneEndSet {
            lane_end_set: unsafe { lane_end_set_ptr.as_ref().expect("Underlying LaneEndSet is null") },
        }
    }
    /// Convert LaneEnd enum to LaneEnd ffi.
    fn from_lane_end_to_ffi(end: &LaneEnd) -> cxx::UniquePtr<maliput_sys::api::ffi::LaneEnd> {
        match end {
            LaneEnd::Start(lane) => unsafe { maliput_sys::api::ffi::LaneEnd_new(lane.lane, true) },
            LaneEnd::Finish(lane) => unsafe { maliput_sys::api::ffi::LaneEnd_new(lane.lane, false) },
        }
    }
}

/// An abstract convenience class that aggregates information pertaining to an
/// intersection. Its primary purpose is to serve as a single source of this
/// information and to remove the need for users to query numerous disparate
/// data structures and state providers.
pub struct Intersection<'a> {
    intersection: &'a mut maliput_sys::api::ffi::Intersection,
}

impl<'a> Intersection<'a> {
    /// Get the id of the `Intersection` as a string.
    pub fn id(&self) -> String {
        maliput_sys::api::ffi::Intersection_id(self.intersection)
    }
}

/// A book of Intersections.
pub struct IntersectionBook<'a> {
    intersection_book: &'a mut maliput_sys::api::ffi::IntersectionBook,
}

impl<'a> IntersectionBook<'a> {
    /// Gets a list of all Intersections within this book.
    pub fn get_intersections(&mut self) -> Vec<Intersection> {
        let book_pin = unsafe { std::pin::Pin::new_unchecked(&mut *self.intersection_book) };
        let intersections_cpp = maliput_sys::api::ffi::IntersectionBook_GetIntersections(book_pin);
        unsafe {
            intersections_cpp
                .into_iter()
                .map(|intersection| Intersection {
                    intersection: intersection
                        .intersection
                        .as_mut()
                        .expect("Underlying Intersection is null"),
                })
                .collect::<Vec<Intersection>>()
        }
    }

    /// Gets the specified Intersection.
    ///
    /// ## Arguments
    ///   * `id` - The id of the Intersection to get.
    ///
    /// ## Returns
    ///   * An `Option<Intersection>`
    ///     * Some(Intersection) - The Intersection with the specified id.
    ///     * None - If the Intersection with the specified id does not exist.
    pub fn get_intersection(&mut self, id: &str) -> Option<Intersection> {
        let book_pin = unsafe { std::pin::Pin::new_unchecked(&mut *self.intersection_book) };
        let intersection_option = unsafe {
            maliput_sys::api::ffi::IntersectionBook_GetIntersection(book_pin, &String::from(id))
                .intersection
                .as_mut()
        };
        match &intersection_option {
            None => None,
            Some(_) => Some(Intersection {
                intersection: intersection_option.expect("Underlying Intersection is null"),
            }),
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

    mod s_range {
        #[test]
        fn s_range_new() {
            let s_range = crate::api::SRange::new(1.0, 2.0);
            assert_eq!(s_range.s0(), 1.0);
            assert_eq!(s_range.s1(), 2.0);
        }
        #[test]
        fn s_range_api() {
            let s_range_1 = crate::api::SRange::new(1.0, 3.0);
            let s_range_2 = crate::api::SRange::new(2.0, 4.0);
            assert_eq!(s_range_1.size(), 2.0);
            assert!(s_range_1.with_s());
            assert!(s_range_1.intersects(&s_range_2, 0.0));
            assert!(!s_range_1.contains(&s_range_2, 0.0));
        }
        #[test]
        fn s_range_setters() {
            let mut s_range = crate::api::SRange::new(0.0, 4.0);
            s_range.set_s0(1.0);
            s_range.set_s1(3.0);
            assert_eq!(s_range.s0(), 1.0);
            assert_eq!(s_range.s1(), 3.0);
        }
        #[test]
        fn s_range_get_intersection_with_intersection() {
            let s_range_1 = crate::api::SRange::new(1.0, 3.0);
            let s_range_2 = crate::api::SRange::new(2.0, 4.0);
            let intersection = s_range_1.get_intersection(&s_range_2, 0.0);
            assert!(intersection.is_some());
            let intersection = intersection.unwrap();
            assert_eq!(intersection.s0(), 2.0);
            assert_eq!(intersection.s1(), 3.0);
        }
        #[test]
        fn s_range_get_intersection_with_no_intersection() {
            let s_range_1 = crate::api::SRange::new(1.0, 2.0);
            let s_range_2 = crate::api::SRange::new(3.0, 4.0);
            let intersection = s_range_1.get_intersection(&s_range_2, 0.0);
            assert!(intersection.is_none());
        }
    }

    mod lane_s_range {
        #[test]
        fn lane_s_range_new() {
            let lane_s_range =
                crate::api::LaneSRange::new(&String::from("lane_test"), &crate::api::SRange::new(1.0, 2.0));
            assert_eq!(lane_s_range.lane_id(), "lane_test");
            assert_eq!(lane_s_range.s_range().s0(), 1.0);
            assert_eq!(lane_s_range.s_range().s1(), 2.0);
            assert_eq!(lane_s_range.length(), 1.0);
        }
        #[test]
        fn lane_s_range_api() {
            let lane_s_range_1 =
                crate::api::LaneSRange::new(&String::from("lane_test"), &crate::api::SRange::new(1.0, 2.0));
            let lane_s_range_2 =
                crate::api::LaneSRange::new(&String::from("lane_test"), &crate::api::SRange::new(2.0, 3.0));
            assert!(lane_s_range_1.intersects(&lane_s_range_2, 0.0));
            assert!(!lane_s_range_1.contains(&lane_s_range_2, 0.0));
        }
        #[test]
        fn lane_s_range_get_intersection_with_intersection() {
            let lane_s_range_1 =
                crate::api::LaneSRange::new(&String::from("lane_test"), &crate::api::SRange::new(1.0, 3.0));
            let lane_s_range_2 =
                crate::api::LaneSRange::new(&String::from("lane_test"), &crate::api::SRange::new(2.0, 4.0));
            let intersection = lane_s_range_1.get_intersection(&lane_s_range_2, 0.0);
            assert!(intersection.is_some());
            let intersection = intersection.unwrap();
            assert_eq!(intersection.lane_id(), "lane_test");
            assert_eq!(intersection.s_range().s0(), 2.0);
            assert_eq!(intersection.s_range().s1(), 3.0);
        }
        #[test]
        fn lane_s_range_get_intersection_with_no_intersection() {
            let lane_s_range_1 =
                crate::api::LaneSRange::new(&String::from("lane test_1"), &crate::api::SRange::new(1.0, 3.0));
            let lane_s_range_2 =
                crate::api::LaneSRange::new(&String::from("lane_test_2"), &crate::api::SRange::new(2.0, 4.0));
            let intersection = lane_s_range_1.get_intersection(&lane_s_range_2, 0.0);
            assert!(intersection.is_none());
        }
    }

    mod lane_s_route {
        // Helper function to create a LaneSRoute
        // with two LaneSRange.
        // ## Arguments
        // * `s0_0` - The s0 of the first LaneSRange.
        // * `s1_0` - The s1 of the first LaneSRange.
        // * `s0_1` - The s0 of the second LaneSRange.
        // * `s1_1` - The s1 of the second LaneSRange.
        fn _get_lane_s_route(s0_0: f64, s1_0: f64, s0_1: f64, s1_1: f64) -> crate::api::LaneSRoute {
            let ranges = vec![
                crate::api::LaneSRange::new(&String::from("lane_test_1"), &crate::api::SRange::new(s0_0, s1_0)),
                crate::api::LaneSRange::new(&String::from("lane_test_2"), &crate::api::SRange::new(s0_1, s1_1)),
            ];
            crate::api::LaneSRoute::new(ranges)
        }
        #[test]
        fn lane_s_route_new() {
            let lane_s_route = _get_lane_s_route(0., 10., 0., 15.);
            assert!(!lane_s_route.lane_s_route.is_null());
            let ranges = lane_s_route.ranges();
            assert_eq!(ranges.len(), 2);
            assert_eq!(ranges[0].lane_id(), "lane_test_1");
            assert_eq!(ranges[1].lane_id(), "lane_test_2");
        }
        #[test]
        fn lane_s_route_length() {
            let lane_s_route = _get_lane_s_route(0., 10., 0., 15.);
            assert_eq!(lane_s_route.length(), 25.0);
        }
        #[test]
        fn lane_s_route_intersects() {
            let lane_s_route = _get_lane_s_route(0., 10., 0., 10.);
            let lane_s_route_that_intersects = _get_lane_s_route(5., 9., 5., 9.);
            let lane_s_route_that_not_intersects = _get_lane_s_route(11., 20., 11., 20.);
            assert!(lane_s_route.intersects(&lane_s_route_that_intersects, 0.0));
            assert!(!lane_s_route.intersects(&lane_s_route_that_not_intersects, 0.0));
        }
    }
}
