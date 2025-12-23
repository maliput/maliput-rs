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

/// Represents a complete Maliput road network.
///
/// A `RoadNetwork` is the main entry point for interacting with a road map in Maliput.
/// It serves as a container for all the elements that describe a road network,
/// including its physical layout and the rules of the road.
///
/// It provides access to the following key components:
///
/// * [`RoadGeometry`]: The geometric description of the road surfaces.
/// * [`rules::RoadRulebook`]: The set of traffic rules, like speed limits and right-of-way.
/// * [`rules::TrafficLightBook`]: A catalog of all traffic lights in the network.
/// * [`IntersectionBook`]: A collection of logical intersections and their states.
///   TODO: Complete with other books when available (e.g., `RuleRegistry / PhaseRingBook / etc)
///
/// More info can be found at https://maliput.readthedocs.io/en/latest/html/deps/maliput/html/maliput_design.html.
///
/// # Example
///
/// ```rust, no_run
/// use maliput::api::RoadNetwork;
/// use std::collections::HashMap;
///
/// // Properties to load an OpenDRIVE file using the "maliput_malidrive" backend.
/// let package_location = std::env::var("CARGO_MANIFEST_DIR").unwrap();
/// let xodr_path = format!("{}/data/xodr/TShapeRoad.xodr", package_location);
/// let road_network_properties = HashMap::from([("road_geometry_id", "my_rg_from_rust"), ("opendrive_file", xodr_path.as_str())]);
///
/// // Create the RoadNetwork by specifying the loader ID and properties.
/// let road_network = RoadNetwork::new("maliput_malidrive", &road_network_properties).unwrap();
/// ```
pub struct RoadNetwork {
    pub(crate) rn: cxx::UniquePtr<maliput_sys::api::ffi::RoadNetwork>,
}

impl RoadNetwork {
    /// Create a new `RoadNetwork` with the given `road_network_loader_id` and `properties`.
    ///
    /// # Arguments
    ///
    /// * `road_network_loader_id` - The id of the road network loader. It identifies the backend to be used (e.g., "maliput_malidrive").
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
        // If MALIPUT_PLUGIN_PATH is not set, it will be created.
        let new_path = match std::env::var_os("MALIPUT_PLUGIN_PATH") {
            Some(current_path) => {
                // Add the maliput_malidrive plugin path obtained from maliput_sdk to MALIPUT_PLUGIN_PATH.
                // This is added first in the list as the plugins are loaded sequentally and we
                // want this to be used only when no others are present. (typically in dev mode).
                let mut new_paths = vec![maliput_sdk::get_maliput_malidrive_plugin_path()];
                new_paths.extend(std::env::split_paths(&current_path).collect::<Vec<_>>());
                std::env::join_paths(new_paths).unwrap()
            }
            None => maliput_sdk::get_maliput_malidrive_plugin_path().into(),
        };
        std::env::set_var("MALIPUT_PLUGIN_PATH", new_path);
        let rn = maliput_sys::plugin::ffi::CreateRoadNetwork(&road_network_loader_id.to_string(), &properties_vec)?;
        Ok(RoadNetwork { rn })
    }

    /// Get the `RoadGeometry` of the `RoadNetwork`.
    pub fn road_geometry(&self) -> RoadGeometry<'_> {
        unsafe {
            RoadGeometry {
                rg: self.rn.road_geometry().as_ref().expect(""),
            }
        }
    }
    /// Get the `IntersectionBook` of the `RoadNetwork`.
    pub fn intersection_book(&self) -> IntersectionBook<'_> {
        let intersection_book_ffi = maliput_sys::api::ffi::RoadNetwork_intersection_book(&self.rn);
        IntersectionBook {
            intersection_book: unsafe {
                intersection_book_ffi
                    .as_ref()
                    .expect("Underlying IntersectionBook is null")
            },
        }
    }
    /// Get the `TrafficLightBook` of the `RoadNetwork`.
    pub fn traffic_light_book(&self) -> rules::TrafficLightBook<'_> {
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
    pub fn rulebook(&self) -> rules::RoadRulebook<'_> {
        let rulebook_ffi = self.rn.rulebook();
        rules::RoadRulebook {
            road_rulebook: unsafe { rulebook_ffi.as_ref().expect("Underlying RoadRulebook is null") },
        }
    }

    /// Get the `PhaseRingBook` of the `RoadNetwork`.
    pub fn phase_ring_book(&self) -> rules::PhaseRingBook<'_> {
        let phase_ring_book_ffi = self.rn.phase_ring_book();
        rules::PhaseRingBook {
            phase_ring_book: unsafe { phase_ring_book_ffi.as_ref().expect("Underlying PhaseRingBook is null") },
        }
    }

    /// Get the `RuleRegistry` of the `RoadNetwork`.
    pub fn rule_registry(&self) -> rules::RuleRegistry<'_> {
        let rule_registry_ffi = self.rn.rule_registry();
        rules::RuleRegistry {
            rule_registry: unsafe { rule_registry_ffi.as_ref().expect("Underlying RuleRegistry is null") },
        }
    }

    /// Get the `PhaseProvider` of the `RoadNetwork`.
    pub fn phase_provider(&self) -> rules::PhaseProvider<'_> {
        let phase_provider_ffi = maliput_sys::api::ffi::RoadNetwork_phase_provider(&self.rn);
        rules::PhaseProvider {
            phase_provider: unsafe { phase_provider_ffi.as_ref().expect("Underlying PhaseProvider is null") },
        }
    }

    /// Get the `DiscreteValueRuleStateProvider` of the `RoadNetwork`.
    pub fn discrete_value_rule_state_provider(&self) -> rules::DiscreteValueRuleStateProvider<'_> {
        let state_provider = maliput_sys::api::ffi::RoadNetwork_discrete_value_rule_state_provider(&self.rn);
        rules::DiscreteValueRuleStateProvider {
            state_provider: unsafe {
                state_provider
                    .as_ref()
                    .expect("Underlying DiscreteValueRuleStateProvider is null")
            },
        }
    }

    /// Get the `RangeValueRuleStateProvider` of the `RoadNetwork`.
    pub fn range_value_rule_state_provider(&self) -> rules::RangeValueRuleStateProvider<'_> {
        let state_provider = maliput_sys::api::ffi::RoadNetwork_range_value_rule_state_provider(&self.rn);
        rules::RangeValueRuleStateProvider {
            state_provider: unsafe {
                state_provider
                    .as_ref()
                    .expect("Underlying RangeValueRuleStateProvider is null")
            },
        }
    }
}

/// Represents the geometry of a road network.
///
/// `RoadGeometry` is the top-level container for the road network's geometric
/// description. It is composed of a set of `Junction`s, which in turn contain
/// `Segment`s and `Lane`s.
///
/// It provides access to the entire road network's geometric structure,
/// allowing for queries about its components (e.g., finding a `Lane` by its ID)
/// and for coordinate conversions between the inertial frame and the road network's
/// intrinsic coordinate systems (lane coordinates).
///
/// An instance of `RoadGeometry` is typically obtained from a `RoadNetwork`.
///
/// More info can be found at https://maliput.readthedocs.io/en/latest/html/deps/maliput/html/maliput_design.html.
///
/// # Example of obtaining a `RoadGeometry`
///
/// ```rust, no_run
/// use maliput::api::RoadNetwork;
/// use std::collections::HashMap;
///
/// let package_location = std::env::var("CARGO_MANIFEST_DIR").unwrap();
/// let xodr_path = format!("{}/data/xodr/TShapeRoad.xodr", package_location);
/// let road_network_properties = HashMap::from([("road_geometry_id", "my_rg_from_rust"), ("opendrive_file", xodr_path.as_str())]);
/// let road_network = RoadNetwork::new("maliput_malidrive", &road_network_properties).unwrap();
/// let road_geometry = road_network.road_geometry();
/// println!("RoadGeometry ID: {}", road_geometry.id());
/// ```
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
    /// Determines the [RoadPosition] on the 3D road manifold that corresponds to
    /// [InertialPosition] `inertial_position`.
    ///
    /// The [RoadGeometry]'s manifold is a 3D volume, with each point defined by (s, r, h)
    /// coordinates. This method returns a [RoadPositionQuery]. Its [RoadPosition] is the
    /// point in the [RoadGeometry]'s manifold which is, in the `Inertial`-frame, closest to
    /// `inertial_position`. Its InertialPosition is the `Inertial`-frame equivalent of the
    /// [RoadPosition] and its distance is the Cartesian distance from
    /// `inertial_position` to the nearest point.
    ///
    /// This method guarantees that its result satisfies the condition that
    /// `result.lane.to_lane_position(result.pos)` is within `linear_tolerance()`
    /// of the returned [InertialPosition].
    ///
    /// The map from [RoadGeometry] to the `Inertial`-frame is not onto (as a bounded
    /// [RoadGeometry] cannot completely cover the unbounded Cartesian universe).
    /// If `inertial_position` does represent a point contained within the volume
    /// of the RoadGeometry, then result distance is guaranteed to be less
    /// than or equal to `linear_tolerance()`.
    ///
    /// The map from [RoadGeometry] to `Inertial`-frame is not necessarily one-to-one.
    /// Different `(s,r,h)` coordinates from different Lanes, potentially from
    /// different Segments, may map to the same `(x,y,z)` `Inertial`-frame location.
    ///
    /// If `inertial_position` is contained within the volumes of multiple Segments,
    /// then to_road_position() will choose a [Segment] which yields the minimum
    /// height `h` value in the result.  If the chosen [Segment] has multiple
    /// Lanes, then to_road_position() will choose a [Lane] which contains
    /// `inertial_position` within its `lane_bounds()` if possible, and if that is
    /// still ambiguous, it will further select a [Lane] which minimizes the
    /// absolute value of the lateral `r` coordinate in the result.
    ///
    /// # Arguments
    /// * `inertial_position` - The [InertialPosition] to convert into a [RoadPosition].
    ///
    /// # Return
    /// A [RoadPositionQuery] with the nearest [RoadPosition], the corresponding [InertialPosition]
    /// to that [RoadPosition] and the distance between the input and output [InertialPosition]s.
    pub fn to_road_position(&self, inertial_position: &InertialPosition) -> Result<RoadPositionQuery, MaliputError> {
        let rpr = maliput_sys::api::ffi::RoadGeometry_ToRoadPosition(self.rg, &inertial_position.ip)?;
        Ok(RoadPositionQuery {
            road_position: RoadPosition {
                rp: maliput_sys::api::ffi::RoadPositionResult_road_position(&rpr),
            },
            nearest_position: InertialPosition {
                ip: maliput_sys::api::ffi::RoadPositionResult_nearest_position(&rpr),
            },
            distance: maliput_sys::api::ffi::RoadPositionResult_distance(&rpr),
        })
    }

    /// Determines the [RoadPosition] on the road surface that corresponds to
    /// [InertialPosition] `inertial_position`.
    ///
    /// This method is similar to [RoadGeometry::to_road_position], in a way that it determines if
    /// `inertial_position` is within the [RoadGeometry]'s 3D volume. If it is, a [RoadPosition] is
    /// returned where the height `h` is set to 0, effectively placing the point on the road
    /// surface.
    ///
    /// # Arguments
    /// * `inertial_position` - The [InertialPosition] to convert into a [RoadPosition].
    ///
    /// # Return
    /// The corresponding [RoadPosition] on the road surface or a [MaliputError] if the `inertial_position` is not on the road surface (i.e., the distance is greater than `linear_tolerance`).
    ///
    /// # Example
    ///
    /// ```rust, no_run
    /// use maliput::api::{RoadNetwork, InertialPosition};
    /// use std::collections::HashMap;
    ///
    /// let package_location = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    /// let xodr_path = format!("{}/data/xodr/TShapeRoad.xodr", package_location);
    /// let road_network_properties = HashMap::from([("road_geometry_id", "my_rg_from_rust"), ("opendrive_file", xodr_path.as_str())]);
    /// let road_network = RoadNetwork::new("maliput_malidrive", &road_network_properties).unwrap();
    /// let road_geometry = road_network.road_geometry();
    ///
    /// // Although this isn't directly on the surface, it is within the RoadGeometry volume.
    /// let inertial_pos = InertialPosition::new(1.0, 0.0, 1.0);
    /// let road_pos = road_geometry.to_road_position_surface(&inertial_pos);
    /// assert!(road_pos.is_ok());
    /// let road_pos = road_pos.unwrap();
    /// println!("Road position on surface: s={}, r={}, h={}", road_pos.pos().s(), road_pos.pos().r(), road_pos.pos().h());
    /// assert_eq!(road_pos.pos().s(), 1.0);
    /// assert_eq!(road_pos.pos().r(), 0.0);
    /// assert_eq!(road_pos.pos().h(), 0.0);  // h is set to 0.
    ///
    /// // An inertial position that is off the road volume.
    /// let inertial_pos= InertialPosition::new(1.0, 0.0, 10.0);
    /// let road_pos= road_geometry.to_road_position_surface(&inertial_pos);
    /// assert!(road_pos.is_err());
    /// ```
    pub fn to_road_position_surface(&self, inertial_position: &InertialPosition) -> Result<RoadPosition, MaliputError> {
        let rpr = maliput_sys::api::ffi::RoadGeometry_ToRoadPosition(self.rg, &inertial_position.ip)?;
        let road_position = RoadPosition {
            rp: maliput_sys::api::ffi::RoadPositionResult_road_position(&rpr),
        };

        let distance = maliput_sys::api::ffi::RoadPositionResult_distance(&rpr);
        if distance > self.linear_tolerance() {
            return Err(MaliputError::Other(format!(
                "InertialPosition {} does not correspond to a RoadPosition. It is off by {}m to the closest lane {} at {}.",
                maliput_sys::api::ffi::InertialPosition_to_str(&inertial_position.ip),
                distance, road_position.lane().id(), road_position.pos())));
        }

        let lane_position =
            maliput_sys::api::ffi::LanePosition_new(road_position.pos().s(), road_position.pos().r(), 0.);
        unsafe {
            Ok(RoadPosition {
                rp: maliput_sys::api::ffi::RoadPosition_new(road_position.lane().lane, &lane_position),
            })
        }
    }

    /// Obtains all [RoadPosition]s within a radius of the inertial_position.
    ///
    /// Only Lanes whose segment regions include points that are within radius of
    /// inertial position are included in the search. For each of these Lanes,
    /// include the [RoadPosition] or [RoadPosition]s with the minimum distance to
    /// inertial position in the returned result.
    ///
    /// # Arguments
    ///
    /// * `inertial_position` - The [InertialPosition] to search around.
    /// * `radius` - The radius around the [InertialPosition] to search for [RoadPosition]s.
    ///
    /// # Return
    ///
    /// A vector of [RoadPositionQuery]s.
    pub fn find_road_positions(
        &self,
        inertial_position: &InertialPosition,
        radius: f64,
    ) -> Result<Vec<RoadPositionQuery>, MaliputError> {
        let positions = maliput_sys::api::ffi::RoadGeometry_FindRoadPositions(self.rg, &inertial_position.ip, radius)?;
        Ok(positions
            .iter()
            .map(|rpr| RoadPositionQuery {
                road_position: RoadPosition {
                    rp: maliput_sys::api::ffi::RoadPositionResult_road_position(rpr),
                },
                nearest_position: InertialPosition {
                    ip: maliput_sys::api::ffi::RoadPositionResult_nearest_position(rpr),
                },
                distance: maliput_sys::api::ffi::RoadPositionResult_distance(rpr),
            })
            .collect())
    }

    /// Get the lane matching given `lane_id`.
    ///
    /// # Arguments
    /// * `lane_id` - The id of the lane.
    ///
    /// # Return
    /// The lane with the given id.
    /// If no lane is found with the given id, return None.
    pub fn get_lane(&self, lane_id: &String) -> Option<Lane<'_>> {
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
    /// let road_network = RoadNetwork::new("maliput_malidrive", &road_network_properties).unwrap();
    /// let road_geometry = road_network.road_geometry();
    /// let lanes = road_geometry.get_lanes();
    /// for lane in lanes {
    ///    println!("lane_id: {}", lane.id());
    /// }
    /// ```
    pub fn get_lanes(&self) -> Vec<Lane<'_>> {
        let lanes = maliput_sys::api::ffi::RoadGeometry_GetLanes(self.rg);
        lanes
            .into_iter()
            .map(|l| Lane {
                lane: unsafe { l.lane.as_ref().expect("") },
            })
            .collect::<Vec<Lane>>()
    }
    /// Get the segment matching given `segment_id`.
    ///
    /// # Arguments
    /// * `segment_id` - The id of the segment.
    ///
    /// # Return
    /// The segment with the given id.
    /// If no segment is found with the given id, return None.
    pub fn get_segment(&self, segment_id: &String) -> Option<Segment<'_>> {
        let segment = maliput_sys::api::ffi::RoadGeometry_GetSegment(self.rg, segment_id);
        if segment.is_null() {
            return None;
        }
        unsafe {
            Some(Segment {
                segment: segment.as_ref().expect(""),
            })
        }
    }
    /// Get the junction at the given index.
    /// The index is in the range [0, num_junctions).
    ///
    /// # Arguments
    /// * `index` - The index of the junction.
    ///
    /// # Return
    /// The junction at the given index.
    /// If no junction is found at the given index, return None.
    pub fn junction(&self, index: i32) -> Option<Junction<'_>> {
        let junction = self.rg.junction(index).ok()?;
        if junction.is_null() {
            return None;
        }
        unsafe {
            Some(Junction {
                junction: junction.as_ref().expect(""),
            })
        }
    }

    /// Get the junction matching given `junction_id`.
    ///
    /// # Arguments
    /// * `junction_id` - The id of the junction.
    ///
    /// # Return
    /// The junction with the given id.
    /// If no junction is found with the given id, return None.
    pub fn get_junction(&self, junction_id: &String) -> Option<Junction<'_>> {
        let junction = maliput_sys::api::ffi::RoadGeometry_GetJunction(self.rg, junction_id);
        if junction.is_null() {
            return None;
        }
        unsafe {
            Some(Junction {
                junction: junction.as_ref().expect(""),
            })
        }
    }
    /// Get the branch point matching given `branch_point_id`.
    ///
    /// # Arguments
    /// * `branch_point_id` - The id of the branch point.
    ///
    /// # Return
    /// The branch point with the given id.
    /// If no branch point is found with the given id, return None.
    pub fn get_branch_point(&self, branch_point_id: &String) -> Option<BranchPoint<'_>> {
        let branch_point = maliput_sys::api::ffi::RoadGeometry_GetBranchPoint(self.rg, branch_point_id);
        if branch_point.is_null() {
            return None;
        }
        unsafe {
            Some(BranchPoint {
                branch_point: branch_point.as_ref().expect(""),
            })
        }
    }
    /// Execute a custom command on the backend.
    ///
    /// # Details
    /// The documentation of the custom command should be provided by the backend: https://github.com/maliput/maliput_malidrive/blob/main/src/maliput_malidrive/base/road_geometry.h
    ///
    /// # Arguments
    /// * `command` - The command to execute.
    ///
    /// # Return
    /// The result of the command.
    // pub fn backend_custom_command(&self, command: &String) -> String {
    pub fn backend_custom_command(&self, command: &String) -> Result<String, MaliputError> {
        Ok(maliput_sys::api::ffi::RoadGeometry_BackendCustomCommand(
            self.rg, command,
        )?)
    }
    /// Obtains the Geo Reference info of this RoadGeometry.
    ///
    /// # Return
    /// A string containing the Geo Reference projection, if any.
    pub fn geo_reference_info(&self) -> String {
        maliput_sys::api::ffi::RoadGeometry_GeoReferenceInfo(self.rg)
    }
}

/// A 3-dimensional position in a `Lane`-frame, consisting of three components:
///
/// * s is longitudinal position, as arc-length along a Lane's reference line.
/// * r is lateral position, perpendicular to the reference line at s. +r is to
///   to the left when traveling in the direction of +s.
/// * h is height above the road surface.
///
/// # Example
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

/// A position in 3-dimensional geographical Cartesian space, i.e., in the
/// `Inertial`-frame, consisting of three components x, y, and z.
///
/// # Example
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

impl Clone for InertialPosition {
    fn clone(&self) -> Self {
        InertialPosition {
            ip: maliput_sys::api::ffi::InertialPosition_new(self.x(), self.y(), self.z()),
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
    /// Create a new `RBounds` with the given `min` and `max` values.
    pub fn new(min: f64, max: f64) -> RBounds {
        RBounds { min, max }
    }
    /// Get the `min` value of the `RBounds`.
    pub fn min(&self) -> f64 {
        self.min
    }
    /// Get the `max` value of the `RBounds`.
    pub fn max(&self) -> f64 {
        self.max
    }
    /// Set the `min` value of the `RBounds`.
    pub fn set_min(&mut self, min: f64) {
        self.min = min;
    }
    /// Set the `max` value of the `RBounds`.
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
    /// Create a new `HBounds` with the given `min` and `max` values.
    pub fn new(min: f64, max: f64) -> HBounds {
        HBounds { min, max }
    }
    /// Get the `min` value of the `HBounds`.
    pub fn min(&self) -> f64 {
        self.min
    }
    /// Get the `max` value of the `HBounds`.
    pub fn max(&self) -> f64 {
        self.max
    }
    /// Set the `min` value of the `HBounds`.
    pub fn set_min(&mut self, min: f64) {
        self.min = min;
    }
    /// Set the `max` value of the `HBounds`.
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

/// Lane classification options.
///
/// LaneType defines the intended use of a lane.
#[derive(strum_macros::Display, Debug, Copy, Clone, PartialEq, Eq)]
pub enum LaneType {
    /// Default state.
    Unknown,
    /// Standard driving lane.
    Driving,
    /// Turn available.
    Turn,
    /// High Occupancy Vehicle lane (+2 passengers).
    Hov,
    /// Bus only.
    Bus,
    /// Taxi only.
    Taxi,
    /// Emergency vehicles only (fire, ambulance, police).
    Emergency,
    /// Soft border at the edge of the road.
    Shoulder,
    /// Reserved for cyclists.
    Biking,
    /// Sidewalks / Crosswalks.
    Walking,
    /// Lane with parking spaces.
    Parking,
    /// Hard shoulder / Emergency stop.
    Stop,
    /// Hard border at the edge of the road.
    Border,
    /// Curb stones.
    Curb,
    /// Sits between driving lanes that lead in opposite directions.
    Median,
    /// Generic restricted (use if HOV/Bus/Emergency don't fit).
    Restricted,
    /// Road works.
    Construction,
    /// Trains/Trams.
    Rail,
    /// Merge into main road.
    Entry,
    /// Exit from the main road.
    Exit,
    /// Ramp leading to a motorway.
    OnRamp,
    /// Ramp leading away from a motorway.
    OffRamp,
    // Ramp that connects two motorways.
    ConnectingRamp,
    /// Change roads without driving into the main intersection.
    SlipLane,
    /// Intersection crossings with no physical markings.
    Virtual,
}

/// A Lane represents a lane of travel in a road network.  A Lane defines
/// a curvilinear coordinate system covering the road surface, with a
/// longitudinal 's' coordinate that expresses the arc-length along a
/// central reference curve.  The reference curve nominally represents
/// an ideal travel trajectory along the Lane.
///
/// Lanes are grouped by Segment.  All Lanes belonging to a Segment
/// represent the same road surface, but with different coordinate
/// parameterizations (e.g., each Lane has its own reference curve).
pub struct Lane<'a> {
    lane: &'a maliput_sys::api::ffi::Lane,
}

impl<'a> Lane<'a> {
    /// Returns the id of the Lane.
    ///
    /// The id is a unique identifier for the Lane within the RoadGeometry.
    ///
    /// # Returns
    /// A `String` containing the id of the Lane.
    pub fn id(&self) -> String {
        maliput_sys::api::ffi::Lane_id(self.lane)
    }
    /// Returns the index of this Lane within the Segment which owns it.
    pub fn index(&self) -> i32 {
        self.lane.index()
    }
    /// Returns the Segment to which this Lane belongs.
    ///
    /// # Returns
    /// A [`Segment`] containing the Segment to which this Lane belongs.
    pub fn segment(&self) -> Segment<'a> {
        unsafe {
            Segment {
                segment: self.lane.segment().as_ref().expect(""),
            }
        }
    }
    /// Returns the adjacent lane to the left, if one exists.
    ///
    /// "Left" is defined as the direction of `+r` in the `(s, r, h)` lane-coordinate frame.
    ///
    /// # Returns
    ///
    /// An [`Option<Lane>`] containing the left lane, or `None` if this is the
    /// leftmost lane.
    pub fn to_left(&self) -> Option<Lane<'_>> {
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
    /// Returns the adjacent lane to the right, if one exists.
    ///
    /// "Right" is defined as the direction of `-r` in the `(s, r, h)` lane-coordinate
    /// frame.
    ///
    /// # Returns
    ///
    /// An [`Option<Lane>`] containing the right lane, or `None` if this is the
    /// rightmost lane.
    pub fn to_right(&self) -> Option<Lane<'_>> {
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
    /// Returns the arc-length of the Lane along its reference curve.
    ///
    /// The value of length() is also the maximum s-coordinate for this Lane;
    /// i.e., the domain of s is [0, length()].
    ///
    /// # Returns
    ///
    /// The length of the Lane in meters.
    pub fn length(&self) -> f64 {
        self.lane.length()
    }

    /// Get the orientation of the `Lane` at the given `LanePosition`.
    pub fn get_orientation(&self, lane_position: &LanePosition) -> Result<Rotation, MaliputError> {
        Ok(Rotation {
            r: maliput_sys::api::ffi::Lane_GetOrientation(self.lane, lane_position.lp.as_ref().expect(""))?,
        })
    }
    /// # Brief
    /// Get the [InertialPosition] of the [Lane] at the given [LanePosition].
    ///
    /// # Notes
    /// Note there is no constraint for the `r` coordinate, as it can be outside the lane boundaries.
    /// In that scenario, the resultant inertial position represents a point in the `s-r` plane at the given `s` and `h`
    /// coordinates. It's on the user side to verify, if needed, that the lane position is within lane boundaries.
    /// Bare in mind that the inertial position will be a point in the `s-r` plane, but *not* necessarily on the road surface.
    ///
    /// # Arguments
    /// * `lane_position` - A maliput [LanePosition].
    ///
    /// # Precondition
    /// The s component of `lane_position` must be in domain [0, Lane::length()].
    ///
    /// # Return
    /// The [InertialPosition] corresponding to the input [LanePosition].
    pub fn to_inertial_position(&self, lane_position: &LanePosition) -> Result<InertialPosition, MaliputError> {
        Ok(InertialPosition {
            ip: maliput_sys::api::ffi::Lane_ToInertialPosition(self.lane, lane_position.lp.as_ref().expect(""))?,
        })
    }
    /// Determines the LanePosition corresponding to InertialPosition `inertial_position`.
    /// The LanePosition is expected to be contained within the lane's 3D boundaries (s, r, h).
    /// See [Lane::to_segment_position] method.
    ///
    /// This method guarantees that its result satisfies the condition that
    /// `to_inertial_position(result.lane_position)` is within `linear_tolerance()`
    ///  of `result.nearest_position`.
    ///
    /// # Arguments
    /// * `inertial_position` - A [InertialPosition] to get a [LanePosition] from.
    ///
    /// # Return
    /// A [LanePositionQuery] with the closest [LanePosition], the corresponding [InertialPosition] to that [LanePosition]
    /// and the distance between the input and output [InertialPosition]s.
    pub fn to_lane_position(&self, inertial_position: &InertialPosition) -> Result<LanePositionQuery, MaliputError> {
        let lpr = maliput_sys::api::ffi::Lane_ToLanePosition(self.lane, inertial_position.ip.as_ref().expect(""))?;
        Ok(LanePositionQuery {
            lane_position: LanePosition {
                lp: maliput_sys::api::ffi::LanePositionResult_road_position(&lpr),
            },
            nearest_position: InertialPosition {
                ip: maliput_sys::api::ffi::LanePositionResult_nearest_position(&lpr),
            },
            distance: maliput_sys::api::ffi::LanePositionResult_distance(&lpr),
        })
    }
    /// Determines the [LanePosition] corresponding to [InertialPosition] `inertial_position`.
    /// The [LanePosition] is expected to be contained within the segment's 3D boundaries (s, r, h).
    /// See [Lane::to_lane_position] method.
    ///
    /// This method guarantees that its result satisfies the condition that
    /// `to_inertial_position(result.lane_position)` is within `linear_tolerance()`
    ///  of `result.nearest_position`.
    ///
    /// # Arguments
    /// * `inertial_position` - A [InertialPosition] to get a SegmentPosition from.
    ///
    /// # Return
    /// A [LanePositionQuery] with the closest [LanePosition] within the segment, the corresponding
    /// [InertialPosition] to that [LanePosition] and the distance between the input and output
    /// [InertialPosition]s.
    pub fn to_segment_position(&self, inertial_position: &InertialPosition) -> Result<LanePositionQuery, MaliputError> {
        let spr = maliput_sys::api::ffi::Lane_ToSegmentPosition(self.lane, inertial_position.ip.as_ref().expect(""))?;
        Ok(LanePositionQuery {
            lane_position: LanePosition {
                lp: maliput_sys::api::ffi::LanePositionResult_road_position(&spr),
            },
            nearest_position: InertialPosition {
                ip: maliput_sys::api::ffi::LanePositionResult_nearest_position(&spr),
            },
            distance: maliput_sys::api::ffi::LanePositionResult_distance(&spr),
        })
    }
    /// Returns the nominal lateral (r) bounds for the lane as a function of s.
    ///
    /// These are the lateral bounds for a position that is considered to be
    /// "staying in the lane".
    ///
    /// See also [Lane::segment_bounds] that defines the whole surface.
    ///
    /// # Arguments
    /// * `s` - The longitudinal position along the lane's reference line.
    ///
    /// # Returns
    /// A [RBounds] containing the lateral bounds of the lane at the given `s.
    ///
    /// # Errors
    /// If lane bounds cannot be computed, an error is returned. This can happen if the
    /// `s` value is out of bounds (i.e., not in the range [0, Lane::length()]).
    pub fn lane_bounds(&self, s: f64) -> Result<RBounds, MaliputError> {
        let bounds = maliput_sys::api::ffi::Lane_lane_bounds(self.lane, s)?;
        Ok(RBounds::new(bounds.min(), bounds.max()))
    }
    /// Returns the lateral segment (r) bounds of the lane as a function of s.
    ///
    /// These are the lateral bounds for a position that is considered to be
    /// "on segment", reflecting the physical extent of the surface of the
    /// lane's segment.
    ///
    /// See also [Lane::lane_bounds] that defines what's considered to be "staying
    /// in the lane".
    ///
    /// # Arguments
    /// * `s` - The longitudinal position along the lane's reference line.
    ///
    /// # Returns
    /// A [RBounds] containing the lateral segment bounds of the lane at the given `
    /// s`.
    ///
    /// # Errors
    /// If segment bounds cannot be computed, an error is returned. This can happen if the
    /// `s` value is out of bounds (i.e., not in the range [0, Lane::length()]).
    pub fn segment_bounds(&self, s: f64) -> Result<RBounds, MaliputError> {
        let bounds = maliput_sys::api::ffi::Lane_segment_bounds(self.lane, s)?;
        Ok(RBounds::new(bounds.min(), bounds.max()))
    }
    /// Returns the elevation (`h`) bounds of the lane as a function of `(s, r)`.
    ///
    /// These are the elevation bounds for a position that is considered to be
    /// within the Lane's volume modeled by the RoadGeometry.
    ///
    /// `s` is within [0, `length()`] of this Lane and `r` is within
    /// `lane_bounds(s)`.
    ///
    /// # Arguments
    /// * `s` - The longitudinal position along the lane's reference line.
    /// * `r` - The lateral position perpendicular to the reference line at `s`.
    ///
    /// # Returns
    /// A [HBounds] containing the elevation bounds of the lane at the given `(s, r)`.
    ///
    /// # Errors
    /// If elevation bounds cannot be computed, an error is returned. This can happen if the
    /// `s` value is out of bounds (i.e., not in the range [0, Lane::length()]) or if `r` is not within the
    /// lane bounds at `s`.
    pub fn elevation_bounds(&self, s: f64, r: f64) -> Result<HBounds, MaliputError> {
        let bounds = maliput_sys::api::ffi::Lane_elevation_bounds(self.lane, s, r)?;
        Ok(HBounds::new(bounds.min(), bounds.max()))
    }
    /// Computes derivatives of [LanePosition] given a velocity vector `velocity`.
    /// `velocity` is a isometric velocity vector oriented in the `Lane`-frame
    /// at `position`.
    ///
    /// # Arguments
    /// * `lane_position` - A [LanePosition] at which to evaluate the derivatives.
    /// * `velocity` - An [IsoLaneVelocity] representing the velocity vector in the `Lane`-frame
    ///   at `lane_position`.
    ///
    /// # Returns
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
    /// Returns the lane's [BranchPoint] for the specified end.
    ///
    /// # Argument
    /// * `end` - This lane's start or end [LaneEnd].
    ///
    /// # Return
    /// The lane's [BranchPoint] for the specified end.
    pub fn get_branch_point(&self, end: &LaneEnd) -> Result<BranchPoint<'_>, MaliputError> {
        if end != &LaneEnd::Start(self.clone()) && end != &LaneEnd::Finish(self.clone()) {
            return Err(MaliputError::AssertionError(format!(
                "LaneEnd must be an end of this lane {:?}",
                end
            )));
        }
        Ok(BranchPoint {
            branch_point: unsafe {
                maliput_sys::api::ffi::Lane_GetBranchPoint(self.lane, end == &LaneEnd::Start(self.clone()))
                    .as_ref()
                    .expect("Underlying BranchPoint is null")
            },
        })
    }
    /// Returns the set of [LaneEnd]'s which connect with this lane on the
    /// same side of the [BranchPoint] at `end`. At a minimum,
    /// this set will include this [Lane].
    ///
    /// # Arguments
    /// * `end` - This lane's start or end [LaneEnd].
    ///
    /// # Return
    /// A [LaneEndSet] with all the [LaneEnd]s at the same side of the [BranchPoint] at `end`.
    pub fn get_confluent_branches(&self, end: &LaneEnd) -> Result<LaneEndSet<'_>, MaliputError> {
        if end != &LaneEnd::Start(self.clone()) && end != &LaneEnd::Finish(self.clone()) {
            return Err(MaliputError::AssertionError(format!(
                "LaneEnd must be an end of this lane {:?}",
                end
            )));
        }
        Ok(LaneEndSet {
            lane_end_set: unsafe {
                maliput_sys::api::ffi::Lane_GetConfluentBranches(self.lane, end == &LaneEnd::Start(self.clone()))?
                    .as_ref()
                    .expect("Underlying LaneEndSet is null")
            },
        })
    }
    /// Returns the set of [LaneEnd]s which continue onward from this lane at the
    /// [BranchPoint] at `end`.
    ///
    /// # Arguments
    /// * `end` - This lane's start or end [LaneEnd].
    ///
    /// # Return
    /// A [LaneEndSet] with all the [LaneEnd]s at the opposite side of the [BranchPoint] at `end`.
    pub fn get_ongoing_branches(&self, end: &LaneEnd) -> Result<LaneEndSet<'_>, MaliputError> {
        if end != &LaneEnd::Start(self.clone()) && end != &LaneEnd::Finish(self.clone()) {
            return Err(MaliputError::AssertionError(format!(
                "LaneEnd must be an end of this lane {:?}",
                end
            )));
        }
        Ok(LaneEndSet {
            lane_end_set: unsafe {
                maliput_sys::api::ffi::Lane_GetOngoingBranches(self.lane, end == &LaneEnd::Start(self.clone()))?
                    .as_ref()
                    .expect("Underlying LaneEndSet is null")
            },
        })
    }
    /// Returns the default ongoing LaneEnd connected at `end`,
    /// or None if no default branch has been established.
    ///
    /// # Arguments
    /// * `end` - This lane's start or end [LaneEnd].
    ///
    /// # Return
    /// An `Option<LaneEnd>` containing the default branch if it exists, or None
    /// if no default branch has been established.
    pub fn get_default_branch(&self, end: &LaneEnd) -> Option<LaneEnd<'_>> {
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
    /// Evaluates if the `Lane` contains the given `LanePosition`.
    ///
    /// # Arguments
    /// * `lane_position` - A [LanePosition] to check if it is contained within the `Lane`.
    ///
    /// # Returns
    /// A boolean indicating whether the `Lane` contains the `LanePosition`.
    pub fn contains(&self, lane_position: &LanePosition) -> bool {
        self.lane.Contains(lane_position.lp.as_ref().expect(""))
    }
    /// Returns the [LaneType] of the [Lane].
    ///
    /// # Returns
    /// The [LaneType] of the [Lane].
    pub fn lane_type(&self) -> LaneType {
        let lane_type = maliput_sys::api::ffi::Lane_type(self.lane);
        match lane_type {
            maliput_sys::api::ffi::LaneType::kDriving => LaneType::Driving,
            maliput_sys::api::ffi::LaneType::kTurn => LaneType::Turn,
            maliput_sys::api::ffi::LaneType::kHov => LaneType::Hov,
            maliput_sys::api::ffi::LaneType::kBus => LaneType::Bus,
            maliput_sys::api::ffi::LaneType::kTaxi => LaneType::Taxi,
            maliput_sys::api::ffi::LaneType::kEmergency => LaneType::Emergency,
            maliput_sys::api::ffi::LaneType::kShoulder => LaneType::Shoulder,
            maliput_sys::api::ffi::LaneType::kBiking => LaneType::Biking,
            maliput_sys::api::ffi::LaneType::kWalking => LaneType::Walking,
            maliput_sys::api::ffi::LaneType::kParking => LaneType::Parking,
            maliput_sys::api::ffi::LaneType::kStop => LaneType::Stop,
            maliput_sys::api::ffi::LaneType::kBorder => LaneType::Border,
            maliput_sys::api::ffi::LaneType::kCurb => LaneType::Curb,
            maliput_sys::api::ffi::LaneType::kMedian => LaneType::Median,
            maliput_sys::api::ffi::LaneType::kRestricted => LaneType::Restricted,
            maliput_sys::api::ffi::LaneType::kConstruction => LaneType::Construction,
            maliput_sys::api::ffi::LaneType::kRail => LaneType::Rail,
            maliput_sys::api::ffi::LaneType::kEntry => LaneType::Entry,
            maliput_sys::api::ffi::LaneType::kExit => LaneType::Exit,
            maliput_sys::api::ffi::LaneType::kOnRamp => LaneType::OnRamp,
            maliput_sys::api::ffi::LaneType::kOffRamp => LaneType::OffRamp,
            maliput_sys::api::ffi::LaneType::kConnectingRamp => LaneType::ConnectingRamp,
            maliput_sys::api::ffi::LaneType::kSlipLane => LaneType::SlipLane,
            maliput_sys::api::ffi::LaneType::kVirtual => LaneType::Virtual,
            _ => LaneType::Unknown,
        }
    }

    /// Returns the boundary at the left of the lane.
    pub fn left_boundary(&self) -> Result<LaneBoundary<'_>, MaliputError> {
        let lane_boundary = self.lane.left_boundary()?;
        if lane_boundary.is_null() {
            return Err(MaliputError::AssertionError(
                "Lane does not have a left boundary".to_string(),
            ));
        }

        Ok(LaneBoundary {
            lane_boundary: unsafe { lane_boundary.as_ref().expect("") },
        })
    }

    /// Returns the boundary at the right of the lane.
    pub fn right_boundary(&self) -> Result<LaneBoundary<'_>, MaliputError> {
        let lane_boundary = self.lane.right_boundary()?;
        if lane_boundary.is_null() {
            return Err(MaliputError::AssertionError(
                "Lane does not have a right boundary".to_string(),
            ));
        }

        Ok(LaneBoundary {
            lane_boundary: unsafe { lane_boundary.as_ref().expect("") },
        })
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
pub struct Segment<'a> {
    segment: &'a maliput_sys::api::ffi::Segment,
}

impl<'a> Segment<'a> {
    /// Returns the id of the Segment.
    /// The id is a unique identifier for the Segment within the RoadGeometry.
    ///
    /// # Returns
    /// A `String` containing the id of the Segment.
    pub fn id(&self) -> String {
        maliput_sys::api::ffi::Segment_id(self.segment)
    }
    /// Returns the [Junction] to which this Segment belongs.
    ///
    /// # Returns
    /// An [`Result<Junction, MaliputError>`] containing the Junction to which this Segment belongs.
    /// If the Segment does not belong to a Junction, an error is returned.
    pub fn junction(&self) -> Result<Junction<'_>, MaliputError> {
        let junction = self.segment.junction()?;
        if junction.is_null() {
            return Err(MaliputError::AssertionError(
                "Segment does not belong to a Junction".to_string(),
            ));
        }
        unsafe {
            Ok(Junction {
                junction: junction.as_ref().expect(""),
            })
        }
    }
    /// Returns the number of lanes in the Segment.
    ///
    /// # Returns
    /// The number of lanes in the Segment.
    pub fn num_lanes(&self) -> i32 {
        self.segment.num_lanes()
    }
    /// Returns the lane at the given `index`.
    ///
    /// # Arguments
    /// * `index` - The index of the lane to retrieve.
    ///
    /// # Returns
    /// A [`Lane`] containing the lane at the given index.
    pub fn lane(&self, index: i32) -> Result<Lane<'_>, MaliputError> {
        if index < 0 || index >= self.num_lanes() {
            return Err(MaliputError::AssertionError(format!(
                "Index {} is out of bounds for Segment with {} lanes",
                index,
                self.num_lanes()
            )));
        }
        let lane = self.segment.lane(index)?;
        unsafe {
            Ok(Lane {
                lane: lane.as_ref().expect(""),
            })
        }
    }

    /// Returns the amount of boundaries in the segment.
    pub fn num_boundaries(&self) -> i32 {
        self.segment.num_boundaries()
    }

    /// Returns the [LaneBoundary] that matches the index.
    ///
    /// # Arguments
    /// * `index` - The index of the boundary. See [LaneBoundary] for more info about indexes.
    ///
    /// # Returns
    /// The [LaneBoundary] at the given index.
    pub fn boundary(&self, index: i32) -> Result<LaneBoundary<'_>, MaliputError> {
        let lane_boundary = self.segment.boundary(index)?;
        if lane_boundary.is_null() {
            return Err(MaliputError::AssertionError(
                "Segment does not have a boundary".to_string(),
            ));
        }
        Ok(LaneBoundary {
            lane_boundary: unsafe { lane_boundary.as_ref().expect("") },
        })
    }
}

/// A Junction is a closed set of [Segment]s which have physically
/// coplanar road surfaces, in the sense that [RoadPosition]s with the
/// same h value (height above surface) in the domains of two [Segment]s
/// map to the same [InertialPosition].  The [Segment]s need not be directly
/// connected to one another in the network topology.
///
/// Junctions are grouped by [RoadGeometry].
pub struct Junction<'a> {
    junction: &'a maliput_sys::api::ffi::Junction,
}

impl<'a> Junction<'a> {
    /// Returns the id of the Junction.
    /// The id is a unique identifier for the Junction within the RoadGeometry.
    ///
    /// # Returns
    /// A `String` containing the id of the Junction.
    pub fn id(&self) -> String {
        maliput_sys::api::ffi::Junction_id(self.junction)
    }
    /// Returns the [RoadGeometry] to which this Junction belongs.
    ///
    /// # Returns
    /// A [`RoadGeometry`] containing the RoadGeometry to which this Junction belongs.
    pub fn road_geometry(&self) -> RoadGeometry<'_> {
        unsafe {
            RoadGeometry {
                rg: self.junction.road_geometry().as_ref().expect(""),
            }
        }
    }
    /// Returns the number of segments in the Junction.
    ///
    /// # Returns
    /// The number of segments in the Junction.
    pub fn num_segments(&self) -> i32 {
        self.junction.num_segments()
    }
    /// Returns the segment at the given `index`.
    ///
    /// # Arguments
    /// * `index` - The index of the segment to retrieve.
    ///
    /// # Returns
    /// A [Result<Segment, MaliputError>] containing the segment at the given index.
    /// If the index is out of bounds, an error is returned.
    pub fn segment(&self, index: i32) -> Result<Segment<'_>, MaliputError> {
        unsafe {
            Ok(Segment {
                segment: self.junction.segment(index)?.as_ref().expect(""),
            })
        }
    }
}

/// A position in the road network compound by a specific lane and a lane-frame position in that lane.
/// This position is defined by a [Lane] and a [LanePosition].
pub struct RoadPosition {
    rp: cxx::UniquePtr<maliput_sys::api::ffi::RoadPosition>,
}

impl RoadPosition {
    /// Create a new `RoadPosition` with the given `lane` and `lane_pos`.
    ///
    /// # Arguments
    /// * `lane` - A reference to a [Lane] that this `RoadPosition` is associated with.
    /// * `lane_pos` - A reference to a [LanePosition] that defines the position within the lane.
    ///
    /// # Returns
    /// A new `RoadPosition` instance.
    pub fn new(lane: &Lane, lane_pos: &LanePosition) -> RoadPosition {
        unsafe {
            RoadPosition {
                rp: maliput_sys::api::ffi::RoadPosition_new(lane.lane, &lane_pos.lp),
            }
        }
    }
    /// Computes the [InertialPosition] corresponding to this `RoadPosition`.
    ///
    /// # Notes
    /// This is an indirection to [Lane::to_inertial_position] method.
    ///
    /// # Returns
    /// An [InertialPosition] corresponding to this `RoadPosition`.
    pub fn to_inertial_position(&self) -> Result<InertialPosition, MaliputError> {
        Ok(InertialPosition {
            ip: maliput_sys::api::ffi::RoadPosition_ToInertialPosition(&self.rp)?,
        })
    }
    /// Gets the [Lane] associated with this `RoadPosition`.
    ///
    /// # Returns
    /// A [Lane] that this `RoadPosition` is associated with.
    pub fn lane(&self) -> Lane<'_> {
        unsafe {
            Lane {
                lane: maliput_sys::api::ffi::RoadPosition_lane(&self.rp).as_ref().expect(""),
            }
        }
    }
    /// Gets the [LanePosition] associated with this `RoadPosition`.
    ///
    /// # Returns
    /// A [LanePosition] that defines the position within the lane for this `RoadPosition`.
    pub fn pos(&self) -> LanePosition {
        LanePosition {
            lp: maliput_sys::api::ffi::RoadPosition_pos(&self.rp),
        }
    }
}

/// Represents the result of a RoadPosition query.
/// This struct contains the `RoadPosition`, the nearest `InertialPosition` to that `RoadPosition`,
/// and the distance between the input `InertialPosition` and the nearest `InertialPosition`.
///
/// This struct is typically used as return type for the methods: [RoadGeometry::to_road_position] and [RoadGeometry::find_road_positions].
pub struct RoadPositionQuery {
    /// The candidate RoadPosition returned by the query.
    pub road_position: RoadPosition,
    /// The nearest InertialPosition to the candidate RoadPosition.
    /// This is the position in the inertial frame that is closest to the candidate RoadPosition
    pub nearest_position: InertialPosition,
    /// The distance between the input InertialPosition and the nearest InertialPosition.
    pub distance: f64,
}

impl RoadPositionQuery {
    /// Create a new `RoadPositionQuery` with the given `road_position`, `nearest_position`, and `distance`.
    pub fn new(road_position: RoadPosition, nearest_position: InertialPosition, distance: f64) -> RoadPositionQuery {
        RoadPositionQuery {
            road_position,
            nearest_position,
            distance,
        }
    }
}

/// Represents the result of a LanePosition query.
/// This struct contains the `LanePosition`, the nearest `InertialPosition` to that `LanePosition`,
/// and the distance between the input `InertialPosition` and the nearest `InertialPosition`.
///
/// This struct is typically used as return type for the methods: [Lane::to_lane_position] and [Lane::to_segment_position].
pub struct LanePositionQuery {
    /// The candidate LanePosition within the Lane' lane-bounds or segment-bounds
    /// depending if [Lane::to_lane_position] or [Lane::to_segment_position] respectively, was called.
    /// The LanePosition is closest to a `inertial_position` supplied to [Lane::to_lane_position]
    /// (measured by the Cartesian metric in the `Inertial`-frame).
    pub lane_position: LanePosition,
    /// The position that exactly corresponds to `lane_position`.
    pub nearest_position: InertialPosition,
    /// The Cartesian distance between `nearest_position` and the
    /// `inertial_position` supplied to [Lane::to_lane_position] / [Lane::to_segment_position].
    pub distance: f64,
}

impl LanePositionQuery {
    /// Create a new `LanePositionQuery` with the given `lane_position`, `nearest_position`, and `distance`.
    pub fn new(lane_position: LanePosition, nearest_position: InertialPosition, distance: f64) -> LanePositionQuery {
        LanePositionQuery {
            lane_position,
            nearest_position,
            distance,
        }
    }
}

/// A 3-dimensional rotation in the road network.
/// This struct represents a rotation in the road network, which can be defined
/// using a quaternion or roll-pitch-yaw angles.
/// It provides methods to create a rotation, convert between representations,
/// and apply the rotation to an inertial position.
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
pub struct SRange {
    s_range: cxx::UniquePtr<maliput_sys::api::ffi::SRange>,
}

impl SRange {
    /// Creates a new `SRange` with the given `s0` and `s1`.
    ///
    /// # Arguments
    /// * `s0` - The starting value of the range.
    /// * `s1` - The ending value of the range.
    ///
    /// # Returns
    /// A new `SRange` instance.
    pub fn new(s0: f64, s1: f64) -> SRange {
        SRange {
            s_range: maliput_sys::api::ffi::SRange_new(s0, s1),
        }
    }
    /// Returns the s0 of the `SRange`.
    ///
    /// # Returns
    /// The starting value of the range.
    pub fn s0(&self) -> f64 {
        self.s_range.s0()
    }
    /// Returns the s1 of the `SRange`.
    ///
    /// # Returns
    /// The ending value of the range.
    pub fn s1(&self) -> f64 {
        self.s_range.s1()
    }
    /// Sets the s0 of the `SRange`.
    ///
    /// # Arguments
    /// * `s0` - The new starting value of the range.
    pub fn set_s0(&mut self, s0: f64) {
        self.s_range.as_mut().expect("Underlying SRange is null").set_s0(s0);
    }
    /// Sets the s1 of the `SRange`.
    ///
    /// # Arguments
    /// * `s1` - The new ending value of the range.
    pub fn set_s1(&mut self, s1: f64) {
        self.s_range.as_mut().expect("Underlying SRange is null").set_s1(s1);
    }
    /// Get the size of the `SRange`.
    ///
    /// # Returns
    /// The size of the range, which is the difference between s1 and s0.
    pub fn size(&self) -> f64 {
        self.s_range.size()
    }
    /// Defines whether this SRange is in the direction of +s (i.e., s1() > s0()).
    ///
    /// # Returns
    /// A boolean indicating whether the SRange is in the direction of +s.
    pub fn with_s(&self) -> bool {
        self.s_range.WithS()
    }
    /// Determines whether this SRange intersects with `s_range`.
    ///
    /// # Arguments
    /// * `s_range` - Another `SRange` to check for intersection.
    /// * `tolerance` - A tolerance value to consider when checking for intersection.
    ///
    /// # Returns
    /// A boolean indicating whether this SRange intersects with `s_range`.
    pub fn intersects(&self, s_range: &SRange, tolerance: f64) -> bool {
        self.s_range.Intersects(&s_range.s_range, tolerance)
    }
    /// Determines whether this SRange contains `s_range`.
    ///
    /// # Arguments
    /// * `s_range` - Another `SRange` to check if it is contained within this SRange.
    /// * `tolerance` - A tolerance value to consider when checking for containment.
    pub fn contains(&self, s_range: &SRange, tolerance: f64) -> bool {
        self.s_range.Contains(&s_range.s_range, tolerance)
    }
    /// Get the intersection of this SRange with `s_range`.
    ///
    /// # Arguments
    /// * `s_range` - Another `SRange` to get the intersection with.
    /// * `tolerance` - A tolerance value to consider when checking for intersection.
    ///
    /// # Returns
    /// An `Option<SRange>` containing the intersection of this SRange with `s_range`.
    /// If the intersection is empty, it returns None.
    pub fn get_intersection(&self, s_range: &SRange, tolerance: f64) -> Option<SRange> {
        let intersection = maliput_sys::api::ffi::SRange_GetIntersection(&self.s_range, &s_range.s_range, tolerance);
        match intersection.is_null() {
            true => None,
            false => Some(SRange { s_range: intersection }),
        }
    }
}

impl std::fmt::Debug for SRange {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "SRange {{ s0: {}, s1: {} }}", self.s0(), self.s1())
    }
}

/// Directed longitudinal range of a specific Lane, identified by a LaneId.
/// Similar to [SRange], but associated with a specific Lane.
pub struct LaneSRange {
    pub(crate) lane_s_range: cxx::UniquePtr<maliput_sys::api::ffi::LaneSRange>,
}

impl LaneSRange {
    /// Creates a new `LaneSRange` with the given `lane_id` and `s_range`.
    /// # Arguments
    /// * `lane_id` - A `String` representing the id of the lane.
    /// * `s_range` - A reference to an [SRange] that defines the longitudinal range of the lane.
    ///
    /// # Returns
    /// A new `LaneSRange` instance.
    pub fn new(lane_id: &String, s_range: &SRange) -> LaneSRange {
        LaneSRange {
            lane_s_range: maliput_sys::api::ffi::LaneSRange_new(lane_id, &s_range.s_range),
        }
    }
    /// Returns the lane id of the `LaneSRange`.
    ///
    /// # Returns
    /// A `String` containing the id of the lane associated with this `LaneSRange`.
    pub fn lane_id(&self) -> String {
        maliput_sys::api::ffi::LaneSRange_lane_id(&self.lane_s_range)
    }
    /// Returns the [SRange] of the `LaneSRange`.
    ///
    /// # Returns
    /// An [SRange] containing the longitudinal range of the lane associated with this `LaneSRange`.
    pub fn s_range(&self) -> SRange {
        SRange {
            s_range: maliput_sys::api::ffi::LaneSRange_s_range(&self.lane_s_range),
        }
    }
    /// Returns the length of the `LaneSRange`.
    ///
    /// This is equivalent to `s_range.size()`.
    ///
    /// # Returns
    /// A `f64` representing the length of the `LaneSRange`.
    pub fn length(&self) -> f64 {
        self.lane_s_range.length()
    }
    /// Determines whether this LaneSRange intersects with `lane_s_range`.
    ///
    /// # Arguments
    /// * `lane_s_range` - Another `LaneSRange` to check for intersection.
    /// * `tolerance` - A tolerance value to consider when checking for intersection.
    ///
    /// # Returns
    /// A boolean indicating whether this LaneSRange intersects with `lane_s_range`.
    pub fn intersects(&self, lane_s_range: &LaneSRange, tolerance: f64) -> bool {
        self.lane_s_range.Intersects(&lane_s_range.lane_s_range, tolerance)
    }
    /// Determines whether this LaneSRange contains `lane_s_range`.
    ///
    /// # Arguments
    /// * `lane_s_range` - Another `LaneSRange` to check if it is contained within this LaneSRange.
    /// * `tolerance` - A tolerance value to consider when checking for containment.
    ///
    /// # Returns
    /// A boolean indicating whether this LaneSRange contains `lane_s_range`.
    /// This checks if the `s_range` of `lane_s_range` is fully contained
    /// within the `s_range` of this `LaneSRange`, considering the lane id.
    /// If the lane id does not match, it returns false.
    pub fn contains(&self, lane_s_range: &LaneSRange, tolerance: f64) -> bool {
        self.lane_s_range.Contains(&lane_s_range.lane_s_range, tolerance)
    }
    /// Computes the intersection of this `LaneSRange` with `lane_s_range`.
    ///
    /// # Arguments
    /// * `lane_s_range` - Another `LaneSRange` to get the intersection with.
    /// * `tolerance` - A tolerance value to consider when checking for intersection.
    ///
    /// # Returns
    /// An `Option<LaneSRange>` containing the intersection of this `LaneSRange` with `lane_s_range`.
    /// If the lane ids do not match, it returns None.
    /// If the intersection is empty, it returns None.
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

impl std::fmt::Debug for LaneSRange {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "LaneSRange {{ lane_id: {}, s_range: {:?} }}",
            self.lane_id(),
            self.s_range()
        )
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
    /// Creates a new `LaneSRoute` with the given `ranges`.
    ///
    /// # Arguments
    /// * `ranges` - A vector of [LaneSRange] to create the [LaneSRoute].
    ///
    /// # Returns
    /// A new `LaneSRoute` instance containing the provided ranges.
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
    ///
    /// # Returns
    /// A vector of [LaneSRange]s that make up this [LaneSRoute].
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
    ///
    /// # Returns
    /// A `f64` representing the total length of the [LaneSRoute].
    pub fn length(&self) -> f64 {
        self.lane_s_route.length()
    }

    /// Determines whether this LaneSRoute intersects with `other`.
    ///
    /// # Arguments
    /// * `other` - The other LaneSRoute to check for intersection.
    /// * `tolerance` - The tolerance to use for intersection checks.
    ///
    /// # Returns
    /// * `true` if the two LaneSRoute intersect, `false` otherwise.
    pub fn intersects(&self, other: &LaneSRoute, tolerance: f64) -> bool {
        self.lane_s_route.Intersects(&other.lane_s_route, tolerance)
    }
}

impl std::fmt::Debug for LaneSRoute {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "LaneSRoute {{ ranges: {:?} }}", self.ranges())
    }
}

/// A specific endpoint of a specific Lane.
pub enum LaneEnd<'a> {
    /// The start of the Lane. ("s == 0")
    Start(Lane<'a>),
    /// The end of the Lane. ("s == length")
    Finish(Lane<'a>),
}

impl LaneEnd<'_> {
    /// Gets the Lane of the `LaneEnd`.
    ///
    /// # Returns
    /// A reference to the [Lane] associated with this `LaneEnd`.
    /// This will return the Lane for both Start and Finish variants.
    pub fn lane(&self) -> &Lane<'_> {
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
    /// Obtains the size of the LaneEndSet.
    ///
    /// # Returns
    /// The number of LaneEnds in the set.
    pub fn size(&self) -> i32 {
        self.lane_end_set.size()
    }
    /// Gets the [LaneEnd] at the given index.
    ///
    /// # Arguments
    /// * `index` - The index of the LaneEnd to retrieve.
    ///
    /// # Returns
    /// A [Result<LaneEnd, MaliputError>] containing the LaneEnd at the given index.
    /// If the index is out of bounds, an error is returned.
    pub fn get(&self, index: i32) -> Result<LaneEnd<'_>, MaliputError> {
        let lane_end = self.lane_end_set.get(index)?;
        // Obtain end type and lane reference.
        let is_start = maliput_sys::api::ffi::LaneEnd_is_start(lane_end);
        let lane_ref = unsafe {
            maliput_sys::api::ffi::LaneEnd_lane(lane_end)
                .as_ref()
                .expect("Underlying LaneEnd is null")
        };
        // Create a LaneEnd enum variant.
        Ok(match is_start {
            true => LaneEnd::Start(Lane { lane: lane_ref }),
            false => LaneEnd::Finish(Lane { lane: lane_ref }),
        })
    }

    /// Converts the LaneEndSet to a map of lane-id to LaneEnd.
    ///
    /// # Returns
    /// A `HashMap<String, LaneEnd>` where the key is the lane id and
    /// the value is the corresponding LaneEnd.
    pub fn to_lane_map(&self) -> std::collections::HashMap<String, LaneEnd<'_>> {
        (0..self.size())
            .map(|i| {
                let end = self.get(i).unwrap();
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
    /// Returns the id of the BranchPoint.
    ///
    /// # Returns
    /// A `String` containing the id of the BranchPoint.
    pub fn id(&self) -> String {
        maliput_sys::api::ffi::BranchPoint_id(self.branch_point)
    }
    /// Returns the [RoadGeometry] to which this BranchPoint belongs.
    ///
    /// # Returns
    /// A [RoadGeometry] containing the RoadGeometry to which this BranchPoint belongs.
    pub fn road_geometry(&self) -> RoadGeometry<'_> {
        unsafe {
            RoadGeometry {
                rg: self.branch_point.road_geometry().as_ref().expect(""),
            }
        }
    }
    /// Returns the set of [LaneEnd]s on the same side as the given [LaneEnd].
    /// E.g: For a T-junction, this would return the set of LaneEnds on the merging side.
    ///
    /// # Arguments
    /// * `end` - This branch's start or end [LaneEnd].
    ///
    /// # Return
    /// A [LaneEndSet] of [LaneEnd]s on the same side as the given [LaneEnd].
    pub fn get_confluent_branches(&self, end: &LaneEnd) -> Result<LaneEndSet<'_>, MaliputError> {
        let lane_end_set_ptr = self.branch_point.GetConfluentBranches(
            BranchPoint::from_lane_end_to_ffi(end)
                .as_ref()
                .expect("Underlying LaneEnd is null"),
        )?;
        Ok(LaneEndSet {
            lane_end_set: unsafe { lane_end_set_ptr.as_ref().expect("Underlying LaneEndSet is null") },
        })
    }
    /// Returns the set of [LaneEnd]s on the opposite side as the given [LaneEnd].
    /// E.g: For a T-junction, this would return the [LaneEnd]s which end flows into the junction.
    ///
    /// # Arguments
    /// * `end` - This branch's start or end [LaneEnd].
    ///
    /// # Return
    /// A [LaneEndSet] of [LaneEnd]s on the opposite side as the given [LaneEnd].
    pub fn get_ongoing_branches(&self, end: &LaneEnd) -> Result<LaneEndSet<'_>, MaliputError> {
        let lane_end_set_ptr = self.branch_point.GetOngoingBranches(
            BranchPoint::from_lane_end_to_ffi(end)
                .as_ref()
                .expect("Underlying LaneEnd is null"),
        )?;
        Ok(LaneEndSet {
            lane_end_set: unsafe { lane_end_set_ptr.as_ref().expect("Underlying LaneEndSet is null") },
        })
    }
    /// Returns the default ongoing branch (if any) for the given `end`.
    /// This typically represents what would be considered "continuing
    /// through-traffic" from `end` (e.g., as opposed to a branch executing
    /// a turn).
    ///
    /// If `end` has no default-branch at this BranchPoint, the return
    /// value will be None.
    ///
    /// # Arguments
    /// * `end` - The [LaneEnd] for which to get the default branch.
    ///
    /// # Returns
    /// An `Option<LaneEnd>` containing the default branch if it exists.
    /// If no default branch exists, it returns None.
    pub fn get_default_branch(&self, end: &LaneEnd) -> Option<LaneEnd<'_>> {
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
    ///
    /// # Returns
    /// A [LaneEndSet] containing the LaneEnds on the "A-side" of the BranchPoint.
    pub fn get_a_side(&self) -> LaneEndSet<'_> {
        let lane_end_set_ptr = self.branch_point.GetASide();
        LaneEndSet {
            lane_end_set: unsafe { lane_end_set_ptr.as_ref().expect("Underlying LaneEndSet is null") },
        }
    }
    /// Returns the set of LaneEnds grouped together on the "B-side".
    ///
    /// # Returns
    /// A [LaneEndSet] containing the LaneEnds on the "B-side" of the BranchPoint.
    /// This is the opposite side of the "A-side".
    pub fn get_b_side(&self) -> LaneEndSet<'_> {
        let lane_end_set_ptr = self.branch_point.GetBSide();
        LaneEndSet {
            lane_end_set: unsafe { lane_end_set_ptr.as_ref().expect("Underlying LaneEndSet is null") },
        }
    }
    /// Converts LaneEnd enum to LaneEnd ffi.
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
    intersection: &'a maliput_sys::api::ffi::Intersection,
}

impl<'a> Intersection<'a> {
    /// Returns the id of the `Intersection` as a string.
    pub fn id(&self) -> String {
        maliput_sys::api::ffi::Intersection_id(self.intersection)
    }
    /// Returns the current `phase` of the [Intersection].
    ///
    /// Based on the current `phase`, it returns a [rules::StateProviderQuery] with the phase's ID
    /// and the next state.
    ///
    /// # Returns
    /// A [rules::StateProviderQuery] for the current [rules::Phase].
    pub fn phase(&self) -> rules::StateProviderQuery<String> {
        let query = maliput_sys::api::ffi::Intersection_Phase(self.intersection);
        let next_state = maliput_sys::api::rules::ffi::PhaseStateProvider_next(&query);
        let next_state = if next_state.is_null() {
            None
        } else {
            Some(rules::NextState {
                next_state: next_state.phase_id.clone(),
                duration_until: if next_state.duration_until.is_null() {
                    None
                } else {
                    Some(next_state.duration_until.value)
                },
            })
        };
        rules::StateProviderQuery {
            state: maliput_sys::api::rules::ffi::PhaseStateProvider_state(&query),
            next: next_state,
        }
    }
    /// Returns the region of the `RoadNetwork` that is considered part of the `Intersection`.
    ///
    /// # Returns
    /// A vector of [LaneSRange]s where the intersection lives.
    pub fn region(&self) -> Vec<LaneSRange> {
        self.intersection
            .region()
            .iter()
            .map(|region| LaneSRange {
                lane_s_range: maliput_sys::api::ffi::LaneSRange_new(
                    &maliput_sys::api::ffi::LaneSRange_lane_id(region),
                    &maliput_sys::api::ffi::LaneSRange_s_range(region),
                ),
            })
            .collect::<Vec<LaneSRange>>()
    }
    /// Returns the ID of the [rules::PhaseRing] that applies to this intersection.
    ///
    /// # Returns
    /// A `String` with the ID of a [rules::PhaseRing].
    pub fn phase_ring_id(&self) -> String {
        maliput_sys::api::ffi::Intersection_ring_id(self.intersection)
    }
    pub fn bulb_ids(&self) -> Vec<rules::UniqueBulbId> {
        maliput_sys::api::ffi::Intersection_unique_bulb_ids(self.intersection)
            .iter()
            .map(|unique_bulb_id| rules::UniqueBulbId {
                unique_bulb_id: maliput_sys::api::rules::ffi::UniqueBulbId_create_unique_ptr(unique_bulb_id),
            })
            .collect()
    }
    /// Returns the current [rules::BulbState]s within the `Intersection`.
    ///
    /// # Returns
    /// A vector of [rules::BulbState]s.
    pub fn get_bulb_state(&self, unique_bulb_id: &rules::UniqueBulbId) -> Option<rules::BulbState> {
        let bulb_state =
            maliput_sys::api::ffi::Intersection_bulb_state(self.intersection, &unique_bulb_id.unique_bulb_id);
        if bulb_state.is_null() {
            return None;
        }
        match *bulb_state {
            maliput_sys::api::ffi::BulbState::kOn => Some(rules::BulbState::On),
            maliput_sys::api::ffi::BulbState::kOff => Some(rules::BulbState::Off),
            maliput_sys::api::ffi::BulbState::kBlinking => Some(rules::BulbState::Blinking),
            _ => None,
        }
    }
    /// Returns the current discrete value rule states within the intersection.
    pub fn discrete_value_rule_states(&self) -> Vec<rules::DiscreteValueRuleState> {
        maliput_sys::api::ffi::Intersection_DiscreteValueRuleStates(self.intersection)
            .iter()
            .map(|dvrs| rules::DiscreteValueRuleState {
                rule_id: dvrs.rule_id.clone(),
                state: rules::discrete_value_from_discrete_value_cxx(&dvrs.state),
            })
            .collect::<Vec<rules::DiscreteValueRuleState>>()
    }
    /// Determines whether the [rules::TrafficLight] is within this [Intersection].
    ///
    /// # Arguments
    /// * `traffic_light_id` - A [rules::TrafficLight] ID.
    ///
    /// # Returns
    /// True when `traffic_light_id` is within this [Intersection].
    pub fn includes_traffic_light(&self, traffic_light_id: &str) -> bool {
        maliput_sys::api::ffi::Intersection_IncludesTrafficLight(self.intersection, &traffic_light_id.to_string())
    }
    /// Determines whether the [rules::DiscreteValueRule] is within this [Intersection].
    ///
    /// # Arguments
    /// * `rule_id` - A [rules::DiscreteValueRule] ID.
    ///
    /// # Returns
    /// True when `rule_id` is within this [Intersection].
    pub fn includes_discrete_value_rule(&self, rule_id: &str) -> bool {
        maliput_sys::api::ffi::Intersection_IncludesDiscreteValueRule(self.intersection, &rule_id.to_string())
    }
    /// Determines whether `inertial_position` is within this [Intersection::region].
    ///
    /// `inertial_position` is contained if the distance to the closest LanePosition in
    /// [Intersection::region] is less or equal than the linear tolerance of the `road_geometry`.
    ///
    /// # Arguments
    /// * `inertial_position` -  An [InertialPosition] in the `Inertial`-frame.
    /// * `road_geometry` - The [RoadGeometry].
    ///
    /// # Returns
    /// True when `inertial_position` is within [Intersection::region]. False otherwise.
    pub fn includes_inertial_position(
        &self,
        inertial_position: &InertialPosition,
        road_geometry: &RoadGeometry,
    ) -> bool {
        maliput_sys::api::ffi::Intersection_IncludesInertialPosition(
            self.intersection,
            &maliput_sys::api::ffi::InertialPosition_new(
                inertial_position.x(),
                inertial_position.y(),
                inertial_position.z(),
            ),
            road_geometry.rg,
        )
    }
}

/// A book of Intersections.
pub struct IntersectionBook<'a> {
    intersection_book: &'a maliput_sys::api::ffi::IntersectionBook,
}

impl<'a> IntersectionBook<'a> {
    /// Returns all Intersections in the book.
    ///
    /// # Returns
    /// A vector of [Intersection]s containing all Intersections in the book.
    pub fn get_intersections(&self) -> Vec<Intersection<'_>> {
        let intersections_cpp = maliput_sys::api::ffi::IntersectionBook_GetIntersections(self.intersection_book);
        unsafe {
            intersections_cpp
                .into_iter()
                .map(|intersection| Intersection {
                    intersection: intersection
                        .intersection
                        .as_ref()
                        .expect("Underlying Intersection is null"),
                })
                .collect::<Vec<Intersection>>()
        }
    }

    /// Gets the specified Intersection.
    ///
    /// # Arguments
    ///   * `id` - The id of the Intersection to get.
    ///
    /// # Returns
    ///   * An `Option<Intersection>`
    ///     * Some(Intersection) - The Intersection with the specified id.
    ///     * None - If the Intersection with the specified id does not exist.
    pub fn get_intersection(&self, id: &str) -> Option<Intersection<'_>> {
        let intersection_option = unsafe {
            maliput_sys::api::ffi::IntersectionBook_GetIntersection(self.intersection_book, &String::from(id))
                .intersection
                .as_ref()
        };
        intersection_option.map(|intersection| Intersection { intersection })
    }

    /// Finds the [Intersection] which contains the `traffic_light_id`.
    ///
    /// # Arguments
    /// * `traffic_light_id` - A String with the ID of a [rules::TrafficLight].
    ///
    /// # Returns
    /// The [Intersection] which contains the `traffic_light_id`.
    pub fn find_intersection_with_traffic_light(&self, traffic_light_id: &str) -> Option<Intersection<'_>> {
        let intersection = maliput_sys::api::ffi::IntersectionBook_FindIntersectionTrafficLight(
            self.intersection_book,
            &String::from(traffic_light_id),
        );
        if intersection.intersection.is_null() {
            return None;
        }
        unsafe {
            Some(Intersection {
                intersection: intersection
                    .intersection
                    .as_ref()
                    .expect("Underlying Intersection is null"),
            })
        }
    }

    /// Finds the [Intersection] which contains the `rule_id`.
    ///
    /// # Arguments
    /// * `rule_id` - A String with the ID of a [rules::DiscreteValueRule].
    ///
    /// # Returns
    /// The [Intersection] which contains the `rule_id`.
    pub fn find_intersection_with_discrete_value_rule(&self, rule_id: &str) -> Option<Intersection<'_>> {
        let intersection = maliput_sys::api::ffi::IntersectionBook_FindIntersectionDiscreteValueRule(
            self.intersection_book,
            &String::from(rule_id),
        );
        if intersection.intersection.is_null() {
            return None;
        }
        unsafe {
            Some(Intersection {
                intersection: intersection
                    .intersection
                    .as_ref()
                    .expect("Underlying Intersection is null"),
            })
        }
    }

    /// Finds the [Intersection] which contains the `inertial_position`.
    ///
    /// # Arguments
    /// * `inertial_position` - An [InertialPosition] to find the [Intersection] for.
    ///
    /// # Returns
    /// The [Intersection] which contains the `inertial_position`.
    pub fn find_intersection_with_inertial_position(
        &self,
        inertial_position: &InertialPosition,
    ) -> Option<Intersection<'_>> {
        let intersection = maliput_sys::api::ffi::IntersectionBook_FindIntersectionInertialPosition(
            self.intersection_book,
            &inertial_position.ip,
        );
        if intersection.intersection.is_null() {
            return None;
        }
        unsafe {
            Some(Intersection {
                intersection: intersection
                    .intersection
                    .as_ref()
                    .expect("Underlying Intersection is null"),
            })
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct LaneMarkingLine {
    /// Length of the visible (painted) part of each dash.
    /// For solid lines, this should be 0 (value is ignored).
    pub length: f64,

    /// Length of the gap between visible parts.
    /// For solid lines, this should be 0.
    pub space: f64,

    /// Width of this line [m].
    pub width: f64,

    /// Lateral offset from the lane boundary.
    /// Positive values offset in the positive r-direction (towards the lane's
    /// left edge when facing the positive s-direction).
    /// This allows positioning multiple lines relative to each other.
    pub r_offset: f64,

    /// Color of this specific line.
    /// If set to kUnknown, the parent LaneMarking's color should be used.
    pub color: LaneMarkingColor,
}

impl LaneMarkingLine {
    /// Creates a new LaneMarkingLine.
    pub fn new(length: f64, space: f64, width: f64, r_offset: f64, color: LaneMarkingColor) -> LaneMarkingLine {
        LaneMarkingLine {
            length,
            space,
            width,
            r_offset,
            color,
        }
    }
}

/// LaneMarking classification options.
///
/// LaneMarkingType defines types of lane markings on the road.
#[derive(strum_macros::Display, Debug, Copy, Clone, PartialEq, Eq)]
pub enum LaneMarkingType {
    Unknown,
    None,
    Solid,
    Broken,
    SolidSolid,
    SolidBroken,
    BrokenSolid,
    BrokenBroken,
    BottsDots,
    Grass,
    Curb,
    Edge,
}

#[derive(strum_macros::Display, Debug, Copy, Clone, PartialEq, Eq)]
pub enum LaneMarkingWeight {
    Unknown,
    Standard,
    Bold,
}

#[derive(strum_macros::Display, Debug, Copy, Clone, PartialEq, Eq)]
pub enum LaneMarkingColor {
    Unknown,
    White,
    Yellow,
    Orange,
    Red,
    Blue,
    Green,
    Violet,
}

#[derive(strum_macros::Display, Debug, Copy, Clone, PartialEq, Eq)]
pub enum LaneChangePermission {
    Unknown,
    Allowed,
    ToLeft,
    ToRight,
    Prohibited,
}

/// Describes the complete lane marking at a lane boundary.
///
/// A LaneMarking describes all properties of the road marking at a lane's
/// boundary. The marking can vary along the lane's s-coordinate, so this
/// structure represents the marking for a specific s-range.
///
/// **Simple markings:** For single-line markings (e.g., a solid white edge line),
/// the `type`, `color`, `width`, and `weight` fields are sufficient. The `lines`
/// vector can be left empty.
///
/// **Complex markings:** For compound markings (e.g., double lines with different
/// patterns like `kSolidBroken`), the `lines` vector provides detailed information
/// about each component line. When `lines` is non-empty, the individual line
/// definitions take precedence for geometry and per-line colors. The top-level
/// `type`, `color`, and `width` fields remain useful as summary/fallback values.
pub struct LaneMarking {
    lane_marking: cxx::UniquePtr<maliput_sys::api::ffi::LaneMarking>,
}

impl LaneMarking {
    /// Returns the total width of the marking.
    pub fn width(&self) -> f64 {
        maliput_sys::api::ffi::LaneMarking_width(&self.lane_marking)
    }

    /// Returns the total height of the marking.
    pub fn height(&self) -> f64 {
        maliput_sys::api::ffi::LaneMarking_height(&self.lane_marking)
    }

    /// Returns the marking material.
    ///
    /// # Returns
    /// A [String] describing the marking material.
    pub fn material(&self) -> String {
        maliput_sys::api::ffi::LaneMarking_material(&self.lane_marking)
    }

    /// Returns the type of marking.
    ///
    /// # Returns
    /// A [LaneMarkingType] representing the pattern or type of marking.
    pub fn get_type(&self) -> LaneMarkingType {
        let marking_type = maliput_sys::api::ffi::LaneMarking_type(&self.lane_marking);
        match marking_type {
            maliput_sys::api::ffi::LaneMarkingType::kUnknown => LaneMarkingType::Unknown,
            maliput_sys::api::ffi::LaneMarkingType::kNone => LaneMarkingType::None,
            maliput_sys::api::ffi::LaneMarkingType::kSolid => LaneMarkingType::Solid,
            maliput_sys::api::ffi::LaneMarkingType::kBroken => LaneMarkingType::Broken,
            maliput_sys::api::ffi::LaneMarkingType::kSolidSolid => LaneMarkingType::SolidSolid,
            maliput_sys::api::ffi::LaneMarkingType::kSolidBroken => LaneMarkingType::SolidBroken,
            maliput_sys::api::ffi::LaneMarkingType::kBrokenSolid => LaneMarkingType::BrokenSolid,
            maliput_sys::api::ffi::LaneMarkingType::kBrokenBroken => LaneMarkingType::BrokenBroken,
            maliput_sys::api::ffi::LaneMarkingType::kBottsDots => LaneMarkingType::BottsDots,
            maliput_sys::api::ffi::LaneMarkingType::kGrass => LaneMarkingType::Grass,
            maliput_sys::api::ffi::LaneMarkingType::kCurb => LaneMarkingType::Curb,
            maliput_sys::api::ffi::LaneMarkingType::kEdge => LaneMarkingType::Edge,
            _ => LaneMarkingType::Unknown,
        }
    }

    /// Returns the visual weight or thickness type of the marking.
    ///
    /// # Returns
    /// A [LaneMarkingWeight] that indicates the type of visual weight of the marking.
    pub fn weight(&self) -> LaneMarkingWeight {
        let marking_weight = maliput_sys::api::ffi::LaneMarking_weight(&self.lane_marking);
        match marking_weight {
            maliput_sys::api::ffi::LaneMarkingWeight::kUnknown => LaneMarkingWeight::Unknown,
            maliput_sys::api::ffi::LaneMarkingWeight::kStandard => LaneMarkingWeight::Standard,
            maliput_sys::api::ffi::LaneMarkingWeight::kBold => LaneMarkingWeight::Bold,
            _ => LaneMarkingWeight::Unknown,
        }
    }
    pub fn color(&self) -> LaneMarkingColor {
        self.get_marking_color(maliput_sys::api::ffi::LaneMarking_color(&self.lane_marking))
    }

    /// Returns the type of lane change the marking allows.
    ///
    /// # Returns
    /// A [LaneChangePermission] that indicates the type of lane change that is allowed.
    pub fn lane_change(&self) -> LaneChangePermission {
        let lane_change = maliput_sys::api::ffi::LaneMarking_lane_change(&self.lane_marking);
        match lane_change {
            maliput_sys::api::ffi::LaneChangePermission::kUnknown => LaneChangePermission::Unknown,
            maliput_sys::api::ffi::LaneChangePermission::kAllowed => LaneChangePermission::Allowed,
            maliput_sys::api::ffi::LaneChangePermission::kToLeft => LaneChangePermission::ToLeft,
            maliput_sys::api::ffi::LaneChangePermission::kToRight => LaneChangePermission::ToRight,
            maliput_sys::api::ffi::LaneChangePermission::kProhibited => LaneChangePermission::Prohibited,
            _ => LaneChangePermission::Unknown,
        }
    }

    /// Returns all lines in the LaneMarking.
    ///
    /// # Returns
    /// A vector of [LaneMarkingLine]s.
    pub fn lines(&self) -> Vec<LaneMarkingLine> {
        let lines = maliput_sys::api::ffi::LaneMarking_lines(&self.lane_marking);
        lines
            .into_iter()
            .map(|line| LaneMarkingLine {
                length: maliput_sys::api::ffi::LaneMarkingLine_length(line),
                space: maliput_sys::api::ffi::LaneMarkingLine_space(line),
                width: maliput_sys::api::ffi::LaneMarkingLine_width(line),
                r_offset: maliput_sys::api::ffi::LaneMarkingLine_r_offset(line),
                color: self.get_marking_color(maliput_sys::api::ffi::LaneMarkingLine_color(line)),
            })
            .collect::<Vec<LaneMarkingLine>>()
    }

    // Private helper to get marking color and avoid code duplication.
    fn get_marking_color(&self, color: maliput_sys::api::ffi::LaneMarkingColor) -> LaneMarkingColor {
        match color {
            maliput_sys::api::ffi::LaneMarkingColor::kUnknown => LaneMarkingColor::Unknown,
            maliput_sys::api::ffi::LaneMarkingColor::kWhite => LaneMarkingColor::White,
            maliput_sys::api::ffi::LaneMarkingColor::kYellow => LaneMarkingColor::Yellow,
            maliput_sys::api::ffi::LaneMarkingColor::kOrange => LaneMarkingColor::Orange,
            maliput_sys::api::ffi::LaneMarkingColor::kRed => LaneMarkingColor::Red,
            maliput_sys::api::ffi::LaneMarkingColor::kBlue => LaneMarkingColor::Blue,
            maliput_sys::api::ffi::LaneMarkingColor::kGreen => LaneMarkingColor::Green,
            maliput_sys::api::ffi::LaneMarkingColor::kViolet => LaneMarkingColor::Violet,
            _ => LaneMarkingColor::Unknown,
        }
    }
}

/// The result of querying a [LaneMarking] at a specific position or range.
/// This structure pairs a [LaneMarking] with the s-range over which it is valid.
/// Lane markings can change along the lane (e.g., solid to broken at an
/// intersection approach), so the validity range is essential.
///
/// The range uses half-open interval semantics: `[s_start, s_end)`.
pub struct LaneMarkingQuery {
    /// LaneMarking description.
    pub lane_marking: LaneMarking,
    /// Start s-coordinate where the marking begins
    /// This is relative to the lane's s-coordinate system..
    pub s_start: f64,
    /// End s-coordinate where the marking ends.
    /// This is relative to the lane's s-coordinate system.
    pub s_end: f64,
}

/// Represents a boundary between adjacent lanes or at the edge of a Segment.
///
/// A [LaneBoundary] is owned by a [Segment] and serves as the interface between
/// two adjacent lanes, or between a lane and the segment edge. For a [Segment]
/// with N lanes, there are N+1 boundaries:
///
/// ```text
///                                                        +r direction
///                                                      <─────────────
///   Boundary N    ...    Boundary 2    Boundary 1    Boundary 0
///      |                     |             |             |
///      | Lane N-1  |   ...   |   Lane 1    |   Lane 0    |
///      |                     |             |             |
///  (left edge)                                     (right edge)
/// ```
///
/// Where:
/// - Boundary 0 is the right edge (minimum r coordinate).
/// - Boundary N is the left edge (maximum r coordinate).
/// - Boundaries are indexed with increasing r direction.
///
/// Each [LaneBoundary] provides:
/// - Reference to the lane on its left (if any).
/// - Reference to the lane on its right (if any).
/// - Query methods for lane markings at specific s-coordinates.
///
/// The design ensures that adjacent lanes share the same boundary object,
/// avoiding redundancy and ensuring consistency. For example, [Lane] 1's right
/// boundary is the same object as [Lane] 0's left boundary (Boundary 1).
pub struct LaneBoundary<'a> {
    lane_boundary: &'a maliput_sys::api::ffi::LaneBoundary,
}

impl<'a> LaneBoundary<'a> {
    /// Returns the ID of the [LaneBoundary].
    /// The ID is a unique identifier for the [LaneBoundary] within the Segment.
    ///
    /// # Returns
    /// A `String` containing the ID of the [LaneBoundary].
    pub fn id(&self) -> String {
        maliput_sys::api::ffi::LaneBoundary_id(self.lane_boundary)
    }

    /// Returns the segment that contains this [LaneBoundary].
    ///
    /// # Returns
    /// An `Option<Segment>` containing a reference to the [Segment] if it exists,
    /// or `None` if the [LaneBoundary] is not associated with a segment.
    pub fn segment(&self) -> Option<Segment<'a>> {
        let segment = self.lane_boundary.segment();
        if segment.is_null() {
            return None;
        }
        Some(unsafe {
            Segment {
                segment: segment.as_ref().expect(""),
            }
        })
    }

    /// Returns the index of this boundary within the parent [Segment].
    ///
    /// Boundaries are indexed from 0 (rightmost, minimum r) to num_lanes()
    /// (leftmost, maximum r).
    pub fn index(&self) -> i32 {
        self.lane_boundary.index()
    }

    /// Returns the [Lane] immediately to the left of this boundary (increasing r direction).
    ///
    /// # Returns
    /// An `Option<Lane>` containing the lane to the left, or `None` if this is
    /// the leftmost boundary of the [Segment].
    pub fn lane_to_left(&self) -> Option<Lane<'a>> {
        let lane = self.lane_boundary.lane_to_left();
        if lane.is_null() {
            return None;
        }
        Some(unsafe {
            Lane {
                lane: lane.as_ref().expect("Lane pointer from lane_to_left is null"),
            }
        })
    }

    /// Returns the [Lane] immediately to the right of this boundary (decreasing r direction).
    ///
    /// # Returns
    /// An `Option<Lane>` containing the lane to the right, or `None` if this is
    /// the rightmost boundary of the Segment.
    pub fn lane_to_right(&self) -> Option<Lane<'a>> {
        let lane = self.lane_boundary.lane_to_right();
        if lane.is_null() {
            return None;
        }
        Some(unsafe {
            Lane {
                lane: lane.as_ref().expect("Lane pointer from lane_to_right is null"),
            }
        })
    }

    /// Gets the lane marking at a specific s-coordinate along this boundary.
    ///
    /// # Arguments
    /// * `s` - The s-coordinate along the boundary (in the lane coordinate system).
    ///   Typically in the range [0, segment_length].
    ///
    /// # Returns
    /// A [LaneMarkingQuery] at the specified position, including the marking details and the
    /// s-range over which it is valid, or `None` if no marking information is available at that location.
    pub fn get_marking(&self, s: f64) -> Option<LaneMarkingQuery> {
        let lane_marking_query = maliput_sys::api::ffi::LaneBoundary_GetMarking(self.lane_boundary, s);
        if lane_marking_query.is_null() {
            return None;
        }
        Some(self.create_lane_marking_query(&lane_marking_query))
    }

    /// Gets all lane markings along this boundary.
    ///
    /// # Returns
    /// A vector of [LaneMarkingQuery], each describing a marking and the s-range over which it is valid.
    /// The queried results are ordered by increasing s_start. If no markings are available, returns an empty vector.
    pub fn get_markings(&self) -> Vec<LaneMarkingQuery> {
        let lane_marking_queries = maliput_sys::api::ffi::LaneBoundary_GetMarkings(self.lane_boundary);
        lane_marking_queries
            .into_iter()
            .map(|lane_marking_query| self.create_lane_marking_query(lane_marking_query))
            .collect::<Vec<LaneMarkingQuery>>()
    }

    /// Gets lane markings within a specified s-range.
    ///
    /// # Arguments
    /// * `s_start` - Start of the s-range to query.
    /// * `s_end` - End of the s-range to query.
    ///
    /// # Returns
    /// A vector of [LaneMarkingQuery] for markings that overlap with the specified range. Queried results are ordered by increasing `s_start`.
    /// If no markings are available in the range, returns an empty vector.
    pub fn get_markings_by_range(&self, s_start: f64, s_end: f64) -> Vec<LaneMarkingQuery> {
        let lane_marking_queries =
            maliput_sys::api::ffi::LaneBoundary_GetMarkingsByRange(self.lane_boundary, s_start, s_end);
        lane_marking_queries
            .into_iter()
            .map(|lane_marking_query| self.create_lane_marking_query(lane_marking_query))
            .collect::<Vec<LaneMarkingQuery>>()
    }

    // Private helper to create a LaneMarkingQuery.
    fn create_lane_marking_query(
        &self,
        lane_marking_query: &maliput_sys::api::ffi::LaneMarkingResult,
    ) -> LaneMarkingQuery {
        LaneMarkingQuery {
            lane_marking: LaneMarking {
                lane_marking: maliput_sys::api::ffi::LaneMarkingResult_marking(lane_marking_query),
            },
            s_start: maliput_sys::api::ffi::LaneMarkingResult_s_start(lane_marking_query),
            s_end: maliput_sys::api::ffi::LaneMarkingResult_s_end(lane_marking_query),
        }
    }
}

mod tests {
    mod road_geometry {
        #[test]
        fn to_road_position_surface_test() {
            let package_location = std::env::var("CARGO_MANIFEST_DIR").unwrap();
            let xodr_path = format!("{}/data/xodr/TShapeRoad.xodr", package_location);
            let road_network_properties = std::collections::HashMap::from([
                ("road_geometry_id", "my_rg_from_rust"),
                ("opendrive_file", xodr_path.as_str()),
            ]);
            let road_network = crate::api::RoadNetwork::new("maliput_malidrive", &road_network_properties).unwrap();
            let road_geometry = road_network.road_geometry();

            // Although this isn't directly on the surface, it is within the RoadGeometry volume.
            let inertial_pos = crate::api::InertialPosition::new(1.0, 0.0, 2.0);
            let road_pos = road_geometry.to_road_position_surface(&inertial_pos);
            assert!(road_pos.is_ok());
            let road_pos = road_pos.unwrap();
            let tolerance = 1e-3;
            assert!((road_pos.pos().s() - 1.0).abs() < tolerance);
            assert!((road_pos.pos().r() - -1.75).abs() < tolerance);
            assert!((road_pos.pos().h() - 0.0).abs() < tolerance);

            // An inertial position that is off the road volume.
            let inertial_pos = crate::api::InertialPosition::new(1.0, 0.0, 10.0);
            let road_pos = road_geometry.to_road_position_surface(&inertial_pos);
            assert!(road_pos.is_err());
        }
    }
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
        // # Arguments
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
