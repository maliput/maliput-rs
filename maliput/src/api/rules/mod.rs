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

use std::collections::HashMap;

use crate::{api::RoadPosition, common::MaliputError};
use strum_macros::{Display, EnumString, IntoStaticStr};

/// Interface for accessing the [TrafficLight] in the [super::RoadNetwork]
pub struct TrafficLightBook<'a> {
    pub(super) traffic_light_book: &'a maliput_sys::api::rules::ffi::TrafficLightBook,
}

impl<'a> TrafficLightBook<'a> {
    /// Gets all the [TrafficLight]s in the [TrafficLightBook]
    ///
    /// # Returns
    /// A vector of [TrafficLight]s
    pub fn traffic_lights(&self) -> Vec<TrafficLight<'_>> {
        let traffic_lights_cpp = maliput_sys::api::rules::ffi::TrafficLightBook_TrafficLights(self.traffic_light_book);
        traffic_lights_cpp
            .into_iter()
            .map(|tl| TrafficLight {
                traffic_light: unsafe { tl.traffic_light.as_ref().expect("") },
            })
            .collect::<Vec<TrafficLight>>()
    }

    /// Gets a [TrafficLight] by its id.
    ///
    /// # Arguments
    /// * `id` - The id of the [TrafficLight].
    ///
    /// # Returns
    /// The [TrafficLight] with the given id.
    /// If no [TrafficLight] is found with the given id, return None.
    pub fn get_traffic_light(&self, id: &String) -> Option<TrafficLight<'_>> {
        let traffic_light = maliput_sys::api::rules::ffi::TrafficLightBook_GetTrafficLight(self.traffic_light_book, id);
        if traffic_light.is_null() {
            return None;
        }
        Some(TrafficLight {
            traffic_light: unsafe {
                traffic_light
                    .as_ref()
                    .expect("Unable to get underlying traffic light pointer")
            },
        })
    }

    /// Gets all [TrafficLight]s whose `related_lanes()` includes the given lane ID.
    ///
    /// # Arguments
    /// * `lane_id` - The lane ID to look up.
    ///
    /// # Returns
    /// A vector of [TrafficLight]s associated with the given lane.
    /// Returns an empty vector if no traffic lights are associated with the lane.
    pub fn find_by_lane(&self, lane_id: &String) -> Vec<TrafficLight<'_>> {
        let traffic_lights_cpp =
            maliput_sys::api::rules::ffi::TrafficLightBook_FindByLane(self.traffic_light_book, lane_id);
        traffic_lights_cpp
            .into_iter()
            .map(|tl| TrafficLight {
                traffic_light: unsafe { tl.traffic_light.as_ref().expect("TrafficLight pointer is null") },
            })
            .collect::<Vec<TrafficLight>>()
    }
}

/// Models a traffic light. A traffic light is a physical signaling device
/// typically located at road intersections. It contains one or more groups of
/// light bulbs with varying colors and shapes. The lighting patterns of the
/// bulbs signify right-of-way rule information to the agents navigating the
/// intersection (e.g., vehicles, bicyclists, pedestrians, etc.). Typically, an
/// intersection will be managed by multiple traffic lights.
///
/// Note that traffic lights are physical manifestations of underlying
/// right-of-way rules and thus naturally have lower signal-to-noise ratio
/// relative to the underlying rules. Thus, oracular agents should directly use
/// the underlying right-of-way rules instead of traffic lights when navigating
/// intersections. TrafficLight exists for testing autonomous vehicles that do
/// not have access to right-of-way rules.
pub struct TrafficLight<'a> {
    pub traffic_light: &'a maliput_sys::api::rules::ffi::TrafficLight,
}

impl<'a> TrafficLight<'a> {
    /// Get the id of the [TrafficLight].
    ///
    /// # Returns
    /// The id of the [TrafficLight].
    pub fn id(&self) -> String {
        maliput_sys::api::rules::ffi::TrafficLight_id(self.traffic_light)
    }

    /// Get the position of the [TrafficLight] in the road network.
    ///
    /// # Returns
    /// An [super::InertialPosition] representing the position of the [TrafficLight] in the road network.
    pub fn position_road_network(&self) -> super::InertialPosition {
        let inertial_position = maliput_sys::api::rules::ffi::TrafficLight_position_road_network(self.traffic_light);
        super::InertialPosition { ip: inertial_position }
    }

    /// Get the orientation of the [TrafficLight] in the road network.
    ///
    /// # Returns
    /// An [super::Rotation] representing the orientation of the [TrafficLight] in the road network.
    pub fn orientation_road_network(&self) -> super::Rotation {
        let rotation = maliput_sys::api::rules::ffi::TrafficLight_orientation_road_network(self.traffic_light);
        super::Rotation { r: rotation }
    }

    /// Get the bulb groups of the [TrafficLight].
    ///
    /// # Returns
    /// A vector of [BulbGroup]s in the [TrafficLight].
    /// If the [TrafficLight] has no bulb groups, return an empty vector.
    pub fn bulb_groups(&self) -> Vec<BulbGroup<'_>> {
        let bulb_groups_cpp = maliput_sys::api::rules::ffi::TrafficLight_bulb_groups(self.traffic_light);
        bulb_groups_cpp
            .into_iter()
            .map(|bg| BulbGroup {
                bulb_group: unsafe { bg.bulb_group.as_ref().expect("") },
            })
            .collect::<Vec<BulbGroup>>()
    }

    /// Get a [BulbGroup] by its id.
    ///
    /// # Arguments
    /// * `id` - The id of the [BulbGroup].
    ///
    /// # Returns
    /// The [BulbGroup] with the given id.
    /// If no [BulbGroup] is found with the given id, return None.
    pub fn get_bulb_group(&self, id: &String) -> Option<BulbGroup<'_>> {
        let bulb_group = maliput_sys::api::rules::ffi::TrafficLight_GetBulbGroup(self.traffic_light, id);
        if bulb_group.is_null() {
            return None;
        }
        Some(BulbGroup {
            bulb_group: unsafe {
                bulb_group
                    .as_ref()
                    .expect("Unable to get underlying bulb group pointer")
            },
        })
    }

    /// Get the lane IDs that this traffic light is physically relevant to.
    ///
    /// # Returns
    /// A vector of lane ID strings.
    pub fn related_lanes(&self) -> Vec<String> {
        maliput_sys::api::rules::ffi::TrafficLight_related_lanes(self.traffic_light)
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
/// Defines the possible bulb colors.
pub enum BulbColor {
    Red,
    Yellow,
    Green,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
/// Defines the possible bulb types.
pub enum BulbType {
    Round,
    /// Arrow with a custom orientation specified by [Bulb::arrow_orientation_rad].
    Arrow,
    /// Predefined arrow pointing left.
    ArrowLeft,
    /// Predefined arrow pointing right.
    ArrowRight,
    /// Predefined arrow pointing up (forward).
    ArrowUp,
    /// Predefined arrow pointing upper-left.
    ArrowUpperLeft,
    /// Predefined arrow pointing upper-right.
    ArrowUpperRight,
    /// U-turn to the left.
    UTurnLeft,
    /// U-turn to the right.
    UTurnRight,
    /// Pedestrian walk signal.
    Walk,
    /// Pedestrian don't walk signal.
    DontWalk,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
/// Defines the possible bulb states.
pub enum BulbState {
    Off,
    On,
    Blinking,
    Counting,
}

/// Models a bulb within a bulb group.
pub struct Bulb<'a> {
    pub bulb: &'a maliput_sys::api::rules::ffi::Bulb,
}

impl Bulb<'_> {
    /// Returns this Bulb instance's unique identifier.
    ///
    /// # Returns
    /// A [UniqueBulbId] representing the unique identifier of the [Bulb].
    pub fn unique_id(&self) -> UniqueBulbId {
        UniqueBulbId {
            unique_bulb_id: maliput_sys::api::rules::ffi::Bulb_unique_id(self.bulb),
        }
    }

    /// Get the id of the [Bulb].
    ///
    /// # Returns
    /// The id of the [Bulb].
    pub fn id(&self) -> String {
        maliput_sys::api::rules::ffi::Bulb_id(self.bulb)
    }

    /// Get the color of the [Bulb].
    ///
    /// # Returns
    /// The [BulbColor].
    pub fn color(&self) -> BulbColor {
        let color = self.bulb.color();
        match *color {
            maliput_sys::api::rules::ffi::BulbColor::kRed => BulbColor::Red,
            maliput_sys::api::rules::ffi::BulbColor::kYellow => BulbColor::Yellow,
            maliput_sys::api::rules::ffi::BulbColor::kGreen => BulbColor::Green,
            _ => panic!("Invalid bulb color"),
        }
    }

    /// Get the type of the [Bulb].
    ///
    /// # Returns
    /// The [BulbType].
    pub fn bulb_type(&self) -> BulbType {
        let bulb_type = maliput_sys::api::rules::ffi::Bulb_type(self.bulb);
        match *bulb_type {
            maliput_sys::api::rules::ffi::BulbType::kRound => BulbType::Round,
            maliput_sys::api::rules::ffi::BulbType::kArrow => BulbType::Arrow,
            maliput_sys::api::rules::ffi::BulbType::kArrowLeft => BulbType::ArrowLeft,
            maliput_sys::api::rules::ffi::BulbType::kArrowRight => BulbType::ArrowRight,
            maliput_sys::api::rules::ffi::BulbType::kArrowUp => BulbType::ArrowUp,
            maliput_sys::api::rules::ffi::BulbType::kArrowUpperLeft => BulbType::ArrowUpperLeft,
            maliput_sys::api::rules::ffi::BulbType::kArrowUpperRight => BulbType::ArrowUpperRight,
            maliput_sys::api::rules::ffi::BulbType::kUTurnLeft => BulbType::UTurnLeft,
            maliput_sys::api::rules::ffi::BulbType::kUTurnRight => BulbType::UTurnRight,
            maliput_sys::api::rules::ffi::BulbType::kWalk => BulbType::Walk,
            maliput_sys::api::rules::ffi::BulbType::kDontWalk => BulbType::DontWalk,
            _ => panic!("Invalid bulb type"),
        }
    }

    /// Get the position of the [Bulb] in the bulb group.
    ///
    /// # Returns
    /// An [super::InertialPosition] representing the position of the [Bulb] in the bulb group.
    pub fn position_bulb_group(&self) -> super::InertialPosition {
        let inertial_position = maliput_sys::api::rules::ffi::Bulb_position_bulb_group(self.bulb);
        super::InertialPosition { ip: inertial_position }
    }

    /// Get the orientation of the [Bulb] in the bulb group.
    ///
    /// # Returns
    /// An [super::Rotation] representing the orientation of the [Bulb] in the bulb group.
    pub fn orientation_bulb_group(&self) -> super::Rotation {
        let rotation = maliput_sys::api::rules::ffi::Bulb_orientation_bulb_group(self.bulb);
        super::Rotation { r: rotation }
    }

    /// Returns the arrow's orientation. Only applicable if [Bulb::bulb_type] returns [BulbType::Arrow].
    ///
    /// # Returns
    /// An `Option<f64>` representing the orientation of the arrow in radians.
    pub fn arrow_orientation_rad(&self) -> Option<f64> {
        let arrow_orientation = maliput_sys::api::rules::ffi::Bulb_arrow_orientation_rad(self.bulb);
        if arrow_orientation.is_null() {
            return None;
        }
        Some(arrow_orientation.value)
    }

    /// Gets the possible states of the [Bulb].
    ///
    /// # Returns
    /// A vector of [BulbState]s representing the possible states of the [Bulb].
    pub fn states(&self) -> Vec<BulbState> {
        let states_cpp = maliput_sys::api::rules::ffi::Bulb_states(self.bulb);
        states_cpp
            .into_iter()
            .map(Bulb::_from_cpp_state_to_rust_state)
            .collect::<Vec<BulbState>>()
    }

    /// Gets the default state of the [Bulb].
    ///
    /// # Returns
    /// A [BulbState] representing the default state of the [Bulb].
    pub fn get_default_state(&self) -> BulbState {
        let default_state = self.bulb.GetDefaultState();
        Bulb::_from_cpp_state_to_rust_state(&default_state)
    }

    /// Gets the initial state of the [Bulb].
    ///
    /// # Returns
    /// A [BulbState] representing the initial state of the [Bulb].
    pub fn get_initial_state(&self) -> BulbState {
        let initial_state = self.bulb.GetInitialState();
        Bulb::_from_cpp_state_to_rust_state(&initial_state)
    }

    /// Check if the given state is possible valid for the [Bulb].
    ///
    /// # Arguments
    /// * `state` - The [BulbState] to check.
    ///
    /// # Returns
    /// A boolean indicating whether the given state is valid for the [Bulb].
    pub fn is_valid_state(&self, state: &BulbState) -> bool {
        self.bulb.IsValidState(&Bulb::_from_rust_state_to_cpp_state(state))
    }

    /// Returns the bounding box of the bulb.
    ///
    /// # Returns
    /// A tuple containing the minimum and maximum points of the bounding box.
    pub fn bounding_box(&self) -> (crate::math::Vector3, crate::math::Vector3) {
        let min = maliput_sys::api::rules::ffi::Bulb_bounding_box_min(self.bulb);
        let max = maliput_sys::api::rules::ffi::Bulb_bounding_box_max(self.bulb);
        (crate::math::Vector3 { v: min }, crate::math::Vector3 { v: max })
    }

    /// Returns the parent [BulbGroup] of the bulb.
    ///
    /// # Returns
    /// The parent [BulbGroup] of the bulb.
    /// If the bulb is not part of any group, return None.
    pub fn bulb_group(&self) -> BulbGroup<'_> {
        BulbGroup {
            bulb_group: unsafe {
                maliput_sys::api::rules::ffi::Bulb_bulb_group(self.bulb)
                    .as_ref()
                    .expect("Unable to get underlying bulb group pointer. The Bulb might not be part of any BulbGroup.")
            },
        }
    }

    /// Convert from the C++ BulbState to the Rust BulbState
    /// It is expected to be used only internally.
    ///
    /// # Arguments
    /// * `cpp_bulb_state` - The C++ BulbState
    ///
    /// # Returns
    /// The Rust BulbState
    ///
    /// # Panics
    /// If the C++ BulbState is invalid.
    fn _from_cpp_state_to_rust_state(cpp_bulb_state: &maliput_sys::api::rules::ffi::BulbState) -> BulbState {
        match *cpp_bulb_state {
            maliput_sys::api::rules::ffi::BulbState::kOff => BulbState::Off,
            maliput_sys::api::rules::ffi::BulbState::kOn => BulbState::On,
            maliput_sys::api::rules::ffi::BulbState::kBlinking => BulbState::Blinking,
            maliput_sys::api::rules::ffi::BulbState::kCounting => BulbState::Counting,
            _ => panic!("Invalid bulb state"),
        }
    }

    /// Convert from the Rust BulbState to the C++ BulbState
    /// It is expected to be used only internally.
    ///
    /// # Arguments
    /// * `rust_bulb_state` - The Rust BulbState
    ///
    /// # Returns
    /// The C++ BulbState
    fn _from_rust_state_to_cpp_state(rust_bulb_state: &BulbState) -> maliput_sys::api::rules::ffi::BulbState {
        match rust_bulb_state {
            BulbState::Off => maliput_sys::api::rules::ffi::BulbState::kOff,
            BulbState::On => maliput_sys::api::rules::ffi::BulbState::kOn,
            BulbState::Blinking => maliput_sys::api::rules::ffi::BulbState::kBlinking,
            BulbState::Counting => maliput_sys::api::rules::ffi::BulbState::kCounting,
        }
    }
}

/// Models a group of bulbs within a traffic light. All of the bulbs within a
/// group should share the same approximate orientation. However, this is not
/// programmatically enforced.
/// About the bulb group pose:
/// - The position of the bulb group is defined as the linear offset of this bulb group's frame
///   relative to the frame of the traffic light that contains it. The origin of
///   this bulb group's frame should approximate the bulb group's CoM.
/// - The orientation of the bulb group is defined as the rotational offset of this bulb
///   group's frame relative to the frame of the traffic light that contains it.
///   The +Z axis should align with the bulb group's "up" direction, and the +X
///   axis should point in the direction that the bulb group is facing.
///   Following a right-handed coordinate frame, the +Y axis should point left
///   when facing the +X direction.
pub struct BulbGroup<'a> {
    pub bulb_group: &'a maliput_sys::api::rules::ffi::BulbGroup,
}

impl BulbGroup<'_> {
    /// Returns this BulbGroup instance's unique identifier.
    ///
    /// # Returns
    /// A [UniqueBulbGroupId] representing the unique identifier of the [BulbGroup].
    pub fn unique_id(&self) -> UniqueBulbGroupId {
        UniqueBulbGroupId {
            unique_bulb_group_id: maliput_sys::api::rules::ffi::BulbGroup_unique_id(self.bulb_group),
        }
    }

    /// Gets the id of the [BulbGroup].
    ///
    /// # Returns
    /// The id of the [BulbGroup].
    pub fn id(&self) -> String {
        maliput_sys::api::rules::ffi::BulbGroup_id(self.bulb_group)
    }

    /// Gets the position of the [BulbGroup] in the traffic light.
    ///
    /// # Returns
    /// An [super::InertialPosition] representing the position of the [BulbGroup] in the traffic light.
    pub fn position_traffic_light(&self) -> super::InertialPosition {
        let inertial_position = maliput_sys::api::rules::ffi::BulbGroup_position_traffic_light(self.bulb_group);
        super::InertialPosition { ip: inertial_position }
    }

    /// Gets the orientation of the [BulbGroup] in the traffic light.
    ///
    /// # Returns
    /// An [super::Rotation] representing the orientation of the [BulbGroup] in the traffic light.
    pub fn orientation_traffic_light(&self) -> super::Rotation {
        let rotation = maliput_sys::api::rules::ffi::BulbGroup_orientation_traffic_light(self.bulb_group);
        super::Rotation { r: rotation }
    }

    /// Returns the bulbs in the bulb group.
    ///
    /// # Returns
    /// A vector of [Bulb]s in the bulb group.
    pub fn bulbs(&self) -> Vec<Bulb<'_>> {
        let bulbs_cpp = maliput_sys::api::rules::ffi::BulbGroup_bulbs(self.bulb_group);
        bulbs_cpp
            .into_iter()
            .map(|b| Bulb {
                bulb: unsafe { b.bulb.as_ref().expect("") },
            })
            .collect::<Vec<Bulb>>()
    }

    /// Gets a [Bulb] by its id
    ///
    /// # Arguments
    /// * `id` - The id of the [Bulb].
    ///
    /// # Returns
    /// The [Bulb] with the given id.
    /// If no [Bulb] is found with the given id, return None.
    pub fn get_bulb(&self, id: &String) -> Option<Bulb<'_>> {
        let bulb = maliput_sys::api::rules::ffi::BulbGroup_GetBulb(self.bulb_group, id);
        if bulb.is_null() {
            return None;
        }
        Some(Bulb {
            bulb: unsafe { bulb.as_ref().expect("Unable to get underlying bulb pointer") },
        })
    }

    /// Returns the parent [TrafficLight] of the bulb group.
    ///
    /// # Returns
    /// The parent [TrafficLight] of the bulb group.
    pub fn traffic_light(&self) -> TrafficLight<'_> {
        TrafficLight {
            traffic_light: unsafe {
                maliput_sys::api::rules::ffi::BulbGroup_traffic_light(self.bulb_group)
                    .as_ref()
                    .expect("Unable to get underlying traffic light pointer. The BulbGroup might not be registered to a TrafficLight.")
            },
        }
    }
}

/// Uniquely identifies a bulb in the `Inertial` space. This consists of the
/// concatenation of the bulb's ID, the ID of the bulb group that contains the
/// bulb, and the the ID of the traffic light that contains the bulb group.
///
/// String representation of this ID is:
/// "`traffic_light_id().string()`-`bulb_group_id.string()`-`bulb_id.string()`"
pub struct UniqueBulbId {
    pub(crate) unique_bulb_id: cxx::UniquePtr<maliput_sys::api::rules::ffi::UniqueBulbId>,
}

impl UniqueBulbId {
    /// Get the traffic light id of the [UniqueBulbId].
    ///
    /// # Returns
    /// The traffic light id of the [UniqueBulbId].
    pub fn traffic_light_id(&self) -> String {
        maliput_sys::api::rules::ffi::UniqueBulbId_traffic_light_id(&self.unique_bulb_id)
    }

    /// Get the bulb group id of the [UniqueBulbId].
    ///
    /// # Returns
    /// The bulb group id of the [UniqueBulbId].
    pub fn bulb_group_id(&self) -> String {
        maliput_sys::api::rules::ffi::UniqueBulbId_bulb_group_id(&self.unique_bulb_id)
    }

    /// Get the bulb id of the [UniqueBulbId].
    ///
    /// # Returns
    /// The bulb id of the [UniqueBulbId].
    pub fn bulb_id(&self) -> String {
        maliput_sys::api::rules::ffi::UniqueBulbId_bulb_id(&self.unique_bulb_id)
    }

    /// Get the string representation of the [UniqueBulbId].
    ///
    /// # Returns
    /// The string representation of the [UniqueBulbId].
    pub fn string(&self) -> String {
        self.unique_bulb_id.string().to_string()
    }
}

/// Uniquely identifies a bulb group in the `Inertial` space. This consists of
/// the concatenation of the ID of the bulb group, and the ID of the traffic
/// light that contains the bulb group.
///
/// String representation of this ID is:
/// "`traffic_light_id().string()`-`bulb_group_id.string()`"
pub struct UniqueBulbGroupId {
    unique_bulb_group_id: cxx::UniquePtr<maliput_sys::api::rules::ffi::UniqueBulbGroupId>,
}

impl UniqueBulbGroupId {
    /// Get the traffic light id of the [UniqueBulbGroupId].
    ///
    /// # Returns
    /// The traffic light id of the [UniqueBulbGroupId].
    pub fn traffic_light_id(&self) -> String {
        maliput_sys::api::rules::ffi::UniqueBulbGroupId_traffic_light_id(&self.unique_bulb_group_id)
    }

    /// Get the bulb group id of the [UniqueBulbGroupId].
    ///
    /// # Returns
    /// The bulb group id of the [UniqueBulbGroupId].
    pub fn bulb_group_id(&self) -> String {
        maliput_sys::api::rules::ffi::UniqueBulbGroupId_bulb_group_id(&self.unique_bulb_group_id)
    }

    /// Get the string representation of the [UniqueBulbGroupId].
    ///
    /// # Returns
    /// The string representation of the [UniqueBulbGroupId].
    pub fn string(&self) -> String {
        self.unique_bulb_group_id.string().to_string()
    }
}

/// Interface for querying types of rules. It includes both Discrete and Range value rules. It
/// provides a registry of the various rule types.
pub struct RuleRegistry<'a> {
    pub(super) rule_registry: &'a maliput_sys::api::rules::ffi::RuleRegistry,
}

/// Represents the rule values the [RuleRegistry] can contain by their Discrete or Range type.
pub enum RuleValuesByType {
    DiscreteValues(Vec<DiscreteValue>),
    Ranges(Vec<Range>),
}

impl<'a> RuleRegistry<'a> {
    /// Returns all [DiscreteValue] rule type IDs.
    ///
    /// # Returns
    /// A vector of [String]s representing rule type IDs that correspond to different
    /// [DiscreteValue]s in the [RuleRegistry].
    pub fn get_discrete_value_rule_types(&self) -> Vec<String> {
        let discrete_value_types =
            maliput_sys::api::rules::ffi::RuleRegistry_DiscreteValueRuleTypes(self.rule_registry);
        let discrete_value_types = discrete_value_types
            .as_ref()
            .expect("Unable to get underlying discrete value rule types pointer.");
        discrete_value_types.iter().map(|dvt| dvt.type_id.clone()).collect()
    }

    /// Returns all [DiscreteValue]s corresponding to the specified `rule_type_id`.
    ///
    /// This methods works in tandem with [RuleRegistry::get_discrete_value_rule_types].
    ///
    /// # Arguments
    /// * `rule_type_id` - The id of the rule type.
    ///
    /// # Returns
    /// A vector of [DiscreteValue]s or [None] if the `rule_type_id` doesn't match any type id in
    /// the [RuleRegistry].
    pub fn discrete_values_by_type(&self, rule_type_id: String) -> Option<Vec<DiscreteValue>> {
        let discrete_value_types =
            maliput_sys::api::rules::ffi::RuleRegistry_DiscreteValueRuleTypes(self.rule_registry);
        let discrete_value_types = discrete_value_types
            .as_ref()
            .expect("Unable to get underlying discrete value rule types pointer.");
        discrete_value_types
            .iter()
            .find(|dvt| dvt.type_id == rule_type_id)
            .map(|dvt| discrete_values_from_cxx(&dvt.values))
    }

    /// Returns all [Range] rule type IDs.
    ///
    /// # Returns
    /// A vector of [String]s representing rule type IDs that correspond to different [Range]s in
    /// the [RuleRegistry].
    pub fn get_range_rule_types(&self) -> Vec<String> {
        let range_value_types = maliput_sys::api::rules::ffi::RuleRegistry_RangeValueRuleTypes(self.rule_registry);
        let range_value_types = range_value_types
            .as_ref()
            .expect("Unable to get underlying range rule types pointer.");
        range_value_types.iter().map(|rvt| rvt.type_id.clone()).collect()
    }

    /// Returns all [Range]s corresponding to the specified `rule_type_id`.
    ///
    /// This methods works in tandem with [RuleRegistry::get_range_rule_types].
    ///
    /// # Arguments
    /// * `rule_type_id` - The id of the rule type.
    ///
    /// # Returns
    /// A vector of [Range]s or [None] if the `rule_type_id` doesn't match any type id in the
    /// [RuleRegistry].
    pub fn range_values_by_type(&self, rule_type_id: String) -> Option<Vec<Range>> {
        let range_value_types = maliput_sys::api::rules::ffi::RuleRegistry_RangeValueRuleTypes(self.rule_registry);
        let range_value_types = range_value_types
            .as_ref()
            .expect("Unable to get underlying range rule types pointer.");
        range_value_types
            .iter()
            .find(|rvt| rvt.type_id == rule_type_id)
            .map(|rvt| range_values_from_cxx(&rvt.values))
    }

    /// Returns all possible states for a given `rule_type_id`.
    ///
    /// # Arguments
    /// * `rule_type_id` - The id of the rule type.
    ///
    /// # Returns
    /// An `Option` containing a [RuleValuesByType] enum with either a vector of [Range]s or a
    /// vector of [DiscreteValue]s. Returns `None` if the `rule_type_id` is not found.
    pub fn get_possible_states_of_rule_type(&self, rule_type_id: String) -> Option<RuleValuesByType> {
        if let Some(ranges) = self.range_values_by_type(rule_type_id.clone()) {
            Some(RuleValuesByType::Ranges(ranges))
        } else {
            self.discrete_values_by_type(rule_type_id)
                .map(RuleValuesByType::DiscreteValues)
        }
    }
}

/// Abstraction for holding the output of [RoadRulebook::rules()] and [RoadRulebook::find_rules()]
/// methods.
/// This struct contains a map of [DiscreteValueRule]s and [RangeValueRule]s.
/// The keys of the map are the ids of the rules.
/// The values of the map are the rules.
pub struct QueryResults {
    pub discrete_value_rules: std::collections::HashMap<String, DiscreteValueRule>,
    pub range_value_rules: std::collections::HashMap<String, RangeValueRule>,
}

/// Interface for querying "rules of the road". This interface
/// provides access to static information about a road network (i.e.,
/// information determined prior to the beginning of a simulation). Some
/// rule types may refer to additional dynamic information which will be
/// provided by other interfaces.
pub struct RoadRulebook<'a> {
    pub(super) road_rulebook: &'a maliput_sys::api::rules::ffi::RoadRulebook,
}

impl<'a> RoadRulebook<'a> {
    /// Returns the DiscreteValueRule with the specified `id`.
    ///
    /// # Arguments
    /// * `rule_id` - The id of the rule.
    ///
    /// # Returns
    /// The DiscreteValueRule with the given id or None if the id is not in the Rulebook.
    pub fn get_discrete_value_rule(&self, rule_id: &String) -> Option<DiscreteValueRule> {
        let discrete_value_rule =
            maliput_sys::api::rules::ffi::RoadRulebook_GetDiscreteValueRule(self.road_rulebook, rule_id);
        if discrete_value_rule.is_null() {
            return None;
        }
        Some(DiscreteValueRule { discrete_value_rule })
    }
    /// Returns the RangeValueRule with the specified `id`.
    ///
    /// # Arguments
    /// * `rule_id` - The id of the rule.
    ///
    /// # Returns
    /// The RangeValueRule with the given id or None if the id is not in the Rulebook.
    pub fn get_range_value_rule(&self, rule_id: &String) -> Option<RangeValueRule> {
        let range_value_rule =
            maliput_sys::api::rules::ffi::RoadRulebook_GetRangeValueRule(self.road_rulebook, rule_id);
        if range_value_rule.is_null() {
            return None;
        }
        Some(RangeValueRule { range_value_rule })
    }

    /// Returns all the rules in the road rulebook.
    ///
    /// # Returns
    /// A [QueryResults] containing all the rules in the road rulebook.
    pub fn rules(&self) -> QueryResults {
        let query_results_cpp = maliput_sys::api::rules::ffi::RoadRulebook_Rules(self.road_rulebook);
        let discrete_value_rules_id =
            maliput_sys::api::rules::ffi::QueryResults_discrete_value_rules(&query_results_cpp);
        let range_value_rules_id = maliput_sys::api::rules::ffi::QueryResults_range_value_rules(&query_results_cpp);
        let mut dvr_map = std::collections::HashMap::new();
        for rule_id in discrete_value_rules_id {
            // It is okay to unwrap here since we are iterating valid IDs obtained above.
            let rule = self.get_discrete_value_rule(&rule_id).unwrap();
            dvr_map.insert(rule.id(), rule);
        }
        let mut rvr_map = std::collections::HashMap::new();
        for rule_id in range_value_rules_id {
            // It is okay to unwrap here since we are iterating valid IDs obtained above.
            let rule = self.get_range_value_rule(&rule_id).unwrap();
            rvr_map.insert(rule.id(), rule);
        }
        QueryResults {
            discrete_value_rules: dvr_map,
            range_value_rules: rvr_map,
        }
    }

    /// Finds rules that apply to the given lane s ranges.
    ///
    /// # Arguments
    /// * `ranges` - A vector of [super::LaneSRange]s to find rules for.
    /// * `tolerance` - A tolerance value to use when finding rules.
    ///
    /// # Returns
    /// A [QueryResults] containing the rules that apply to the given lane s ranges.
    /// If no rules are found, an empty [QueryResults] is returned.
    ///
    /// # Errors
    /// Returns a [MaliputError] if the underlying C++ function fails.
    pub fn find_rules(&self, ranges: &Vec<super::LaneSRange>, tolerance: f64) -> Result<QueryResults, MaliputError> {
        let mut ranges_cpp = Vec::new();
        for range in ranges {
            ranges_cpp.push(maliput_sys::api::rules::ffi::ConstLaneSRangeRef {
                lane_s_range: &range.lane_s_range,
            });
        }
        let query_results_cpp =
            maliput_sys::api::rules::ffi::RoadRulebook_FindRules(self.road_rulebook, &ranges_cpp, tolerance)?;

        let discrete_value_rules_id =
            maliput_sys::api::rules::ffi::QueryResults_discrete_value_rules(&query_results_cpp);
        let range_value_rules_id = maliput_sys::api::rules::ffi::QueryResults_range_value_rules(&query_results_cpp);
        let mut dvr_map = std::collections::HashMap::new();
        for rule_id in discrete_value_rules_id {
            if let Some(rule) = self.get_discrete_value_rule(&rule_id) {
                dvr_map.insert(rule.id(), rule);
            }
        }
        let mut rvr_map = std::collections::HashMap::new();
        for rule_id in range_value_rules_id {
            if let Some(rule) = self.get_range_value_rule(&rule_id) {
                rvr_map.insert(rule.id(), rule);
            }
        }
        Ok(QueryResults {
            discrete_value_rules: dvr_map,
            range_value_rules: rvr_map,
        })
    }
}

/// # Rule
///
/// A Rule may have multiple states that affect agent behavior while it is
/// driving through the rule's zone. The possible states of a Rule must be
/// semantically coherent. The current state of a Rule is given by a
/// [RuleStateProvider]. States can be:
///
/// - range based ([RangeValueRule]).
/// - discrete ([DiscreteValueRule]).
///
/// # DiscreteValueRule
///
/// [DiscreteValue]s are defined by a string value.
/// Semantics of this rule are based on _all_ possible values that this
/// [DiscreteValueRule::type_id] could have (as specified by RuleRegistry::FindRuleByType()),
/// not only the subset of values that a specific instance of this rule can
/// be in.
pub struct DiscreteValueRule {
    discrete_value_rule: cxx::UniquePtr<maliput_sys::api::rules::ffi::DiscreteValueRule>,
}

impl DiscreteValueRule {
    /// Returns the Id of the rule as a string.
    ///
    /// # Returns
    /// The id of the rule.
    pub fn id(&self) -> String {
        maliput_sys::api::rules::ffi::DiscreteValueRule_id(&self.discrete_value_rule)
    }
    /// Returns the type of the rule as a string.
    /// Example: "right-of-way-rule-type-id", "direction-usage-rule-type-id"
    ///
    /// # Returns
    /// The type id of the rule.
    pub fn type_id(&self) -> String {
        maliput_sys::api::rules::ffi::DiscreteValueRule_type_id(&self.discrete_value_rule)
    }
    /// Returns a [super::LaneSRoute] that represents the zone that the rule applies to.
    ///
    /// # Returns
    /// A [super::LaneSRoute] representing the zone of the rule.
    pub fn zone(&self) -> super::LaneSRoute {
        let lane_s_route = maliput_sys::api::rules::ffi::DiscreteValueRule_zone(&self.discrete_value_rule);
        super::LaneSRoute { lane_s_route }
    }
    /// Returns the states of the rule.
    ///
    /// # Returns
    /// A vector of [DiscreteValue]s representing the states of the rule.
    /// If the rule has no states, an empty vector is returned.
    pub fn states(&self) -> Vec<DiscreteValue> {
        discrete_values_from_cxx(self.discrete_value_rule.states())
    }
}

impl std::fmt::Debug for DiscreteValueRule {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "DiscreteValueRule {{ id: {}, type_id: {}, zone: {:?}, states: {:?} }}",
            self.id(),
            self.type_id(),
            self.zone(),
            self.states()
        )
    }
}

/// Holds a `Rule` ID and the current state of that `Rule`.
/// It is usually used as a return type for [super::Intersection::discrete_value_rule_states].
pub struct DiscreteValueRuleState {
    /// Rule ID.
    pub rule_id: String,
    /// Current state of the rule.
    pub state: DiscreteValue,
}

/// # Rule
///
/// A Rule may have multiple states that affect agent behavior while it is
/// driving through the rule's zone. The possible states of a Rule must be
/// semantically coherent. The current state of a Rule is given by a
/// [RuleStateProvider]. States can be:
///
/// - range based ([RangeValueRule]).
/// - discrete ([DiscreteValueRule]).
///
/// # RangeValueRule
///
/// [Range]s describe a numeric range based rule.
/// Ranges are closed and continuous, defined by a minimum and maximum quantity.
/// When only one extreme is formally defined, the other should take a
/// semantically correct value. For example, if a speed limit only specifies a
/// maximum value, the minimum value is typically zero.
pub struct RangeValueRule {
    range_value_rule: cxx::UniquePtr<maliput_sys::api::rules::ffi::RangeValueRule>,
}

impl RangeValueRule {
    /// Returns the Id of the rule as a string.
    ///
    /// # Returns
    /// The id of the rule.
    pub fn id(&self) -> String {
        maliput_sys::api::rules::ffi::RangeValueRule_id(&self.range_value_rule)
    }
    /// Returns the type of the rule as a string.
    /// Example: "right-of-way-rule-type-id", "direction-usage-rule-type-id"
    ///
    /// # Returns
    /// The type id of the rule.
    pub fn type_id(&self) -> String {
        maliput_sys::api::rules::ffi::RangeValueRule_type_id(&self.range_value_rule)
    }
    /// Returns a [super::LaneSRoute] that represents the zone that the rule applies to.
    ///
    /// # Returns
    /// A [super::LaneSRoute] representing the zone of the rule.
    pub fn zone(&self) -> super::LaneSRoute {
        let lane_s_route = maliput_sys::api::rules::ffi::RangeValueRule_zone(&self.range_value_rule);
        super::LaneSRoute { lane_s_route }
    }
    /// Returns the states of the rule.
    ///
    /// # Returns
    /// A vector of [Range]s representing the states of the rule.
    /// If the rule has no states, an empty vector is returned.
    pub fn states(&self) -> Vec<Range> {
        range_values_from_cxx(self.range_value_rule.states())
    }
}

/// Defines a Rule Type.
///
/// # RuleType
///
/// [RuleType]s provide a way of obtaining a rule type's string defined in
/// maliput's backend. Since new rule types can be created in a custom manner,
/// [RuleType] only holds the most common types which are already defined in
/// the backend.
#[derive(Display, IntoStaticStr)]
pub enum RuleType {
    #[strum(serialize = "Direction-Usage Rule Type")]
    DirectionUsage,
    #[strum(serialize = "Right-Of-Way Rule Type")]
    RightOfWay,
    #[strum(serialize = "Vehicle-Stop-In-Zone-Behavior Rule Type")]
    VehicleStopInZoneBehavior,
    #[strum(serialize = "Speed-Limit Rule Type")]
    SpeedLimit,
}

impl RuleType {
    /// Gets the Rule ID for the [RuleType] and `lane_id`.
    ///
    /// # Arguments
    /// - `lane_id` - The lane ID to get the rule ID from.
    ///
    /// # Returns
    /// A rule ID formatted the way the backend defines it.
    pub fn get_rule_id(&self, lane_id: &str) -> String {
        // We rely on maliput_malidrive which define the rule id as:
        // "<rule_type>/<lane_id>"
        self.to_string() + "/" + lane_id
    }
}

/// Defines a base state for a rule.
///
/// # RuleStateBase
///
/// - `severity` - The severity of the rule state.
/// - `related_rules` - A map of related rules. The key is the group name and the value is a vector of rule ids.
/// - `related_unique_ids` - A map of related unique ids. The key is the group name and the value is a vector of unique ids.
///
/// See [DiscreteValueRule] and [RangeValueRule] for more information.
pub struct RuleStateBase {
    /// Severity of the rule's state. A non-negative quantity that specifies the
    /// level of enforcement. The smaller it is, the more strictly the rule is
    /// enforced. Each rule type can define its own set of severity level
    /// semantics.
    severity: i32,
    related_rules: cxx::UniquePtr<cxx::CxxVector<maliput_sys::api::rules::ffi::RelatedRule>>,
    related_unique_ids: cxx::UniquePtr<cxx::CxxVector<maliput_sys::api::rules::ffi::RelatedUniqueId>>,
}

/// A trait representing a possible state of a `Rule`.
///
/// A `Rule` can have multiple states that affect agent behavior. This trait
/// provides a common interface for accessing the properties shared by all
/// rule states, such as severity and related rules.
///
/// This trait is implemented by specific state types like [`DiscreteValue`]
/// and [`Range`].
///
/// # Implementors
///
/// When implementing this trait, you must provide an implementation for the
/// [`get_rule_state()`] method, which gives access to the underlying
/// [`RuleStateBase`] data. The other methods have default implementations.
pub trait RuleState {
    /// Gets the underlying [`RuleStateBase`] that contains common state properties.
    ///
    /// # Returns
    /// A reference to the [`RuleStateBase`] that contains the severity, related rules,
    /// and related unique ids for the rule state.
    fn get_rule_state(&self) -> &RuleStateBase;

    /// Returns the severity of the rule state.
    ///
    /// # Returns
    /// An `i32` representing the severity of the rule state.
    /// The severity is a numeric value that indicates the importance or urgency of the rule. The lower the value, the more strictly the rule is enforced.
    fn severity(&self) -> i32 {
        self.get_rule_state().severity
    }

    /// Returns a map of related rules ids. The key is the group name and the value is a vector of rule ids.
    ///
    /// # Returns
    /// A map of related rules where the key is the group name and the value is a vector of rule ids.
    fn related_rules(&self) -> std::collections::HashMap<&String, &Vec<String>> {
        self.get_rule_state()
            .related_rules
            .iter()
            .map(|rr| (&rr.group_name, &rr.rule_ids))
            .collect::<std::collections::HashMap<&String, &Vec<String>>>()
    }
    /// Returns a map of related unique ids. The key is the group name and the value is a vector of unique ids.
    ///
    /// # Returns
    /// A map of related unique ids where the key is the group name and the value is a vector of unique ids.
    fn related_unique_ids(&self) -> std::collections::HashMap<&String, &Vec<String>> {
        self.get_rule_state()
            .related_unique_ids
            .iter()
            .map(|rui| (&rui.group_name, &rui.unique_ids))
            .collect::<std::collections::HashMap<&String, &Vec<String>>>()
    }
}

/// Defines a discrete value for a [DiscreteValueRule].
/// It extends the [RuleStateBase] with the value of the discrete value.
pub struct DiscreteValue {
    rule_state: RuleStateBase,
    value: String,
}

impl RuleState for DiscreteValue {
    fn get_rule_state(&self) -> &RuleStateBase {
        &self.rule_state
    }
}

impl DiscreteValue {
    /// Returns the value of the discrete value.
    pub fn value(&self) -> &String {
        &self.value
    }
}

impl std::fmt::Debug for DiscreteValue {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "DiscreteValue {{ value: {}, severity: {}, related_rules: {:?}, related_unique_ids: {:?} }}",
            self.value(),
            self.severity(),
            self.related_rules(),
            self.related_unique_ids()
        )
    }
}

/// Defines a range value for a [RangeValueRule].
/// It extends the [RuleStateBase] with the description, and min and max values of the range.
pub struct Range {
    rule_state: RuleStateBase,
    description: String,
    min: f64,
    max: f64,
}

impl RuleState for Range {
    fn get_rule_state(&self) -> &RuleStateBase {
        &self.rule_state
    }
}

impl Range {
    /// Returns the description of the range value.
    pub fn description(&self) -> &String {
        &self.description
    }
    /// Returns the minimum value of the range.
    pub fn min(&self) -> f64 {
        self.min
    }
    /// Returns the maximum value of the range.
    pub fn max(&self) -> f64 {
        self.max
    }
}

/// Defines a phase in a traffic rule system.
///
/// A phase represents a specific state or configuration of traffic signals
/// and semantic rules within a traffic control system. Each phase has a unique
/// identifier and may include various traffic signal states and rule configurations
/// that dictate how traffic should behave during that phase.
pub struct Phase {
    phase: cxx::UniquePtr<maliput_sys::api::rules::ffi::Phase>,
}

impl Phase {
    /// Gets the id of the [Phase].
    ///
    /// # Returns
    /// The id of the [Phase].
    pub fn id(&self) -> String {
        maliput_sys::api::rules::ffi::Phase_id(&self.phase)
    }

    /// Gets the states of all discrete value rules for this phase.
    ///
    /// # Returns
    /// A `HashMap` where the key is the rule ID as a [String] and the value is the
    /// [DiscreteValue] state of that rule.
    pub fn discrete_value_rule_states(&self) -> HashMap<String, DiscreteValue> {
        let rule_states = maliput_sys::api::rules::ffi::Phase_discrete_value_rule_states(&self.phase);
        rule_states
            .iter()
            .map(|state| {
                (
                    state.rule_id.clone(),
                    discrete_value_from_discrete_value_cxx(&state.state),
                )
            })
            .collect()
    }

    /// Obtains all [UniqueBulbId]s in the [Phase].
    ///
    /// # Returns
    /// A vector of [UniqueBulbId].
    pub fn unique_bulb_ids(&self) -> Vec<UniqueBulbId> {
        let unique_bulb_ids = maliput_sys::api::rules::ffi::Phase_unique_bulb_ids(&self.phase);
        unique_bulb_ids
            .iter()
            .map(|bulb_id| UniqueBulbId {
                unique_bulb_id: maliput_sys::api::rules::ffi::ptr_from_unique_bulb_id(bulb_id),
            })
            .collect()
    }

    /// Returns the [BulbState] corresponding to a `bulb_id`.
    ///
    /// # Arguments
    /// * `unique_bulb_id` - The [UniqueBulbId] to get the [BulbState] from.
    ///
    /// # Returns
    /// The [BulbState] the `unique_bulb_id` is in, or [None] if the [UniqueBulbId] is not in this [Phase].
    pub fn bulb_state(&self, unique_bulb_id: &UniqueBulbId) -> Option<BulbState> {
        let bulb_state = maliput_sys::api::rules::ffi::Phase_bulb_state(&self.phase, &unique_bulb_id.unique_bulb_id);
        if bulb_state.is_null() {
            return None;
        }
        Some(match *bulb_state {
            maliput_sys::api::rules::ffi::BulbState::kOff => BulbState::Off,
            maliput_sys::api::rules::ffi::BulbState::kOn => BulbState::On,
            maliput_sys::api::rules::ffi::BulbState::kBlinking => BulbState::Blinking,
            maliput_sys::api::rules::ffi::BulbState::kCounting => BulbState::Counting,
            _ => return None,
        })
    }
}

/// Defines a phase that comes after another [Phase].
/// Used as a return type by:
///   - [PhaseRing::get_next_phases].
pub struct NextPhase {
    /// The next phase.
    pub next_phase: Phase,
    /// The default time before transitioning to the next phase. This is
    /// relative to when the current phase began. It is just a recommendation,
    /// the actual duration is determined by the PhaseProvider and may depend on
    /// events like a vehicle arriving at a left-turn lane or a pedestrian
    /// hitting a crosswalk button.
    pub duration_until: Option<f64>,
}

/// Defines a ring of phases in a traffic rule system.
///
/// A phase ring represents a sequence of phases that a traffic control system
/// cycles through.
pub struct PhaseRing {
    phase_ring: cxx::UniquePtr<maliput_sys::api::rules::ffi::PhaseRing>,
}

impl PhaseRing {
    /// Gets the id of the [PhaseRing].
    ///
    /// # Returns
    /// The id of the [PhaseRing].
    pub fn id(&self) -> String {
        maliput_sys::api::rules::ffi::PhaseRing_id(&self.phase_ring)
    }

    /// Gets a [Phase] by its id
    ///
    /// # Arguments
    /// * `id` - The id of the [Phase].
    /// # Returns
    /// The [Phase] with the given id.
    /// If no [Phase] is found with the given id, return None.
    pub fn get_phase(&self, id: &String) -> Option<Phase> {
        let phase = maliput_sys::api::rules::ffi::PhaseRing_GetPhase(&self.phase_ring, id);
        if phase.is_null() {
            return None;
        }
        Some(Phase { phase })
    }

    /// Returns the ids of all Phases in the PhaseRing.
    ///
    /// # Returns
    /// A vector of strings representing the ids of all Phases in the PhaseRing.
    pub fn phases(&self) -> Vec<String> {
        maliput_sys::api::rules::ffi::PhaseRing_phases_ids(&self.phase_ring)
    }

    /// Returns the next phases for a given phase `id`.
    ///
    /// # Arguments
    /// * `id` - The id of the phase to get the next phases from.
    ///
    /// # Returns
    /// A `Result` containing a vector of [NextPhase]s.
    ///
    /// # Errors
    /// Returns a [MaliputError] if the provided `id` is not found in the [PhaseRing].
    pub fn get_next_phases(&self, id: &String) -> Result<Vec<NextPhase>, MaliputError> {
        let next_phases = maliput_sys::api::rules::ffi::PhaseRing_GetNextPhases(&self.phase_ring, id)?;
        Ok(next_phases
            .iter()
            .map(|np| NextPhase {
                next_phase: Phase {
                    phase: maliput_sys::api::rules::ffi::PhaseRing_GetPhase(&self.phase_ring, &np.phase_id),
                },
                duration_until: if np.duration_until.is_null() {
                    None
                } else {
                    Some(np.duration_until.value)
                },
            })
            .collect())
    }
}

/// Defines a book of phase rings in a traffic rule system.
pub struct PhaseRingBook<'a> {
    pub(super) phase_ring_book: &'a maliput_sys::api::rules::ffi::PhaseRingBook,
}

impl<'a> PhaseRingBook<'a> {
    /// Returns the ids of all PhaseRings in the PhaseRingBook.
    ///
    /// # Returns
    /// A vector of strings representing the ids of all PhaseRings in the PhaseRingBook.
    pub fn get_phase_rings_ids(&self) -> Vec<String> {
        maliput_sys::api::rules::ffi::PhaseRingBook_GetPhaseRingsId(self.phase_ring_book)
    }

    /// Returns the PhaseRing with the specified `id`.
    ///
    /// # Arguments
    /// * `phase_ring_id` - The id of the phase ring.
    ///
    /// # Returns
    /// The PhaseRing with the given id or None if the id is not in the PhaseRingBook.
    pub fn get_phase_ring(&self, phase_ring_id: &String) -> Option<PhaseRing> {
        let phase_ring = maliput_sys::api::rules::ffi::PhaseRingBook_GetPhaseRing(self.phase_ring_book, phase_ring_id);
        if phase_ring.is_null() {
            return None;
        }
        Some(PhaseRing { phase_ring })
    }

    /// Finds the [PhaseRing] that contains the rule with the specified `rule_id`.
    ///
    /// # Arguments
    /// * `rule_id` - The id of the rule.
    ///
    /// # Returns
    /// The [PhaseRing] that contains the rule with the given id or `None` if no [PhaseRing] is found.
    pub fn find_phase_ring(&self, rule_id: &String) -> Option<PhaseRing> {
        let phase_ring = maliput_sys::api::rules::ffi::PhaseRingBook_FindPhaseRing(self.phase_ring_book, rule_id);
        if phase_ring.is_null() {
            return None;
        }
        Some(PhaseRing { phase_ring })
    }
}

/// Defines a next state of a generic type.
pub struct NextState<T> {
    /// The next state.
    pub next_state: T,
    /// The default time before transitioning to the next state. This is
    /// relative to when the current state began. It is just a recommendation,
    /// the actual duration is determined by the StateProvider and may depend on
    /// events like a vehicle arriving at a left-turn lane or a pedestrian
    /// hitting a crosswalk button.
    pub duration_until: Option<f64>,
}

/// Holds the current and possible next state of a system.
/// It is usually returned by the different types of state providers.
pub struct StateProviderQuery<T> {
    /// The current state.
    pub state: T,
    /// The next state.
    pub next: Option<NextState<T>>,
}

/// Alias for the [StateProviderQuery] returned by [PhaseProvider::get_phase].
pub type PhaseStateProviderQuery = StateProviderQuery<String>;

/// Defines a phase provider.
///
/// A phase provider is able to get the current phase from a phase-based system.
pub struct PhaseProvider<'a> {
    pub(super) phase_provider: &'a maliput_sys::api::rules::ffi::PhaseProvider,
}

impl<'a> PhaseProvider<'a> {
    /// Returns the [PhaseStateProviderQuery] for the specified `phase_ring_id`.
    ///
    /// The states are represented with Strings containing the IDs of each [Phase].
    ///
    /// # Arguments
    /// * `phase_ring_id` - The id of the phase ring.
    ///
    /// # Returns
    /// An `Option` containing the [PhaseStateProviderQuery] for the given `phase_ring_id`.
    /// Returns `None` if no phase provider is found for the given id.
    pub fn get_phase(&self, phase_ring_id: &String) -> Option<PhaseStateProviderQuery> {
        let phase_state = maliput_sys::api::rules::ffi::PhaseProvider_GetPhase(self.phase_provider, phase_ring_id);
        if phase_state.is_null() {
            return None;
        }

        let next_state = maliput_sys::api::rules::ffi::PhaseStateProvider_next(&phase_state);
        let next_phase = if next_state.is_null() {
            None
        } else {
            Some(NextState {
                next_state: next_state.phase_id.clone(),
                duration_until: if next_state.duration_until.is_null() {
                    None
                } else {
                    Some(next_state.duration_until.value)
                },
            })
        };

        Some(StateProviderQuery {
            state: maliput_sys::api::rules::ffi::PhaseStateProvider_state(&phase_state),
            next: next_phase,
        })
    }
}

/// Provides the dynamic state of [DiscreteValueRule]s.
///
/// While a [RoadRulebook] provides the static definitions of rules, a
/// `DiscreteValueRuleStateProvider` provides the current state of those rules
/// at runtime. This allows for querying what state a rule is currently in,
/// which is essential for dynamic systems where rule states can change over
/// time (e.g., traffic light phases changing).
pub struct DiscreteValueRuleStateProvider<'a> {
    pub(super) state_provider: &'a maliput_sys::api::rules::ffi::DiscreteValueRuleStateProvider,
}

impl<'a> DiscreteValueRuleStateProvider<'a> {
    /// Gets a state from the provider based on it's `rule_id`.
    ///
    /// # Arguments
    /// * `rule_id` - A Rule ID.
    ///
    /// # Returns
    /// An Option containing the [StateProviderQuery] with a [DiscreteValue] if the `rule_id` matches with any rule.
    /// Otherwise, None is returned.
    pub fn get_state_by_rule_id(&self, rule_id: &String) -> Option<StateProviderQuery<DiscreteValue>> {
        let query_state =
            maliput_sys::api::rules::ffi::DiscreteValueRuleStateProvider_GetStateById(self.state_provider, rule_id);
        Self::next_state_from_cxx_query(query_state)
    }

    /// Gets a state from the provider if there is a `rule_type` in the received `road_position`.
    ///
    /// # Arguments
    /// * `road_position` - A position in the road geometry.
    /// * `rule_type` - A Rule Type.
    /// * `tolerance` - The tolerance in which to look for the Rule of type `rule_type` around the `road_position`.
    ///
    /// # Returns
    /// An Option containing the [StateProviderQuery] with a [DiscreteValue] if `rule_type` matches with any rule's type near `road_position`.
    /// Otherwise, None is returned.
    pub fn get_state_by_rule_type(
        &self,
        road_position: &RoadPosition,
        rule_type: RuleType,
        tolerance: f64,
    ) -> Option<StateProviderQuery<DiscreteValue>> {
        let query_state = maliput_sys::api::rules::ffi::DiscreteValueRuleStateProvider_GetStateByType(
            self.state_provider,
            &road_position.rp,
            &rule_type.to_string(),
            tolerance,
        );
        Self::next_state_from_cxx_query(query_state)
    }

    // Internal helper to avoid code duplication.
    fn next_state_from_cxx_query(
        query_state: cxx::UniquePtr<maliput_sys::api::rules::ffi::DiscreteValueRuleStateProviderQuery>,
    ) -> Option<StateProviderQuery<DiscreteValue>> {
        if query_state.is_null() {
            return None;
        }
        let next_state = maliput_sys::api::rules::ffi::DiscreteValueRuleStateProviderQuery_next(&query_state);
        Some(StateProviderQuery {
            state: discrete_value_from_discrete_value_cxx(
                &maliput_sys::api::rules::ffi::DiscreteValueRuleStateProviderQuery_state(&query_state),
            ),
            next: if next_state.is_null() {
                None
            } else {
                Some(NextState {
                    next_state: discrete_value_from_discrete_value_cxx(&next_state.state),
                    duration_until: if next_state.duration_until.is_null() {
                        None
                    } else {
                        Some(next_state.duration_until.value)
                    },
                })
            },
        })
    }
}

/// Provides the dynamic state of [RangeValueRule]s.
///
/// While a [RoadRulebook] provides the static definitions of rules, a
/// `RangeValueRuleStateProvider` provides the current state of those rules
/// at runtime. This allows for querying what state a rule is currently in,
/// which is essential for dynamic systems where rule states can change over
/// time (e.g., variable speed limits based on types of roads).
pub struct RangeValueRuleStateProvider<'a> {
    pub(super) state_provider: &'a maliput_sys::api::rules::ffi::RangeValueRuleStateProvider,
}

impl<'a> RangeValueRuleStateProvider<'a> {
    /// Gets a state from the provider based on it's `rule_id`.
    ///
    /// # Arguments
    /// * `rule_id` - A Rule ID.
    ///
    /// # Returns
    /// An Option containing the [StateProviderQuery] with a [Range] if the `rule_id` matches with any rule.
    /// Otherwise, None is returned.
    pub fn get_state_by_rule_id(&self, rule_id: &String) -> Option<StateProviderQuery<Range>> {
        let query_state =
            maliput_sys::api::rules::ffi::RangeValueRuleStateProvider_GetStateById(self.state_provider, rule_id);
        Self::next_state_from_cxx_query(query_state)
    }

    /// Gets a state from the provider if there is a `rule_type` in the received `road_position`.
    ///
    /// # Arguments
    /// * `road_position` - A position in the road geometry.
    /// * `rule_type` - A Rule Type.
    /// * `tolerance` - The tolerance in which to look for the Rule of type `rule_type` around the `road_position`.
    ///
    /// # Returns
    /// An Option containing the [StateProviderQuery] with a [Range] if `rule_type` matches with any rule's type near `road_position`.
    /// Otherwise, None is returned.
    pub fn get_state_by_rule_type(
        &self,
        road_position: &RoadPosition,
        rule_type: RuleType,
        tolerance: f64,
    ) -> Option<StateProviderQuery<Range>> {
        let query_state = maliput_sys::api::rules::ffi::RangeValueRuleStateProvider_GetStateByType(
            self.state_provider,
            &road_position.rp,
            &rule_type.to_string(),
            tolerance,
        );
        Self::next_state_from_cxx_query(query_state)
    }

    // Internal helper to avoid code duplication.
    fn next_state_from_cxx_query(
        query_state: cxx::UniquePtr<maliput_sys::api::rules::ffi::RangeValueRuleStateProviderQuery>,
    ) -> Option<StateProviderQuery<Range>> {
        if query_state.is_null() {
            return None;
        }
        let next_state = maliput_sys::api::rules::ffi::RangeValueRuleStateProviderQuery_next(&query_state);
        Some(StateProviderQuery {
            state: range_value_from_range_value_cxx(
                &maliput_sys::api::rules::ffi::RangeValueRuleStateProviderQuery_state(&query_state),
            ),
            next: if next_state.is_null() {
                None
            } else {
                Some(NextState {
                    next_state: range_value_from_range_value_cxx(&next_state.state),
                    duration_until: if next_state.duration_until.is_null() {
                        None
                    } else {
                        Some(next_state.duration_until.value)
                    },
                })
            },
        })
    }
}

// Auxiliary method to create a [Vec<Range>] from a [cxx::Vector<RangeValueRuleRange>].
fn range_values_from_cxx(
    range_values_cxx: &cxx::Vector<maliput_sys::api::rules::ffi::RangeValueRuleRange>,
) -> Vec<Range> {
    range_values_cxx
        .iter()
        .map(|range| Range {
            rule_state: RuleStateBase {
                severity: maliput_sys::api::rules::ffi::RangeValueRuleRange_severity(range),
                related_rules: maliput_sys::api::rules::ffi::RangeValueRuleRange_related_rules(range),
                related_unique_ids: maliput_sys::api::rules::ffi::RangeValueRuleRange_related_unique_ids(range),
            },
            description: maliput_sys::api::rules::ffi::RangeValueRuleRange_description(range),
            min: maliput_sys::api::rules::ffi::RangeValueRuleRange_min(range),
            max: maliput_sys::api::rules::ffi::RangeValueRuleRange_max(range),
        })
        .collect()
}

// Auxiliary method to create a [Vec<DiscreteValue>] from a [cxx::Vector<DiscreteValueRuleDiscreteValue>].
fn discrete_values_from_cxx(
    discrete_values_cxx: &cxx::Vector<maliput_sys::api::rules::ffi::DiscreteValueRuleDiscreteValue>,
) -> Vec<DiscreteValue> {
    discrete_values_cxx
        .iter()
        .map(discrete_value_from_discrete_value_cxx)
        .collect()
}

// Auxiliary method to create a [DiscreteValue] from a [maliput_sys::api::rules::ffi::DiscreteValueRuleDiscreteValue].
pub(crate) fn discrete_value_from_discrete_value_cxx(
    discrete_value: &maliput_sys::api::rules::ffi::DiscreteValueRuleDiscreteValue,
) -> DiscreteValue {
    DiscreteValue {
        rule_state: RuleStateBase {
            severity: maliput_sys::api::rules::ffi::DiscreteValueRuleDiscreteValue_severity(discrete_value),
            related_rules: maliput_sys::api::rules::ffi::DiscreteValueRuleDiscreteValue_related_rules(discrete_value),
            related_unique_ids: maliput_sys::api::rules::ffi::DiscreteValueRuleDiscreteValue_related_unique_ids(
                discrete_value,
            ),
        },
        value: maliput_sys::api::rules::ffi::DiscreteValueRuleDiscreteValue_value(discrete_value),
    }
}

// Auxiliary method to create a [Range] from a [maliput_sys::api::rules::ffi::RangeValueRuleRange].
fn range_value_from_range_value_cxx(range: &maliput_sys::api::rules::ffi::RangeValueRuleRange) -> Range {
    Range {
        rule_state: RuleStateBase {
            severity: maliput_sys::api::rules::ffi::RangeValueRuleRange_severity(range),
            related_rules: maliput_sys::api::rules::ffi::RangeValueRuleRange_related_rules(range),
            related_unique_ids: maliput_sys::api::rules::ffi::RangeValueRuleRange_related_unique_ids(range),
        },
        description: maliput_sys::api::rules::ffi::RangeValueRuleRange_description(range),
        min: maliput_sys::api::rules::ffi::RangeValueRuleRange_min(range),
        max: maliput_sys::api::rules::ffi::RangeValueRuleRange_max(range),
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Display, EnumString)]
/// Defines the possible traffic sign types.
pub enum TrafficControlDeviceType {
    None,
    Other,
    Stop,
    Yield,
    SpeedLimit,
    NoEntry,
    OneWay,
    PedestrianCrossing,
    NoLeftTurn,
    NoRightTurn,
    NoUTurn,
    SchoolZone,
    Construction,
    RailroadCrossing,
    NoOvertaking,
    AllWay,
    NoUTurnLeft,
    NoUTurnRight,
    StopLine,
    Crosswalk,
    DangerSpot,
    ZebraCrossing,
    Flight,
    Cattle,
    HorseRiders,
    Amphibians,
    FallingRocks,
    SnowOrIce,
    LooseGravel,
    Waterside,
    Clearance,
    MovableBridge,
    RightBeforeLeftNextIntersection,
    TurnLeft,
    TurnRight,
    DoubleTurnLeft,
    DoubleTurnRight,
    HillDownwards,
    HillUpwards,
    UnevenRoad,
    RoadSlipperyWetOrDirty,
    SideWinds,
    RoadNarrowing,
    RoadNarrowingRight,
    RoadNarrowingLeft,
    RoadWorks,
    TrafficQueues,
    TwoWayTraffic,
    AttentionTrafficLight,
    Pedestrians,
    ChildrenCrossing,
    CycleRoute,
    DeerCrossing,
    UngatedLevelCrossing,
    LevelCrossingMarker,
    RailwayTrafficPriority,
    GiveWay,
    PriorityToOppositeDirection,
    PriorityToOppositeDirectionUpsideDown,
    PrescribedLeftTurn,
    PrescribedRightTurn,
    PrescribedStraight,
    PrescribedRightWay,
    PrescribedLeftWay,
    PrescribedRightTurnAndStraight,
    PrescribedLeftTurnAndStraight,
    PrescribedLeftTurnAndRightTurn,
    PrescribedLeftTurnRightTurnAndStraight,
    Roundabout,
    OnewayLeft,
    OnewayRight,
    PassLeft,
    PassRight,
    SideLaneOpenForTraffic,
    SideLaneClosedForTraffic,
    SideLaneClosingForTraffic,
    BusStop,
    TaxiStand,
    BicyclesOnly,
    HorseRidersOnly,
    PedestriansOnly,
    BicyclesPedestriansSharedOnly,
    BicyclesPedestriansSeparatedLeftOnly,
    BicyclesPedestriansSeparatedRightOnly,
    PedestrianZoneBegin,
    PedestrianZoneEnd,
    BicycleRoadBegin,
    BicycleRoadEnd,
    BusLane,
    BusLaneBegin,
    BusLaneEnd,
    AllProhibited,
    MotorizedMultitrackProhibited,
    TrucksProhibited,
    BicyclesProhibited,
    MotorcyclesProhibited,
    MopedsProhibited,
    HorseRidersProhibited,
    HorseCarriagesProhibited,
    CattleProhibited,
    BusesProhibited,
    CarsProhibited,
    CarsTrailersProhibited,
    TrucksTrailersProhibited,
    TractorsProhibited,
    PedestriansProhibited,
    MotorVehiclesProhibited,
    HazardousGoodsVehiclesProhibited,
    OverWeightVehiclesProhibited,
    VehiclesAxleOverWeightProhibited,
    VehiclesExcessWidthProhibited,
    VehiclesExcessHeightProhibited,
    VehiclesExcessLengthProhibited,
    DoNotEnter,
    SnowChainsRequired,
    WaterPollutantVehiclesProhibited,
    EnvironmentalZoneBegin,
    EnvironmentalZoneEnd,
    PrescribedUTurnLeft,
    PrescribedUTurnRight,
    MinimumDistanceForTrucks,
    SpeedLimitBegin,
    SpeedLimitZoneBegin,
    SpeedLimitZoneEnd,
    MinimumSpeedBegin,
    OvertakingBanBegin,
    OvertakingBanForTrucksBegin,
    SpeedLimitEnd,
    MinimumSpeedEnd,
    OvertakingBanEnd,
    OvertakingBanForTrucksEnd,
    AllRestrictionsEnd,
    NoStopping,
    NoParking,
    NoParkingZoneBegin,
    NoParkingZoneEnd,
    RightOfWayNextIntersection,
    RightOfWayBegin,
    RightOfWayEnd,
    PriorityOverOppositeDirection,
    PriorityOverOppositeDirectionUpsideDown,
    TownBegin,
    TownEnd,
    CarParking,
    CarParkingZoneBegin,
    CarParkingZoneEnd,
    SidewalkHalfParkingLeft,
    SidewalkHalfParkingRight,
    SidewalkParkingLeft,
    SidewalkParkingRight,
    SidewalkPerpendicularHalfParkingLeft,
    SidewalkPerpendicularHalfParkingRight,
    SidewalkPerpendicularParkingLeft,
    SidewalkPerpendicularParkingRight,
    LivingStreetBegin,
    LivingStreetEnd,
    Tunnel,
    EmergencyStoppingLeft,
    EmergencyStoppingRight,
    HighwayBegin,
    HighwayEnd,
    ExpresswayBegin,
    ExpresswayEnd,
    NamedHighwayExit,
    NamedExpresswayExit,
    NamedRoadExit,
    HighwayExit,
    ExpresswayExit,
    OnewayStreet,
    CrossingGuards,
    Deadend,
    DeadendExcludingDesignatedActors,
    FirstAidStation,
    PoliceStation,
    Telephone,
    FillingStation,
    Hotel,
    Inn,
    Kiosk,
    Toilet,
    Chapel,
    TouristInfo,
    RepairService,
    PedestrianUnderpass,
    PedestrianBridge,
    CamperPlace,
    AdvisorySpeedLimitBegin,
    AdvisorySpeedLimitEnd,
    PlaceName,
    TouristAttraction,
    TouristRoute,
    TouristArea,
    ShoulderNotPassableMotorVehicles,
    ShoulderUnsafeTrucksTractors,
    TollBegin,
    TollEnd,
    TollRoad,
    Customs,
    InternationalBorderInfo,
    StreetlightRedBand,
    FederalHighwayRouteNumber,
    HighwayRouteNumber,
    HighwayInterchangeNumber,
    EuropeanRouteNumber,
    FederalHighwayDirectionLeft,
    FederalHighwayDirectionRight,
    PrimaryRoadDirectionLeft,
    PrimaryRoadDirectionRight,
    SecondaryRoadDirectionLeft,
    SecondaryRoadDirectionRight,
    DirectionDesignatedActorsLeft,
    DirectionDesignatedActorsRight,
    RoutingDesignatedActors,
    DirectionToHighwayLeft,
    DirectionToHighwayRight,
    DirectionToLocalDestinationLeft,
    DirectionToLocalDestinationRight,
    ConsolidatedDirections,
    StreetName,
    DirectionPreannouncement,
    DirectionPreannouncementLaneConfig,
    DirectionPreannouncementHighwayEntries,
    HighwayAnnouncement,
    OtherRoadAnnouncement,
    HighwayAnnouncementTruckStop,
    HighwayPreannouncementDirections,
    PoleExit,
    HighwayDistanceBoard,
    DetourLeft,
    DetourRight,
    NumberedDetour,
    DetourBegin,
    DetourEnd,
    DetourRoutingBoard,
    OptionalDetour,
    OptionalDetourRouting,
    RouteRecommendation,
    RouteRecommendationEnd,
    AnnounceLaneTransitionLeft,
    AnnounceLaneTransitionRight,
    AnnounceRightLaneEnd,
    AnnounceLeftLaneEnd,
    AnnounceRightLaneBegin,
    AnnounceLeftLaneBegin,
    AnnounceLaneConsolidation,
    DetourCityBlock,
    Gate,
    PoleWarning,
    TrafficCone,
    MobileLaneClosure,
    ReflectorPost,
    DirectionalBoardWarning,
    GuidingPlate,
    GuidingPlateWedges,
    ParkingHazard,
    TrafficLightGreenArrow,
    Text,
    Space,
    Time,
    Arrow,
    ConstrainedTo,
    Except,
    ValidForDistance,
    PriorityRoadBottomLeftFourWay,
    PriorityRoadTopLeftFourWay,
    PriorityRoadBottomLeftThreeWayStraight,
    PriorityRoadBottomLeftThreeWaySideways,
    PriorityRoadTopLeftThreeWayStraight,
    PriorityRoadBottomRightFourWay,
    PriorityRoadTopRightFourWay,
    PriorityRoadBottomRightThreeWayStraight,
    PriorityRoadBottomRightThreeWaySideway,
    PriorityRoadTopRightThreeWayStraight,
    ValidInDistance,
    StopIn,
    LeftArrow,
    LeftBendArrow,
    RightArrow,
    RightBendArrow,
    Accident,
    Snow,
    Fog,
    RollingHighwayInformation,
    Services,
    TimeRange,
    ParkingDiscTimeRestriction,
    Weight,
    Wet,
    ParkingConstraint,
    NoWaitingSideStripes,
    Rain,
    SnowRain,
    Night,
    Stop4Way,
    Truck,
    TractorsMayBePassed,
    Hazardous,
    Trailer,
    Zone,
    Motorcycle,
    MotorcycleAllowed,
    Car,
    EmergencyLane,
    Unknown,
}

/// Domain alias for traffic sign semantic types.
pub type TrafficSignType = TrafficControlDeviceType;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
/// Defines the unit for a traffic sign's numeric value.
pub enum TrafficSignValueUnit {
    MetersPerSecond,
    KilometersPerHour,
    MilesPerHour,
    Meters,
    Kilometers,
    Feet,
    Miles,
    Percent,
    Kilograms,
    MetricTons,
}

#[derive(Debug, Copy, Clone, PartialEq)]
/// Holds a numeric value and its associated unit for a traffic sign.
pub struct TrafficSignValue {
    pub value: f64,
    pub unit: TrafficSignValueUnit,
}

pub(crate) fn traffic_control_device_type_from_cpp(
    sign_type: &maliput_sys::api::rules::ffi::TrafficControlDeviceType,
) -> TrafficControlDeviceType {
    match *sign_type {
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNone => TrafficSignType::None,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOther => TrafficSignType::Other,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kStop => TrafficSignType::Stop,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kYield => TrafficSignType::Yield,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSpeedLimit => TrafficSignType::SpeedLimit,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNoEntry => TrafficSignType::NoEntry,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOneWay => TrafficSignType::OneWay,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPedestrianCrossing => {
            TrafficSignType::PedestrianCrossing
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNoLeftTurn => TrafficSignType::NoLeftTurn,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNoRightTurn => TrafficSignType::NoRightTurn,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNoUTurn => TrafficSignType::NoUTurn,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSchoolZone => TrafficSignType::SchoolZone,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kConstruction => TrafficSignType::Construction,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRailroadCrossing => TrafficSignType::RailroadCrossing,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNoOvertaking => TrafficSignType::NoOvertaking,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAllWay => TrafficSignType::AllWay,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNoUTurnLeft => TrafficSignType::NoUTurnLeft,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNoUTurnRight => TrafficSignType::NoUTurnRight,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kStopLine => TrafficSignType::StopLine,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCrosswalk => TrafficSignType::Crosswalk,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDangerSpot => TrafficSignType::DangerSpot,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kZebraCrossing => TrafficSignType::ZebraCrossing,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kFlight => TrafficSignType::Flight,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCattle => TrafficSignType::Cattle,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHorseRiders => TrafficSignType::HorseRiders,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAmphibians => TrafficSignType::Amphibians,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kFallingRocks => TrafficSignType::FallingRocks,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSnowOrIce => TrafficSignType::SnowOrIce,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kLooseGravel => TrafficSignType::LooseGravel,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kWaterside => TrafficSignType::Waterside,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kClearance => TrafficSignType::Clearance,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kMovableBridge => TrafficSignType::MovableBridge,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRightBeforeLeftNextIntersection => {
            TrafficSignType::RightBeforeLeftNextIntersection
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTurnLeft => TrafficSignType::TurnLeft,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTurnRight => TrafficSignType::TurnRight,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDoubleTurnLeft => TrafficSignType::DoubleTurnLeft,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDoubleTurnRight => TrafficSignType::DoubleTurnRight,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHillDownwards => TrafficSignType::HillDownwards,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHillUpwards => TrafficSignType::HillUpwards,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kUnevenRoad => TrafficSignType::UnevenRoad,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRoadSlipperyWetOrDirty => {
            TrafficSignType::RoadSlipperyWetOrDirty
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSideWinds => TrafficSignType::SideWinds,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRoadNarrowing => TrafficSignType::RoadNarrowing,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRoadNarrowingRight => {
            TrafficSignType::RoadNarrowingRight
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRoadNarrowingLeft => {
            TrafficSignType::RoadNarrowingLeft
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRoadWorks => TrafficSignType::RoadWorks,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTrafficQueues => TrafficSignType::TrafficQueues,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTwoWayTraffic => TrafficSignType::TwoWayTraffic,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAttentionTrafficLight => {
            TrafficSignType::AttentionTrafficLight
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPedestrians => TrafficSignType::Pedestrians,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kChildrenCrossing => TrafficSignType::ChildrenCrossing,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCycleRoute => TrafficSignType::CycleRoute,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDeerCrossing => TrafficSignType::DeerCrossing,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kUngatedLevelCrossing => {
            TrafficSignType::UngatedLevelCrossing
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kLevelCrossingMarker => {
            TrafficSignType::LevelCrossingMarker
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRailwayTrafficPriority => {
            TrafficSignType::RailwayTrafficPriority
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kGiveWay => TrafficSignType::GiveWay,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityToOppositeDirection => {
            TrafficSignType::PriorityToOppositeDirection
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityToOppositeDirectionUpsideDown => {
            TrafficSignType::PriorityToOppositeDirectionUpsideDown
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrescribedLeftTurn => {
            TrafficSignType::PrescribedLeftTurn
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrescribedRightTurn => {
            TrafficSignType::PrescribedRightTurn
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrescribedStraight => {
            TrafficSignType::PrescribedStraight
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrescribedRightWay => {
            TrafficSignType::PrescribedRightWay
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrescribedLeftWay => {
            TrafficSignType::PrescribedLeftWay
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrescribedRightTurnAndStraight => {
            TrafficSignType::PrescribedRightTurnAndStraight
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrescribedLeftTurnAndStraight => {
            TrafficSignType::PrescribedLeftTurnAndStraight
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrescribedLeftTurnAndRightTurn => {
            TrafficSignType::PrescribedLeftTurnAndRightTurn
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrescribedLeftTurnRightTurnAndStraight => {
            TrafficSignType::PrescribedLeftTurnRightTurnAndStraight
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRoundabout => TrafficSignType::Roundabout,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOnewayLeft => TrafficSignType::OnewayLeft,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOnewayRight => TrafficSignType::OnewayRight,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPassLeft => TrafficSignType::PassLeft,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPassRight => TrafficSignType::PassRight,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSideLaneOpenForTraffic => {
            TrafficSignType::SideLaneOpenForTraffic
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSideLaneClosedForTraffic => {
            TrafficSignType::SideLaneClosedForTraffic
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSideLaneClosingForTraffic => {
            TrafficSignType::SideLaneClosingForTraffic
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kBusStop => TrafficSignType::BusStop,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTaxiStand => TrafficSignType::TaxiStand,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kBicyclesOnly => TrafficSignType::BicyclesOnly,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHorseRidersOnly => TrafficSignType::HorseRidersOnly,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPedestriansOnly => TrafficSignType::PedestriansOnly,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kBicyclesPedestriansSharedOnly => {
            TrafficSignType::BicyclesPedestriansSharedOnly
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kBicyclesPedestriansSeparatedLeftOnly => {
            TrafficSignType::BicyclesPedestriansSeparatedLeftOnly
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kBicyclesPedestriansSeparatedRightOnly => {
            TrafficSignType::BicyclesPedestriansSeparatedRightOnly
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPedestrianZoneBegin => {
            TrafficSignType::PedestrianZoneBegin
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPedestrianZoneEnd => {
            TrafficSignType::PedestrianZoneEnd
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kBicycleRoadBegin => TrafficSignType::BicycleRoadBegin,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kBicycleRoadEnd => TrafficSignType::BicycleRoadEnd,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kBusLane => TrafficSignType::BusLane,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kBusLaneBegin => TrafficSignType::BusLaneBegin,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kBusLaneEnd => TrafficSignType::BusLaneEnd,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAllProhibited => TrafficSignType::AllProhibited,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kMotorizedMultitrackProhibited => {
            TrafficSignType::MotorizedMultitrackProhibited
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTrucksProhibited => TrafficSignType::TrucksProhibited,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kBicyclesProhibited => {
            TrafficSignType::BicyclesProhibited
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kMotorcyclesProhibited => {
            TrafficSignType::MotorcyclesProhibited
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kMopedsProhibited => TrafficSignType::MopedsProhibited,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHorseRidersProhibited => {
            TrafficSignType::HorseRidersProhibited
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHorseCarriagesProhibited => {
            TrafficSignType::HorseCarriagesProhibited
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCattleProhibited => TrafficSignType::CattleProhibited,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kBusesProhibited => TrafficSignType::BusesProhibited,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCarsProhibited => TrafficSignType::CarsProhibited,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCarsTrailersProhibited => {
            TrafficSignType::CarsTrailersProhibited
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTrucksTrailersProhibited => {
            TrafficSignType::TrucksTrailersProhibited
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTractorsProhibited => {
            TrafficSignType::TractorsProhibited
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPedestriansProhibited => {
            TrafficSignType::PedestriansProhibited
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kMotorVehiclesProhibited => {
            TrafficSignType::MotorVehiclesProhibited
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHazardousGoodsVehiclesProhibited => {
            TrafficSignType::HazardousGoodsVehiclesProhibited
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOverWeightVehiclesProhibited => {
            TrafficSignType::OverWeightVehiclesProhibited
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kVehiclesAxleOverWeightProhibited => {
            TrafficSignType::VehiclesAxleOverWeightProhibited
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kVehiclesExcessWidthProhibited => {
            TrafficSignType::VehiclesExcessWidthProhibited
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kVehiclesExcessHeightProhibited => {
            TrafficSignType::VehiclesExcessHeightProhibited
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kVehiclesExcessLengthProhibited => {
            TrafficSignType::VehiclesExcessLengthProhibited
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDoNotEnter => TrafficSignType::DoNotEnter,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSnowChainsRequired => {
            TrafficSignType::SnowChainsRequired
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kWaterPollutantVehiclesProhibited => {
            TrafficSignType::WaterPollutantVehiclesProhibited
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kEnvironmentalZoneBegin => {
            TrafficSignType::EnvironmentalZoneBegin
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kEnvironmentalZoneEnd => {
            TrafficSignType::EnvironmentalZoneEnd
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrescribedUTurnLeft => {
            TrafficSignType::PrescribedUTurnLeft
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrescribedUTurnRight => {
            TrafficSignType::PrescribedUTurnRight
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kMinimumDistanceForTrucks => {
            TrafficSignType::MinimumDistanceForTrucks
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSpeedLimitBegin => TrafficSignType::SpeedLimitBegin,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSpeedLimitZoneBegin => {
            TrafficSignType::SpeedLimitZoneBegin
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSpeedLimitZoneEnd => {
            TrafficSignType::SpeedLimitZoneEnd
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kMinimumSpeedBegin => {
            TrafficSignType::MinimumSpeedBegin
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOvertakingBanBegin => {
            TrafficSignType::OvertakingBanBegin
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOvertakingBanForTrucksBegin => {
            TrafficSignType::OvertakingBanForTrucksBegin
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSpeedLimitEnd => TrafficSignType::SpeedLimitEnd,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kMinimumSpeedEnd => TrafficSignType::MinimumSpeedEnd,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOvertakingBanEnd => TrafficSignType::OvertakingBanEnd,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOvertakingBanForTrucksEnd => {
            TrafficSignType::OvertakingBanForTrucksEnd
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAllRestrictionsEnd => {
            TrafficSignType::AllRestrictionsEnd
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNoStopping => TrafficSignType::NoStopping,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNoParking => TrafficSignType::NoParking,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNoParkingZoneBegin => {
            TrafficSignType::NoParkingZoneBegin
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNoParkingZoneEnd => TrafficSignType::NoParkingZoneEnd,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRightOfWayNextIntersection => {
            TrafficSignType::RightOfWayNextIntersection
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRightOfWayBegin => TrafficSignType::RightOfWayBegin,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRightOfWayEnd => TrafficSignType::RightOfWayEnd,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityOverOppositeDirection => {
            TrafficSignType::PriorityOverOppositeDirection
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityOverOppositeDirectionUpsideDown => {
            TrafficSignType::PriorityOverOppositeDirectionUpsideDown
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTownBegin => TrafficSignType::TownBegin,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTownEnd => TrafficSignType::TownEnd,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCarParking => TrafficSignType::CarParking,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCarParkingZoneBegin => {
            TrafficSignType::CarParkingZoneBegin
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCarParkingZoneEnd => {
            TrafficSignType::CarParkingZoneEnd
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSidewalkHalfParkingLeft => {
            TrafficSignType::SidewalkHalfParkingLeft
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSidewalkHalfParkingRight => {
            TrafficSignType::SidewalkHalfParkingRight
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSidewalkParkingLeft => {
            TrafficSignType::SidewalkParkingLeft
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSidewalkParkingRight => {
            TrafficSignType::SidewalkParkingRight
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSidewalkPerpendicularHalfParkingLeft => {
            TrafficSignType::SidewalkPerpendicularHalfParkingLeft
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSidewalkPerpendicularHalfParkingRight => {
            TrafficSignType::SidewalkPerpendicularHalfParkingRight
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSidewalkPerpendicularParkingLeft => {
            TrafficSignType::SidewalkPerpendicularParkingLeft
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSidewalkPerpendicularParkingRight => {
            TrafficSignType::SidewalkPerpendicularParkingRight
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kLivingStreetBegin => {
            TrafficSignType::LivingStreetBegin
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kLivingStreetEnd => TrafficSignType::LivingStreetEnd,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTunnel => TrafficSignType::Tunnel,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kEmergencyStoppingLeft => {
            TrafficSignType::EmergencyStoppingLeft
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kEmergencyStoppingRight => {
            TrafficSignType::EmergencyStoppingRight
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHighwayBegin => TrafficSignType::HighwayBegin,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHighwayEnd => TrafficSignType::HighwayEnd,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kExpresswayBegin => TrafficSignType::ExpresswayBegin,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kExpresswayEnd => TrafficSignType::ExpresswayEnd,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNamedHighwayExit => TrafficSignType::NamedHighwayExit,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNamedExpresswayExit => {
            TrafficSignType::NamedExpresswayExit
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNamedRoadExit => TrafficSignType::NamedRoadExit,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHighwayExit => TrafficSignType::HighwayExit,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kExpresswayExit => TrafficSignType::ExpresswayExit,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOnewayStreet => TrafficSignType::OnewayStreet,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCrossingGuards => TrafficSignType::CrossingGuards,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDeadend => TrafficSignType::Deadend,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDeadendExcludingDesignatedActors => {
            TrafficSignType::DeadendExcludingDesignatedActors
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kFirstAidStation => TrafficSignType::FirstAidStation,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPoliceStation => TrafficSignType::PoliceStation,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTelephone => TrafficSignType::Telephone,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kFillingStation => TrafficSignType::FillingStation,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHotel => TrafficSignType::Hotel,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kInn => TrafficSignType::Inn,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kKiosk => TrafficSignType::Kiosk,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kToilet => TrafficSignType::Toilet,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kChapel => TrafficSignType::Chapel,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTouristInfo => TrafficSignType::TouristInfo,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRepairService => TrafficSignType::RepairService,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPedestrianUnderpass => {
            TrafficSignType::PedestrianUnderpass
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPedestrianBridge => TrafficSignType::PedestrianBridge,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCamperPlace => TrafficSignType::CamperPlace,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAdvisorySpeedLimitBegin => {
            TrafficSignType::AdvisorySpeedLimitBegin
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAdvisorySpeedLimitEnd => {
            TrafficSignType::AdvisorySpeedLimitEnd
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPlaceName => TrafficSignType::PlaceName,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTouristAttraction => {
            TrafficSignType::TouristAttraction
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTouristRoute => TrafficSignType::TouristRoute,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTouristArea => TrafficSignType::TouristArea,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kShoulderNotPassableMotorVehicles => {
            TrafficSignType::ShoulderNotPassableMotorVehicles
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kShoulderUnsafeTrucksTractors => {
            TrafficSignType::ShoulderUnsafeTrucksTractors
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTollBegin => TrafficSignType::TollBegin,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTollEnd => TrafficSignType::TollEnd,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTollRoad => TrafficSignType::TollRoad,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCustoms => TrafficSignType::Customs,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kInternationalBorderInfo => {
            TrafficSignType::InternationalBorderInfo
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kStreetlightRedBand => {
            TrafficSignType::StreetlightRedBand
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kFederalHighwayRouteNumber => {
            TrafficSignType::FederalHighwayRouteNumber
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHighwayRouteNumber => {
            TrafficSignType::HighwayRouteNumber
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHighwayInterchangeNumber => {
            TrafficSignType::HighwayInterchangeNumber
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kEuropeanRouteNumber => {
            TrafficSignType::EuropeanRouteNumber
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kFederalHighwayDirectionLeft => {
            TrafficSignType::FederalHighwayDirectionLeft
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kFederalHighwayDirectionRight => {
            TrafficSignType::FederalHighwayDirectionRight
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrimaryRoadDirectionLeft => {
            TrafficSignType::PrimaryRoadDirectionLeft
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrimaryRoadDirectionRight => {
            TrafficSignType::PrimaryRoadDirectionRight
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSecondaryRoadDirectionLeft => {
            TrafficSignType::SecondaryRoadDirectionLeft
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSecondaryRoadDirectionRight => {
            TrafficSignType::SecondaryRoadDirectionRight
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDirectionDesignatedActorsLeft => {
            TrafficSignType::DirectionDesignatedActorsLeft
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDirectionDesignatedActorsRight => {
            TrafficSignType::DirectionDesignatedActorsRight
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRoutingDesignatedActors => {
            TrafficSignType::RoutingDesignatedActors
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDirectionToHighwayLeft => {
            TrafficSignType::DirectionToHighwayLeft
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDirectionToHighwayRight => {
            TrafficSignType::DirectionToHighwayRight
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDirectionToLocalDestinationLeft => {
            TrafficSignType::DirectionToLocalDestinationLeft
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDirectionToLocalDestinationRight => {
            TrafficSignType::DirectionToLocalDestinationRight
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kConsolidatedDirections => {
            TrafficSignType::ConsolidatedDirections
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kStreetName => TrafficSignType::StreetName,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDirectionPreannouncement => {
            TrafficSignType::DirectionPreannouncement
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDirectionPreannouncementLaneConfig => {
            TrafficSignType::DirectionPreannouncementLaneConfig
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDirectionPreannouncementHighwayEntries => {
            TrafficSignType::DirectionPreannouncementHighwayEntries
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHighwayAnnouncement => {
            TrafficSignType::HighwayAnnouncement
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOtherRoadAnnouncement => {
            TrafficSignType::OtherRoadAnnouncement
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHighwayAnnouncementTruckStop => {
            TrafficSignType::HighwayAnnouncementTruckStop
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHighwayPreannouncementDirections => {
            TrafficSignType::HighwayPreannouncementDirections
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPoleExit => TrafficSignType::PoleExit,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHighwayDistanceBoard => {
            TrafficSignType::HighwayDistanceBoard
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDetourLeft => TrafficSignType::DetourLeft,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDetourRight => TrafficSignType::DetourRight,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNumberedDetour => TrafficSignType::NumberedDetour,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDetourBegin => TrafficSignType::DetourBegin,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDetourEnd => TrafficSignType::DetourEnd,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDetourRoutingBoard => {
            TrafficSignType::DetourRoutingBoard
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOptionalDetour => TrafficSignType::OptionalDetour,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOptionalDetourRouting => {
            TrafficSignType::OptionalDetourRouting
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRouteRecommendation => {
            TrafficSignType::RouteRecommendation
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRouteRecommendationEnd => {
            TrafficSignType::RouteRecommendationEnd
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAnnounceLaneTransitionLeft => {
            TrafficSignType::AnnounceLaneTransitionLeft
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAnnounceLaneTransitionRight => {
            TrafficSignType::AnnounceLaneTransitionRight
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAnnounceRightLaneEnd => {
            TrafficSignType::AnnounceRightLaneEnd
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAnnounceLeftLaneEnd => {
            TrafficSignType::AnnounceLeftLaneEnd
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAnnounceRightLaneBegin => {
            TrafficSignType::AnnounceRightLaneBegin
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAnnounceLeftLaneBegin => {
            TrafficSignType::AnnounceLeftLaneBegin
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAnnounceLaneConsolidation => {
            TrafficSignType::AnnounceLaneConsolidation
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDetourCityBlock => TrafficSignType::DetourCityBlock,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kGate => TrafficSignType::Gate,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPoleWarning => TrafficSignType::PoleWarning,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTrafficCone => TrafficSignType::TrafficCone,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kMobileLaneClosure => {
            TrafficSignType::MobileLaneClosure
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kReflectorPost => TrafficSignType::ReflectorPost,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDirectionalBoardWarning => {
            TrafficSignType::DirectionalBoardWarning
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kGuidingPlate => TrafficSignType::GuidingPlate,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kGuidingPlateWedges => {
            TrafficSignType::GuidingPlateWedges
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kParkingHazard => TrafficSignType::ParkingHazard,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTrafficLightGreenArrow => {
            TrafficSignType::TrafficLightGreenArrow
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kText => TrafficSignType::Text,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSpace => TrafficSignType::Space,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTime => TrafficSignType::Time,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kArrow => TrafficSignType::Arrow,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kConstrainedTo => TrafficSignType::ConstrainedTo,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kExcept => TrafficSignType::Except,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kValidForDistance => TrafficSignType::ValidForDistance,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityRoadBottomLeftFourWay => {
            TrafficSignType::PriorityRoadBottomLeftFourWay
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityRoadTopLeftFourWay => {
            TrafficSignType::PriorityRoadTopLeftFourWay
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityRoadBottomLeftThreeWayStraight => {
            TrafficSignType::PriorityRoadBottomLeftThreeWayStraight
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityRoadBottomLeftThreeWaySideways => {
            TrafficSignType::PriorityRoadBottomLeftThreeWaySideways
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityRoadTopLeftThreeWayStraight => {
            TrafficSignType::PriorityRoadTopLeftThreeWayStraight
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityRoadBottomRightFourWay => {
            TrafficSignType::PriorityRoadBottomRightFourWay
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityRoadTopRightFourWay => {
            TrafficSignType::PriorityRoadTopRightFourWay
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityRoadBottomRightThreeWayStraight => {
            TrafficSignType::PriorityRoadBottomRightThreeWayStraight
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityRoadBottomRightThreeWaySideway => {
            TrafficSignType::PriorityRoadBottomRightThreeWaySideway
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityRoadTopRightThreeWayStraight => {
            TrafficSignType::PriorityRoadTopRightThreeWayStraight
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kValidInDistance => TrafficSignType::ValidInDistance,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kStopIn => TrafficSignType::StopIn,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kLeftArrow => TrafficSignType::LeftArrow,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kLeftBendArrow => TrafficSignType::LeftBendArrow,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRightArrow => TrafficSignType::RightArrow,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRightBendArrow => TrafficSignType::RightBendArrow,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAccident => TrafficSignType::Accident,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSnow => TrafficSignType::Snow,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kFog => TrafficSignType::Fog,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRollingHighwayInformation => {
            TrafficSignType::RollingHighwayInformation
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kServices => TrafficSignType::Services,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTimeRange => TrafficSignType::TimeRange,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kParkingDiscTimeRestriction => {
            TrafficSignType::ParkingDiscTimeRestriction
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kWeight => TrafficSignType::Weight,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kWet => TrafficSignType::Wet,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kParkingConstraint => {
            TrafficSignType::ParkingConstraint
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNoWaitingSideStripes => {
            TrafficSignType::NoWaitingSideStripes
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRain => TrafficSignType::Rain,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSnowRain => TrafficSignType::SnowRain,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNight => TrafficSignType::Night,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kStop4Way => TrafficSignType::Stop4Way,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTruck => TrafficSignType::Truck,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTractorsMayBePassed => {
            TrafficSignType::TractorsMayBePassed
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHazardous => TrafficSignType::Hazardous,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTrailer => TrafficSignType::Trailer,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kZone => TrafficSignType::Zone,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kMotorcycle => TrafficSignType::Motorcycle,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kMotorcycleAllowed => {
            TrafficSignType::MotorcycleAllowed
        }
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCar => TrafficSignType::Car,
        maliput_sys::api::rules::ffi::TrafficControlDeviceType::kEmergencyLane => TrafficSignType::EmergencyLane,
        _ => TrafficSignType::Unknown,
    }
}

pub(crate) fn traffic_control_device_type_to_cpp(
    sign_type: &TrafficControlDeviceType,
) -> maliput_sys::api::rules::ffi::TrafficControlDeviceType {
    match sign_type {
        TrafficSignType::None => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNone,
        TrafficSignType::Other => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOther,
        TrafficSignType::Stop => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kStop,
        TrafficSignType::Yield => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kYield,
        TrafficSignType::SpeedLimit => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSpeedLimit,
        TrafficSignType::NoEntry => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNoEntry,
        TrafficSignType::OneWay => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOneWay,
        TrafficSignType::PedestrianCrossing => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPedestrianCrossing
        }
        TrafficSignType::NoLeftTurn => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNoLeftTurn,
        TrafficSignType::NoRightTurn => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNoRightTurn,
        TrafficSignType::NoUTurn => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNoUTurn,
        TrafficSignType::SchoolZone => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSchoolZone,
        TrafficSignType::Construction => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kConstruction,
        TrafficSignType::RailroadCrossing => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRailroadCrossing,
        TrafficSignType::NoOvertaking => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNoOvertaking,
        TrafficSignType::AllWay => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAllWay,
        TrafficSignType::NoUTurnLeft => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNoUTurnLeft,
        TrafficSignType::NoUTurnRight => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNoUTurnRight,
        TrafficSignType::StopLine => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kStopLine,
        TrafficSignType::Crosswalk => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCrosswalk,
        TrafficSignType::DangerSpot => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDangerSpot,
        TrafficSignType::ZebraCrossing => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kZebraCrossing,
        TrafficSignType::Flight => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kFlight,
        TrafficSignType::Cattle => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCattle,
        TrafficSignType::HorseRiders => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHorseRiders,
        TrafficSignType::Amphibians => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAmphibians,
        TrafficSignType::FallingRocks => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kFallingRocks,
        TrafficSignType::SnowOrIce => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSnowOrIce,
        TrafficSignType::LooseGravel => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kLooseGravel,
        TrafficSignType::Waterside => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kWaterside,
        TrafficSignType::Clearance => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kClearance,
        TrafficSignType::MovableBridge => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kMovableBridge,
        TrafficSignType::RightBeforeLeftNextIntersection => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRightBeforeLeftNextIntersection
        }
        TrafficSignType::TurnLeft => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTurnLeft,
        TrafficSignType::TurnRight => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTurnRight,
        TrafficSignType::DoubleTurnLeft => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDoubleTurnLeft,
        TrafficSignType::DoubleTurnRight => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDoubleTurnRight,
        TrafficSignType::HillDownwards => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHillDownwards,
        TrafficSignType::HillUpwards => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHillUpwards,
        TrafficSignType::UnevenRoad => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kUnevenRoad,
        TrafficSignType::RoadSlipperyWetOrDirty => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRoadSlipperyWetOrDirty
        }
        TrafficSignType::SideWinds => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSideWinds,
        TrafficSignType::RoadNarrowing => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRoadNarrowing,
        TrafficSignType::RoadNarrowingRight => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRoadNarrowingRight
        }
        TrafficSignType::RoadNarrowingLeft => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRoadNarrowingLeft
        }
        TrafficSignType::RoadWorks => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRoadWorks,
        TrafficSignType::TrafficQueues => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTrafficQueues,
        TrafficSignType::TwoWayTraffic => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTwoWayTraffic,
        TrafficSignType::AttentionTrafficLight => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAttentionTrafficLight
        }
        TrafficSignType::Pedestrians => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPedestrians,
        TrafficSignType::ChildrenCrossing => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kChildrenCrossing,
        TrafficSignType::CycleRoute => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCycleRoute,
        TrafficSignType::DeerCrossing => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDeerCrossing,
        TrafficSignType::UngatedLevelCrossing => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kUngatedLevelCrossing
        }
        TrafficSignType::LevelCrossingMarker => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kLevelCrossingMarker
        }
        TrafficSignType::RailwayTrafficPriority => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRailwayTrafficPriority
        }
        TrafficSignType::GiveWay => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kGiveWay,
        TrafficSignType::PriorityToOppositeDirection => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityToOppositeDirection
        }
        TrafficSignType::PriorityToOppositeDirectionUpsideDown => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityToOppositeDirectionUpsideDown
        }
        TrafficSignType::PrescribedLeftTurn => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrescribedLeftTurn
        }
        TrafficSignType::PrescribedRightTurn => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrescribedRightTurn
        }
        TrafficSignType::PrescribedStraight => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrescribedStraight
        }
        TrafficSignType::PrescribedRightWay => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrescribedRightWay
        }
        TrafficSignType::PrescribedLeftWay => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrescribedLeftWay
        }
        TrafficSignType::PrescribedRightTurnAndStraight => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrescribedRightTurnAndStraight
        }
        TrafficSignType::PrescribedLeftTurnAndStraight => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrescribedLeftTurnAndStraight
        }
        TrafficSignType::PrescribedLeftTurnAndRightTurn => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrescribedLeftTurnAndRightTurn
        }
        TrafficSignType::PrescribedLeftTurnRightTurnAndStraight => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrescribedLeftTurnRightTurnAndStraight
        }
        TrafficSignType::Roundabout => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRoundabout,
        TrafficSignType::OnewayLeft => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOnewayLeft,
        TrafficSignType::OnewayRight => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOnewayRight,
        TrafficSignType::PassLeft => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPassLeft,
        TrafficSignType::PassRight => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPassRight,
        TrafficSignType::SideLaneOpenForTraffic => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSideLaneOpenForTraffic
        }
        TrafficSignType::SideLaneClosedForTraffic => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSideLaneClosedForTraffic
        }
        TrafficSignType::SideLaneClosingForTraffic => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSideLaneClosingForTraffic
        }
        TrafficSignType::BusStop => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kBusStop,
        TrafficSignType::TaxiStand => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTaxiStand,
        TrafficSignType::BicyclesOnly => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kBicyclesOnly,
        TrafficSignType::HorseRidersOnly => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHorseRidersOnly,
        TrafficSignType::PedestriansOnly => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPedestriansOnly,
        TrafficSignType::BicyclesPedestriansSharedOnly => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kBicyclesPedestriansSharedOnly
        }
        TrafficSignType::BicyclesPedestriansSeparatedLeftOnly => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kBicyclesPedestriansSeparatedLeftOnly
        }
        TrafficSignType::BicyclesPedestriansSeparatedRightOnly => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kBicyclesPedestriansSeparatedRightOnly
        }
        TrafficSignType::PedestrianZoneBegin => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPedestrianZoneBegin
        }
        TrafficSignType::PedestrianZoneEnd => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPedestrianZoneEnd
        }
        TrafficSignType::BicycleRoadBegin => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kBicycleRoadBegin,
        TrafficSignType::BicycleRoadEnd => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kBicycleRoadEnd,
        TrafficSignType::BusLane => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kBusLane,
        TrafficSignType::BusLaneBegin => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kBusLaneBegin,
        TrafficSignType::BusLaneEnd => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kBusLaneEnd,
        TrafficSignType::AllProhibited => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAllProhibited,
        TrafficSignType::MotorizedMultitrackProhibited => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kMotorizedMultitrackProhibited
        }
        TrafficSignType::TrucksProhibited => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTrucksProhibited,
        TrafficSignType::BicyclesProhibited => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kBicyclesProhibited
        }
        TrafficSignType::MotorcyclesProhibited => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kMotorcyclesProhibited
        }
        TrafficSignType::MopedsProhibited => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kMopedsProhibited,
        TrafficSignType::HorseRidersProhibited => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHorseRidersProhibited
        }
        TrafficSignType::HorseCarriagesProhibited => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHorseCarriagesProhibited
        }
        TrafficSignType::CattleProhibited => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCattleProhibited,
        TrafficSignType::BusesProhibited => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kBusesProhibited,
        TrafficSignType::CarsProhibited => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCarsProhibited,
        TrafficSignType::CarsTrailersProhibited => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCarsTrailersProhibited
        }
        TrafficSignType::TrucksTrailersProhibited => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTrucksTrailersProhibited
        }
        TrafficSignType::TractorsProhibited => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTractorsProhibited
        }
        TrafficSignType::PedestriansProhibited => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPedestriansProhibited
        }
        TrafficSignType::MotorVehiclesProhibited => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kMotorVehiclesProhibited
        }
        TrafficSignType::HazardousGoodsVehiclesProhibited => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHazardousGoodsVehiclesProhibited
        }
        TrafficSignType::OverWeightVehiclesProhibited => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOverWeightVehiclesProhibited
        }
        TrafficSignType::VehiclesAxleOverWeightProhibited => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kVehiclesAxleOverWeightProhibited
        }
        TrafficSignType::VehiclesExcessWidthProhibited => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kVehiclesExcessWidthProhibited
        }
        TrafficSignType::VehiclesExcessHeightProhibited => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kVehiclesExcessHeightProhibited
        }
        TrafficSignType::VehiclesExcessLengthProhibited => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kVehiclesExcessLengthProhibited
        }
        TrafficSignType::DoNotEnter => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDoNotEnter,
        TrafficSignType::SnowChainsRequired => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSnowChainsRequired
        }
        TrafficSignType::WaterPollutantVehiclesProhibited => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kWaterPollutantVehiclesProhibited
        }
        TrafficSignType::EnvironmentalZoneBegin => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kEnvironmentalZoneBegin
        }
        TrafficSignType::EnvironmentalZoneEnd => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kEnvironmentalZoneEnd
        }
        TrafficSignType::PrescribedUTurnLeft => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrescribedUTurnLeft
        }
        TrafficSignType::PrescribedUTurnRight => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrescribedUTurnRight
        }
        TrafficSignType::MinimumDistanceForTrucks => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kMinimumDistanceForTrucks
        }
        TrafficSignType::SpeedLimitBegin => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSpeedLimitBegin,
        TrafficSignType::SpeedLimitZoneBegin => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSpeedLimitZoneBegin
        }
        TrafficSignType::SpeedLimitZoneEnd => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSpeedLimitZoneEnd
        }
        TrafficSignType::MinimumSpeedBegin => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kMinimumSpeedBegin
        }
        TrafficSignType::OvertakingBanBegin => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOvertakingBanBegin
        }
        TrafficSignType::OvertakingBanForTrucksBegin => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOvertakingBanForTrucksBegin
        }
        TrafficSignType::SpeedLimitEnd => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSpeedLimitEnd,
        TrafficSignType::MinimumSpeedEnd => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kMinimumSpeedEnd,
        TrafficSignType::OvertakingBanEnd => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOvertakingBanEnd,
        TrafficSignType::OvertakingBanForTrucksEnd => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOvertakingBanForTrucksEnd
        }
        TrafficSignType::AllRestrictionsEnd => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAllRestrictionsEnd
        }
        TrafficSignType::NoStopping => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNoStopping,
        TrafficSignType::NoParking => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNoParking,
        TrafficSignType::NoParkingZoneBegin => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNoParkingZoneBegin
        }
        TrafficSignType::NoParkingZoneEnd => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNoParkingZoneEnd,
        TrafficSignType::RightOfWayNextIntersection => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRightOfWayNextIntersection
        }
        TrafficSignType::RightOfWayBegin => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRightOfWayBegin,
        TrafficSignType::RightOfWayEnd => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRightOfWayEnd,
        TrafficSignType::PriorityOverOppositeDirection => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityOverOppositeDirection
        }
        TrafficSignType::PriorityOverOppositeDirectionUpsideDown => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityOverOppositeDirectionUpsideDown
        }
        TrafficSignType::TownBegin => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTownBegin,
        TrafficSignType::TownEnd => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTownEnd,
        TrafficSignType::CarParking => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCarParking,
        TrafficSignType::CarParkingZoneBegin => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCarParkingZoneBegin
        }
        TrafficSignType::CarParkingZoneEnd => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCarParkingZoneEnd
        }
        TrafficSignType::SidewalkHalfParkingLeft => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSidewalkHalfParkingLeft
        }
        TrafficSignType::SidewalkHalfParkingRight => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSidewalkHalfParkingRight
        }
        TrafficSignType::SidewalkParkingLeft => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSidewalkParkingLeft
        }
        TrafficSignType::SidewalkParkingRight => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSidewalkParkingRight
        }
        TrafficSignType::SidewalkPerpendicularHalfParkingLeft => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSidewalkPerpendicularHalfParkingLeft
        }
        TrafficSignType::SidewalkPerpendicularHalfParkingRight => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSidewalkPerpendicularHalfParkingRight
        }
        TrafficSignType::SidewalkPerpendicularParkingLeft => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSidewalkPerpendicularParkingLeft
        }
        TrafficSignType::SidewalkPerpendicularParkingRight => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSidewalkPerpendicularParkingRight
        }
        TrafficSignType::LivingStreetBegin => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kLivingStreetBegin
        }
        TrafficSignType::LivingStreetEnd => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kLivingStreetEnd,
        TrafficSignType::Tunnel => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTunnel,
        TrafficSignType::EmergencyStoppingLeft => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kEmergencyStoppingLeft
        }
        TrafficSignType::EmergencyStoppingRight => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kEmergencyStoppingRight
        }
        TrafficSignType::HighwayBegin => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHighwayBegin,
        TrafficSignType::HighwayEnd => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHighwayEnd,
        TrafficSignType::ExpresswayBegin => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kExpresswayBegin,
        TrafficSignType::ExpresswayEnd => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kExpresswayEnd,
        TrafficSignType::NamedHighwayExit => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNamedHighwayExit,
        TrafficSignType::NamedExpresswayExit => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNamedExpresswayExit
        }
        TrafficSignType::NamedRoadExit => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNamedRoadExit,
        TrafficSignType::HighwayExit => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHighwayExit,
        TrafficSignType::ExpresswayExit => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kExpresswayExit,
        TrafficSignType::OnewayStreet => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOnewayStreet,
        TrafficSignType::CrossingGuards => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCrossingGuards,
        TrafficSignType::Deadend => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDeadend,
        TrafficSignType::DeadendExcludingDesignatedActors => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDeadendExcludingDesignatedActors
        }
        TrafficSignType::FirstAidStation => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kFirstAidStation,
        TrafficSignType::PoliceStation => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPoliceStation,
        TrafficSignType::Telephone => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTelephone,
        TrafficSignType::FillingStation => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kFillingStation,
        TrafficSignType::Hotel => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHotel,
        TrafficSignType::Inn => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kInn,
        TrafficSignType::Kiosk => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kKiosk,
        TrafficSignType::Toilet => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kToilet,
        TrafficSignType::Chapel => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kChapel,
        TrafficSignType::TouristInfo => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTouristInfo,
        TrafficSignType::RepairService => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRepairService,
        TrafficSignType::PedestrianUnderpass => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPedestrianUnderpass
        }
        TrafficSignType::PedestrianBridge => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPedestrianBridge,
        TrafficSignType::CamperPlace => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCamperPlace,
        TrafficSignType::AdvisorySpeedLimitBegin => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAdvisorySpeedLimitBegin
        }
        TrafficSignType::AdvisorySpeedLimitEnd => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAdvisorySpeedLimitEnd
        }
        TrafficSignType::PlaceName => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPlaceName,
        TrafficSignType::TouristAttraction => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTouristAttraction
        }
        TrafficSignType::TouristRoute => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTouristRoute,
        TrafficSignType::TouristArea => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTouristArea,
        TrafficSignType::ShoulderNotPassableMotorVehicles => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kShoulderNotPassableMotorVehicles
        }
        TrafficSignType::ShoulderUnsafeTrucksTractors => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kShoulderUnsafeTrucksTractors
        }
        TrafficSignType::TollBegin => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTollBegin,
        TrafficSignType::TollEnd => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTollEnd,
        TrafficSignType::TollRoad => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTollRoad,
        TrafficSignType::Customs => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCustoms,
        TrafficSignType::InternationalBorderInfo => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kInternationalBorderInfo
        }
        TrafficSignType::StreetlightRedBand => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kStreetlightRedBand
        }
        TrafficSignType::FederalHighwayRouteNumber => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kFederalHighwayRouteNumber
        }
        TrafficSignType::HighwayRouteNumber => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHighwayRouteNumber
        }
        TrafficSignType::HighwayInterchangeNumber => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHighwayInterchangeNumber
        }
        TrafficSignType::EuropeanRouteNumber => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kEuropeanRouteNumber
        }
        TrafficSignType::FederalHighwayDirectionLeft => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kFederalHighwayDirectionLeft
        }
        TrafficSignType::FederalHighwayDirectionRight => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kFederalHighwayDirectionRight
        }
        TrafficSignType::PrimaryRoadDirectionLeft => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrimaryRoadDirectionLeft
        }
        TrafficSignType::PrimaryRoadDirectionRight => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPrimaryRoadDirectionRight
        }
        TrafficSignType::SecondaryRoadDirectionLeft => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSecondaryRoadDirectionLeft
        }
        TrafficSignType::SecondaryRoadDirectionRight => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSecondaryRoadDirectionRight
        }
        TrafficSignType::DirectionDesignatedActorsLeft => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDirectionDesignatedActorsLeft
        }
        TrafficSignType::DirectionDesignatedActorsRight => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDirectionDesignatedActorsRight
        }
        TrafficSignType::RoutingDesignatedActors => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRoutingDesignatedActors
        }
        TrafficSignType::DirectionToHighwayLeft => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDirectionToHighwayLeft
        }
        TrafficSignType::DirectionToHighwayRight => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDirectionToHighwayRight
        }
        TrafficSignType::DirectionToLocalDestinationLeft => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDirectionToLocalDestinationLeft
        }
        TrafficSignType::DirectionToLocalDestinationRight => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDirectionToLocalDestinationRight
        }
        TrafficSignType::ConsolidatedDirections => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kConsolidatedDirections
        }
        TrafficSignType::StreetName => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kStreetName,
        TrafficSignType::DirectionPreannouncement => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDirectionPreannouncement
        }
        TrafficSignType::DirectionPreannouncementLaneConfig => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDirectionPreannouncementLaneConfig
        }
        TrafficSignType::DirectionPreannouncementHighwayEntries => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDirectionPreannouncementHighwayEntries
        }
        TrafficSignType::HighwayAnnouncement => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHighwayAnnouncement
        }
        TrafficSignType::OtherRoadAnnouncement => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOtherRoadAnnouncement
        }
        TrafficSignType::HighwayAnnouncementTruckStop => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHighwayAnnouncementTruckStop
        }
        TrafficSignType::HighwayPreannouncementDirections => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHighwayPreannouncementDirections
        }
        TrafficSignType::PoleExit => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPoleExit,
        TrafficSignType::HighwayDistanceBoard => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHighwayDistanceBoard
        }
        TrafficSignType::DetourLeft => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDetourLeft,
        TrafficSignType::DetourRight => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDetourRight,
        TrafficSignType::NumberedDetour => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNumberedDetour,
        TrafficSignType::DetourBegin => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDetourBegin,
        TrafficSignType::DetourEnd => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDetourEnd,
        TrafficSignType::DetourRoutingBoard => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDetourRoutingBoard
        }
        TrafficSignType::OptionalDetour => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOptionalDetour,
        TrafficSignType::OptionalDetourRouting => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kOptionalDetourRouting
        }
        TrafficSignType::RouteRecommendation => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRouteRecommendation
        }
        TrafficSignType::RouteRecommendationEnd => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRouteRecommendationEnd
        }
        TrafficSignType::AnnounceLaneTransitionLeft => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAnnounceLaneTransitionLeft
        }
        TrafficSignType::AnnounceLaneTransitionRight => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAnnounceLaneTransitionRight
        }
        TrafficSignType::AnnounceRightLaneEnd => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAnnounceRightLaneEnd
        }
        TrafficSignType::AnnounceLeftLaneEnd => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAnnounceLeftLaneEnd
        }
        TrafficSignType::AnnounceRightLaneBegin => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAnnounceRightLaneBegin
        }
        TrafficSignType::AnnounceLeftLaneBegin => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAnnounceLeftLaneBegin
        }
        TrafficSignType::AnnounceLaneConsolidation => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAnnounceLaneConsolidation
        }
        TrafficSignType::DetourCityBlock => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDetourCityBlock,
        TrafficSignType::Gate => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kGate,
        TrafficSignType::PoleWarning => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPoleWarning,
        TrafficSignType::TrafficCone => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTrafficCone,
        TrafficSignType::MobileLaneClosure => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kMobileLaneClosure
        }
        TrafficSignType::ReflectorPost => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kReflectorPost,
        TrafficSignType::DirectionalBoardWarning => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kDirectionalBoardWarning
        }
        TrafficSignType::GuidingPlate => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kGuidingPlate,
        TrafficSignType::GuidingPlateWedges => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kGuidingPlateWedges
        }
        TrafficSignType::ParkingHazard => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kParkingHazard,
        TrafficSignType::TrafficLightGreenArrow => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTrafficLightGreenArrow
        }
        TrafficSignType::Text => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kText,
        TrafficSignType::Space => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSpace,
        TrafficSignType::Time => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTime,
        TrafficSignType::Arrow => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kArrow,
        TrafficSignType::ConstrainedTo => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kConstrainedTo,
        TrafficSignType::Except => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kExcept,
        TrafficSignType::ValidForDistance => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kValidForDistance,
        TrafficSignType::PriorityRoadBottomLeftFourWay => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityRoadBottomLeftFourWay
        }
        TrafficSignType::PriorityRoadTopLeftFourWay => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityRoadTopLeftFourWay
        }
        TrafficSignType::PriorityRoadBottomLeftThreeWayStraight => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityRoadBottomLeftThreeWayStraight
        }
        TrafficSignType::PriorityRoadBottomLeftThreeWaySideways => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityRoadBottomLeftThreeWaySideways
        }
        TrafficSignType::PriorityRoadTopLeftThreeWayStraight => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityRoadTopLeftThreeWayStraight
        }
        TrafficSignType::PriorityRoadBottomRightFourWay => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityRoadBottomRightFourWay
        }
        TrafficSignType::PriorityRoadTopRightFourWay => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityRoadTopRightFourWay
        }
        TrafficSignType::PriorityRoadBottomRightThreeWayStraight => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityRoadBottomRightThreeWayStraight
        }
        TrafficSignType::PriorityRoadBottomRightThreeWaySideway => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityRoadBottomRightThreeWaySideway
        }
        TrafficSignType::PriorityRoadTopRightThreeWayStraight => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kPriorityRoadTopRightThreeWayStraight
        }
        TrafficSignType::ValidInDistance => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kValidInDistance,
        TrafficSignType::StopIn => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kStopIn,
        TrafficSignType::LeftArrow => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kLeftArrow,
        TrafficSignType::LeftBendArrow => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kLeftBendArrow,
        TrafficSignType::RightArrow => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRightArrow,
        TrafficSignType::RightBendArrow => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRightBendArrow,
        TrafficSignType::Accident => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kAccident,
        TrafficSignType::Snow => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSnow,
        TrafficSignType::Fog => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kFog,
        TrafficSignType::RollingHighwayInformation => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRollingHighwayInformation
        }
        TrafficSignType::Services => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kServices,
        TrafficSignType::TimeRange => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTimeRange,
        TrafficSignType::ParkingDiscTimeRestriction => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kParkingDiscTimeRestriction
        }
        TrafficSignType::Weight => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kWeight,
        TrafficSignType::Wet => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kWet,
        TrafficSignType::ParkingConstraint => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kParkingConstraint
        }
        TrafficSignType::NoWaitingSideStripes => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNoWaitingSideStripes
        }
        TrafficSignType::Rain => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kRain,
        TrafficSignType::SnowRain => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kSnowRain,
        TrafficSignType::Night => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kNight,
        TrafficSignType::Stop4Way => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kStop4Way,
        TrafficSignType::Truck => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTruck,
        TrafficSignType::TractorsMayBePassed => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTractorsMayBePassed
        }
        TrafficSignType::Hazardous => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kHazardous,
        TrafficSignType::Trailer => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kTrailer,
        TrafficSignType::Zone => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kZone,
        TrafficSignType::Motorcycle => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kMotorcycle,
        TrafficSignType::MotorcycleAllowed => {
            maliput_sys::api::rules::ffi::TrafficControlDeviceType::kMotorcycleAllowed
        }
        TrafficSignType::Car => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kCar,
        TrafficSignType::EmergencyLane => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kEmergencyLane,
        TrafficSignType::Unknown => maliput_sys::api::rules::ffi::TrafficControlDeviceType::kUnknown,
    }
}

fn traffic_sign_value_unit_from_cpp(unit: &maliput_sys::api::rules::ffi::TrafficSignValueUnit) -> TrafficSignValueUnit {
    match *unit {
        maliput_sys::api::rules::ffi::TrafficSignValueUnit::kMetersPerSecond => TrafficSignValueUnit::MetersPerSecond,
        maliput_sys::api::rules::ffi::TrafficSignValueUnit::kKilometersPerHour => {
            TrafficSignValueUnit::KilometersPerHour
        }
        maliput_sys::api::rules::ffi::TrafficSignValueUnit::kMilesPerHour => TrafficSignValueUnit::MilesPerHour,
        maliput_sys::api::rules::ffi::TrafficSignValueUnit::kMeters => TrafficSignValueUnit::Meters,
        maliput_sys::api::rules::ffi::TrafficSignValueUnit::kKilometers => TrafficSignValueUnit::Kilometers,
        maliput_sys::api::rules::ffi::TrafficSignValueUnit::kFeet => TrafficSignValueUnit::Feet,
        maliput_sys::api::rules::ffi::TrafficSignValueUnit::kMiles => TrafficSignValueUnit::Miles,
        maliput_sys::api::rules::ffi::TrafficSignValueUnit::kPercent => TrafficSignValueUnit::Percent,
        maliput_sys::api::rules::ffi::TrafficSignValueUnit::kKilograms => TrafficSignValueUnit::Kilograms,
        maliput_sys::api::rules::ffi::TrafficSignValueUnit::kMetricTons => TrafficSignValueUnit::MetricTons,
        _ => panic!("Invalid traffic sign value unit"),
    }
}

/// Interface for accessing the [TrafficSign]s in the [super::RoadNetwork].
pub struct TrafficSignBook<'a> {
    pub(super) traffic_sign_book: &'a maliput_sys::api::rules::ffi::TrafficSignBook,
}

impl<'a> TrafficSignBook<'a> {
    /// Gets all the [TrafficSign]s in the [TrafficSignBook].
    ///
    /// # Returns
    /// A vector of [TrafficSign]s.
    pub fn traffic_signs(&self) -> Vec<TrafficSign<'_>> {
        let traffic_signs_cpp = maliput_sys::api::rules::ffi::TrafficSignBook_TrafficSigns(self.traffic_sign_book);
        traffic_signs_cpp
            .into_iter()
            .map(|ts| TrafficSign {
                traffic_sign: unsafe { ts.traffic_sign.as_ref().expect("TrafficSign pointer is null") },
            })
            .collect::<Vec<TrafficSign>>()
    }

    /// Gets a [TrafficSign] by its id.
    ///
    /// # Arguments
    /// * `id` - The id of the [TrafficSign].
    ///
    /// # Returns
    /// The [TrafficSign] with the given id, or `None` if not found.
    pub fn get_traffic_sign(&self, id: &String) -> Option<TrafficSign<'_>> {
        let ptr = maliput_sys::api::rules::ffi::TrafficSignBook_GetTrafficSign(self.traffic_sign_book, id);
        if ptr.is_null() {
            return None;
        }
        Some(TrafficSign {
            traffic_sign: unsafe { ptr.as_ref().expect("Unable to get underlying traffic sign pointer") },
        })
    }

    /// Gets all [TrafficSign]s whose `related_lanes()` includes the given lane ID.
    ///
    /// # Arguments
    /// * `lane_id` - The lane ID to filter by.
    ///
    /// # Returns
    /// A vector of [TrafficSign]s associated with the given lane.
    pub fn find_by_lane(&self, lane_id: &String) -> Vec<TrafficSign<'_>> {
        let traffic_signs_cpp =
            maliput_sys::api::rules::ffi::TrafficSignBook_FindByLane(self.traffic_sign_book, lane_id);
        traffic_signs_cpp
            .into_iter()
            .map(|ts| TrafficSign {
                traffic_sign: unsafe { ts.traffic_sign.as_ref().expect("TrafficSign pointer is null") },
            })
            .collect::<Vec<TrafficSign>>()
    }

    /// Gets all [TrafficSign]s of the given [TrafficSignType].
    ///
    /// # Arguments
    /// * `sign_type` - The [TrafficSignType] to filter by.
    ///
    /// # Returns
    /// A vector of [TrafficSign]s of the given type.
    pub fn find_by_type(&self, sign_type: &TrafficSignType) -> Vec<TrafficSign<'_>> {
        let sign_type_ffi = traffic_control_device_type_to_cpp(sign_type);
        let traffic_signs_cpp =
            maliput_sys::api::rules::ffi::TrafficSignBook_FindByType(self.traffic_sign_book, sign_type_ffi);
        traffic_signs_cpp
            .into_iter()
            .map(|ts| TrafficSign {
                traffic_sign: unsafe { ts.traffic_sign.as_ref().expect("TrafficSign pointer is null") },
            })
            .collect::<Vec<TrafficSign>>()
    }
}

/// Models a physical traffic sign — a static, passive signaling device placed
/// along or above the road to convey regulatory, warning, or informational
/// messages to road users.
///
/// Unlike [TrafficLight], traffic signs do not expose phase-based bulb states.
/// A sign may still be marked as dynamic or movable by backend metadata.
pub struct TrafficSign<'a> {
    pub traffic_sign: &'a maliput_sys::api::rules::ffi::TrafficSign,
}

impl<'a> TrafficSign<'a> {
    /// Gets the unique identifier of the [TrafficSign].
    ///
    /// # Returns
    /// The id of the [TrafficSign].
    pub fn id(&self) -> String {
        maliput_sys::api::rules::ffi::TrafficSign_id(self.traffic_sign)
    }

    /// Gets the [TrafficSignType] of the [TrafficSign].
    ///
    /// # Returns
    /// The [TrafficSignType].
    pub fn sign_type(&self) -> TrafficSignType {
        let sign_type = maliput_sys::api::rules::ffi::TrafficSign_type(self.traffic_sign);
        traffic_control_device_type_from_cpp(&sign_type)
    }

    /// Gets the position of the [TrafficSign] in the road network's Inertial frame.
    ///
    /// # Returns
    /// An [super::InertialPosition] representing the position of the [TrafficSign].
    pub fn position_road_network(&self) -> super::InertialPosition {
        let inertial_position = maliput_sys::api::rules::ffi::TrafficSign_position_road_network(self.traffic_sign);
        super::InertialPosition { ip: inertial_position }
    }

    /// Gets the orientation of the [TrafficSign] in the road network's Inertial frame.
    ///
    /// # Returns
    /// An [super::Rotation] representing the orientation of the [TrafficSign].
    pub fn orientation_road_network(&self) -> super::Rotation {
        let rotation = maliput_sys::api::rules::ffi::TrafficSign_orientation_road_network(self.traffic_sign);
        super::Rotation { r: rotation }
    }

    /// Gets the optional text message displayed on the [TrafficSign].
    ///
    /// # Returns
    /// `Some(String)` if a message is set, `None` otherwise.
    pub fn message(&self) -> Option<String> {
        let wrapper = maliput_sys::api::rules::ffi::TrafficSign_message(self.traffic_sign);
        if wrapper.is_null() {
            return None;
        }
        Some(wrapper.value.clone())
    }

    /// Returns whether this sign can change semantically over time.
    pub fn is_dynamic(&self) -> bool {
        maliput_sys::api::rules::ffi::TrafficSign::is_dynamic(self.traffic_sign)
    }

    /// Returns whether this sign's position can change.
    pub fn is_movable(&self) -> bool {
        maliput_sys::api::rules::ffi::TrafficSign::is_movable(self.traffic_sign)
    }

    /// Gets the lane IDs that this sign is physically relevant to.
    ///
    /// # Returns
    /// A vector of lane ID strings.
    pub fn related_lanes(&self) -> Vec<String> {
        maliput_sys::api::rules::ffi::TrafficSign_related_lanes(self.traffic_sign)
    }

    /// Gets the bounding box of the [TrafficSign].
    ///
    /// # Returns
    /// A [crate::math::BoundingBox] describing the sign's oriented bounding volume.
    /// The box position is the centroid, `box_size` gives full extents, and `orientation`
    /// is expressed as roll-pitch-yaw angles.
    pub fn bounding_box(&self) -> crate::math::BoundingBox {
        let b = maliput_sys::api::rules::ffi::TrafficSign_bounding_box(self.traffic_sign);
        crate::math::BoundingBox { b }
    }

    /// Gets the optional numeric value associated with the [TrafficSign].
    ///
    /// # Returns
    /// `Some(TrafficSignValue)` if a value is set, `None` otherwise.
    pub fn value(&self) -> Option<TrafficSignValue> {
        let data = maliput_sys::api::rules::ffi::TrafficSign_value(self.traffic_sign);
        if !data.has_value {
            return None;
        }
        Some(TrafficSignValue {
            value: data.value,
            unit: traffic_sign_value_unit_from_cpp(&data.unit),
        })
    }

    /// Returns backend-specific key-value properties for this [TrafficSign].
    pub fn properties(&self) -> HashMap<String, String> {
        maliput_sys::api::rules::ffi::TrafficSign_properties(self.traffic_sign)
            .into_iter()
            .map(|p| (p.key, p.value))
            .collect()
    }

    /// Returns the [TrafficSign]s' IDs that depend on this sign, if any.
    /// For example, a "Stop" sign may have an associated "All way" sign.
    pub fn dependent_signs(&self) -> Vec<String> {
        maliput_sys::api::rules::ffi::TrafficSign_dependent_signs(self.traffic_sign)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn traffic_sign_type_roundtrips_all_known_variants() {
        let variants = [
            TrafficSignType::None,
            TrafficSignType::Other,
            TrafficSignType::Stop,
            TrafficSignType::Yield,
            TrafficSignType::SpeedLimit,
            TrafficSignType::NoEntry,
            TrafficSignType::OneWay,
            TrafficSignType::PedestrianCrossing,
            TrafficSignType::NoLeftTurn,
            TrafficSignType::NoRightTurn,
            TrafficSignType::NoUTurn,
            TrafficSignType::SchoolZone,
            TrafficSignType::Construction,
            TrafficSignType::RailroadCrossing,
            TrafficSignType::NoOvertaking,
            TrafficSignType::AllWay,
            TrafficSignType::NoUTurnLeft,
            TrafficSignType::NoUTurnRight,
            TrafficSignType::StopLine,
            TrafficSignType::Crosswalk,
            TrafficSignType::DangerSpot,
            TrafficSignType::ZebraCrossing,
            TrafficSignType::Flight,
            TrafficSignType::Cattle,
            TrafficSignType::HorseRiders,
            TrafficSignType::Amphibians,
            TrafficSignType::FallingRocks,
            TrafficSignType::SnowOrIce,
            TrafficSignType::LooseGravel,
            TrafficSignType::Waterside,
            TrafficSignType::Clearance,
            TrafficSignType::MovableBridge,
            TrafficSignType::RightBeforeLeftNextIntersection,
            TrafficSignType::TurnLeft,
            TrafficSignType::TurnRight,
            TrafficSignType::DoubleTurnLeft,
            TrafficSignType::DoubleTurnRight,
            TrafficSignType::HillDownwards,
            TrafficSignType::HillUpwards,
            TrafficSignType::UnevenRoad,
            TrafficSignType::RoadSlipperyWetOrDirty,
            TrafficSignType::SideWinds,
            TrafficSignType::RoadNarrowing,
            TrafficSignType::RoadNarrowingRight,
            TrafficSignType::RoadNarrowingLeft,
            TrafficSignType::RoadWorks,
            TrafficSignType::TrafficQueues,
            TrafficSignType::TwoWayTraffic,
            TrafficSignType::AttentionTrafficLight,
            TrafficSignType::Pedestrians,
            TrafficSignType::ChildrenCrossing,
            TrafficSignType::CycleRoute,
            TrafficSignType::DeerCrossing,
            TrafficSignType::UngatedLevelCrossing,
            TrafficSignType::LevelCrossingMarker,
            TrafficSignType::RailwayTrafficPriority,
            TrafficSignType::GiveWay,
            TrafficSignType::PriorityToOppositeDirection,
            TrafficSignType::PriorityToOppositeDirectionUpsideDown,
            TrafficSignType::PrescribedLeftTurn,
            TrafficSignType::PrescribedRightTurn,
            TrafficSignType::PrescribedStraight,
            TrafficSignType::PrescribedRightWay,
            TrafficSignType::PrescribedLeftWay,
            TrafficSignType::PrescribedRightTurnAndStraight,
            TrafficSignType::PrescribedLeftTurnAndStraight,
            TrafficSignType::PrescribedLeftTurnAndRightTurn,
            TrafficSignType::PrescribedLeftTurnRightTurnAndStraight,
            TrafficSignType::Roundabout,
            TrafficSignType::OnewayLeft,
            TrafficSignType::OnewayRight,
            TrafficSignType::PassLeft,
            TrafficSignType::PassRight,
            TrafficSignType::SideLaneOpenForTraffic,
            TrafficSignType::SideLaneClosedForTraffic,
            TrafficSignType::SideLaneClosingForTraffic,
            TrafficSignType::BusStop,
            TrafficSignType::TaxiStand,
            TrafficSignType::BicyclesOnly,
            TrafficSignType::HorseRidersOnly,
            TrafficSignType::PedestriansOnly,
            TrafficSignType::BicyclesPedestriansSharedOnly,
            TrafficSignType::BicyclesPedestriansSeparatedLeftOnly,
            TrafficSignType::BicyclesPedestriansSeparatedRightOnly,
            TrafficSignType::PedestrianZoneBegin,
            TrafficSignType::PedestrianZoneEnd,
            TrafficSignType::BicycleRoadBegin,
            TrafficSignType::BicycleRoadEnd,
            TrafficSignType::BusLane,
            TrafficSignType::BusLaneBegin,
            TrafficSignType::BusLaneEnd,
            TrafficSignType::AllProhibited,
            TrafficSignType::MotorizedMultitrackProhibited,
            TrafficSignType::TrucksProhibited,
            TrafficSignType::BicyclesProhibited,
            TrafficSignType::MotorcyclesProhibited,
            TrafficSignType::MopedsProhibited,
            TrafficSignType::HorseRidersProhibited,
            TrafficSignType::HorseCarriagesProhibited,
            TrafficSignType::CattleProhibited,
            TrafficSignType::BusesProhibited,
            TrafficSignType::CarsProhibited,
            TrafficSignType::CarsTrailersProhibited,
            TrafficSignType::TrucksTrailersProhibited,
            TrafficSignType::TractorsProhibited,
            TrafficSignType::PedestriansProhibited,
            TrafficSignType::MotorVehiclesProhibited,
            TrafficSignType::HazardousGoodsVehiclesProhibited,
            TrafficSignType::OverWeightVehiclesProhibited,
            TrafficSignType::VehiclesAxleOverWeightProhibited,
            TrafficSignType::VehiclesExcessWidthProhibited,
            TrafficSignType::VehiclesExcessHeightProhibited,
            TrafficSignType::VehiclesExcessLengthProhibited,
            TrafficSignType::DoNotEnter,
            TrafficSignType::SnowChainsRequired,
            TrafficSignType::WaterPollutantVehiclesProhibited,
            TrafficSignType::EnvironmentalZoneBegin,
            TrafficSignType::EnvironmentalZoneEnd,
            TrafficSignType::PrescribedUTurnLeft,
            TrafficSignType::PrescribedUTurnRight,
            TrafficSignType::MinimumDistanceForTrucks,
            TrafficSignType::SpeedLimitBegin,
            TrafficSignType::SpeedLimitZoneBegin,
            TrafficSignType::SpeedLimitZoneEnd,
            TrafficSignType::MinimumSpeedBegin,
            TrafficSignType::OvertakingBanBegin,
            TrafficSignType::OvertakingBanForTrucksBegin,
            TrafficSignType::SpeedLimitEnd,
            TrafficSignType::MinimumSpeedEnd,
            TrafficSignType::OvertakingBanEnd,
            TrafficSignType::OvertakingBanForTrucksEnd,
            TrafficSignType::AllRestrictionsEnd,
            TrafficSignType::NoStopping,
            TrafficSignType::NoParking,
            TrafficSignType::NoParkingZoneBegin,
            TrafficSignType::NoParkingZoneEnd,
            TrafficSignType::RightOfWayNextIntersection,
            TrafficSignType::RightOfWayBegin,
            TrafficSignType::RightOfWayEnd,
            TrafficSignType::PriorityOverOppositeDirection,
            TrafficSignType::PriorityOverOppositeDirectionUpsideDown,
            TrafficSignType::TownBegin,
            TrafficSignType::TownEnd,
            TrafficSignType::CarParking,
            TrafficSignType::CarParkingZoneBegin,
            TrafficSignType::CarParkingZoneEnd,
            TrafficSignType::SidewalkHalfParkingLeft,
            TrafficSignType::SidewalkHalfParkingRight,
            TrafficSignType::SidewalkParkingLeft,
            TrafficSignType::SidewalkParkingRight,
            TrafficSignType::SidewalkPerpendicularHalfParkingLeft,
            TrafficSignType::SidewalkPerpendicularHalfParkingRight,
            TrafficSignType::SidewalkPerpendicularParkingLeft,
            TrafficSignType::SidewalkPerpendicularParkingRight,
            TrafficSignType::LivingStreetBegin,
            TrafficSignType::LivingStreetEnd,
            TrafficSignType::Tunnel,
            TrafficSignType::EmergencyStoppingLeft,
            TrafficSignType::EmergencyStoppingRight,
            TrafficSignType::HighwayBegin,
            TrafficSignType::HighwayEnd,
            TrafficSignType::ExpresswayBegin,
            TrafficSignType::ExpresswayEnd,
            TrafficSignType::NamedHighwayExit,
            TrafficSignType::NamedExpresswayExit,
            TrafficSignType::NamedRoadExit,
            TrafficSignType::HighwayExit,
            TrafficSignType::ExpresswayExit,
            TrafficSignType::OnewayStreet,
            TrafficSignType::CrossingGuards,
            TrafficSignType::Deadend,
            TrafficSignType::DeadendExcludingDesignatedActors,
            TrafficSignType::FirstAidStation,
            TrafficSignType::PoliceStation,
            TrafficSignType::Telephone,
            TrafficSignType::FillingStation,
            TrafficSignType::Hotel,
            TrafficSignType::Inn,
            TrafficSignType::Kiosk,
            TrafficSignType::Toilet,
            TrafficSignType::Chapel,
            TrafficSignType::TouristInfo,
            TrafficSignType::RepairService,
            TrafficSignType::PedestrianUnderpass,
            TrafficSignType::PedestrianBridge,
            TrafficSignType::CamperPlace,
            TrafficSignType::AdvisorySpeedLimitBegin,
            TrafficSignType::AdvisorySpeedLimitEnd,
            TrafficSignType::PlaceName,
            TrafficSignType::TouristAttraction,
            TrafficSignType::TouristRoute,
            TrafficSignType::TouristArea,
            TrafficSignType::ShoulderNotPassableMotorVehicles,
            TrafficSignType::ShoulderUnsafeTrucksTractors,
            TrafficSignType::TollBegin,
            TrafficSignType::TollEnd,
            TrafficSignType::TollRoad,
            TrafficSignType::Customs,
            TrafficSignType::InternationalBorderInfo,
            TrafficSignType::StreetlightRedBand,
            TrafficSignType::FederalHighwayRouteNumber,
            TrafficSignType::HighwayRouteNumber,
            TrafficSignType::HighwayInterchangeNumber,
            TrafficSignType::EuropeanRouteNumber,
            TrafficSignType::FederalHighwayDirectionLeft,
            TrafficSignType::FederalHighwayDirectionRight,
            TrafficSignType::PrimaryRoadDirectionLeft,
            TrafficSignType::PrimaryRoadDirectionRight,
            TrafficSignType::SecondaryRoadDirectionLeft,
            TrafficSignType::SecondaryRoadDirectionRight,
            TrafficSignType::DirectionDesignatedActorsLeft,
            TrafficSignType::DirectionDesignatedActorsRight,
            TrafficSignType::RoutingDesignatedActors,
            TrafficSignType::DirectionToHighwayLeft,
            TrafficSignType::DirectionToHighwayRight,
            TrafficSignType::DirectionToLocalDestinationLeft,
            TrafficSignType::DirectionToLocalDestinationRight,
            TrafficSignType::ConsolidatedDirections,
            TrafficSignType::StreetName,
            TrafficSignType::DirectionPreannouncement,
            TrafficSignType::DirectionPreannouncementLaneConfig,
            TrafficSignType::DirectionPreannouncementHighwayEntries,
            TrafficSignType::HighwayAnnouncement,
            TrafficSignType::OtherRoadAnnouncement,
            TrafficSignType::HighwayAnnouncementTruckStop,
            TrafficSignType::HighwayPreannouncementDirections,
            TrafficSignType::PoleExit,
            TrafficSignType::HighwayDistanceBoard,
            TrafficSignType::DetourLeft,
            TrafficSignType::DetourRight,
            TrafficSignType::NumberedDetour,
            TrafficSignType::DetourBegin,
            TrafficSignType::DetourEnd,
            TrafficSignType::DetourRoutingBoard,
            TrafficSignType::OptionalDetour,
            TrafficSignType::OptionalDetourRouting,
            TrafficSignType::RouteRecommendation,
            TrafficSignType::RouteRecommendationEnd,
            TrafficSignType::AnnounceLaneTransitionLeft,
            TrafficSignType::AnnounceLaneTransitionRight,
            TrafficSignType::AnnounceRightLaneEnd,
            TrafficSignType::AnnounceLeftLaneEnd,
            TrafficSignType::AnnounceRightLaneBegin,
            TrafficSignType::AnnounceLeftLaneBegin,
            TrafficSignType::AnnounceLaneConsolidation,
            TrafficSignType::DetourCityBlock,
            TrafficSignType::Gate,
            TrafficSignType::PoleWarning,
            TrafficSignType::TrafficCone,
            TrafficSignType::MobileLaneClosure,
            TrafficSignType::ReflectorPost,
            TrafficSignType::DirectionalBoardWarning,
            TrafficSignType::GuidingPlate,
            TrafficSignType::GuidingPlateWedges,
            TrafficSignType::ParkingHazard,
            TrafficSignType::TrafficLightGreenArrow,
            TrafficSignType::Text,
            TrafficSignType::Space,
            TrafficSignType::Time,
            TrafficSignType::Arrow,
            TrafficSignType::ConstrainedTo,
            TrafficSignType::Except,
            TrafficSignType::ValidForDistance,
            TrafficSignType::PriorityRoadBottomLeftFourWay,
            TrafficSignType::PriorityRoadTopLeftFourWay,
            TrafficSignType::PriorityRoadBottomLeftThreeWayStraight,
            TrafficSignType::PriorityRoadBottomLeftThreeWaySideways,
            TrafficSignType::PriorityRoadTopLeftThreeWayStraight,
            TrafficSignType::PriorityRoadBottomRightFourWay,
            TrafficSignType::PriorityRoadTopRightFourWay,
            TrafficSignType::PriorityRoadBottomRightThreeWayStraight,
            TrafficSignType::PriorityRoadBottomRightThreeWaySideway,
            TrafficSignType::PriorityRoadTopRightThreeWayStraight,
            TrafficSignType::ValidInDistance,
            TrafficSignType::StopIn,
            TrafficSignType::LeftArrow,
            TrafficSignType::LeftBendArrow,
            TrafficSignType::RightArrow,
            TrafficSignType::RightBendArrow,
            TrafficSignType::Accident,
            TrafficSignType::Snow,
            TrafficSignType::Fog,
            TrafficSignType::RollingHighwayInformation,
            TrafficSignType::Services,
            TrafficSignType::TimeRange,
            TrafficSignType::ParkingDiscTimeRestriction,
            TrafficSignType::Weight,
            TrafficSignType::Wet,
            TrafficSignType::ParkingConstraint,
            TrafficSignType::NoWaitingSideStripes,
            TrafficSignType::Rain,
            TrafficSignType::SnowRain,
            TrafficSignType::Night,
            TrafficSignType::Stop4Way,
            TrafficSignType::Truck,
            TrafficSignType::TractorsMayBePassed,
            TrafficSignType::Hazardous,
            TrafficSignType::Trailer,
            TrafficSignType::Zone,
            TrafficSignType::Motorcycle,
            TrafficSignType::MotorcycleAllowed,
            TrafficSignType::Car,
            TrafficSignType::EmergencyLane,
            TrafficSignType::Unknown,
        ];

        for variant in variants {
            let cpp = traffic_control_device_type_to_cpp(&variant);
            let roundtrip = traffic_control_device_type_from_cpp(&cpp);
            assert_eq!(roundtrip, variant);
        }
    }
}
