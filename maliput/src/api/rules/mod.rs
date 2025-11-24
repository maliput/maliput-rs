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
use strum_macros::{Display, IntoStaticStr};

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
    /// An [crate::api::InertialPosition] representing the position of the [TrafficLight] in the road network.
    pub fn position_road_network(&self) -> crate::api::InertialPosition {
        let inertial_position = maliput_sys::api::rules::ffi::TrafficLight_position_road_network(self.traffic_light);
        crate::api::InertialPosition { ip: inertial_position }
    }

    /// Get the orientation of the [TrafficLight] in the road network.
    ///
    /// # Returns
    /// An [crate::api::Rotation] representing the orientation of the [TrafficLight] in the road network.
    pub fn orientation_road_network(&self) -> crate::api::Rotation {
        let rotation = maliput_sys::api::rules::ffi::TrafficLight_orientation_road_network(self.traffic_light);
        crate::api::Rotation { r: rotation }
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
    Arrow,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
/// Defines the possible bulb types.
pub enum BulbState {
    Off,
    On,
    Blinking,
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
            _ => panic!("Invalid bulb type"),
        }
    }

    /// Get the position of the [Bulb] in the bulb group.
    ///
    /// # Returns
    /// An [crate::api::InertialPosition] representing the position of the [Bulb] in the bulb group.
    pub fn position_bulb_group(&self) -> crate::api::InertialPosition {
        let inertial_position = maliput_sys::api::rules::ffi::Bulb_position_bulb_group(self.bulb);
        crate::api::InertialPosition { ip: inertial_position }
    }

    /// Get the orientation of the [Bulb] in the bulb group.
    ///
    /// # Returns
    /// An [crate::api::Rotation] representing the orientation of the [Bulb] in the bulb group.
    pub fn orientation_bulb_group(&self) -> crate::api::Rotation {
        let rotation = maliput_sys::api::rules::ffi::Bulb_orientation_bulb_group(self.bulb);
        crate::api::Rotation { r: rotation }
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
    /// An [crate::api::InertialPosition] representing the position of the [BulbGroup] in the traffic light.
    pub fn position_traffic_light(&self) -> crate::api::InertialPosition {
        let inertial_position = maliput_sys::api::rules::ffi::BulbGroup_position_traffic_light(self.bulb_group);
        crate::api::InertialPosition { ip: inertial_position }
    }

    /// Gets the orientation of the [BulbGroup] in the traffic light.
    ///
    /// # Returns
    /// An [crate::api::Rotation] representing the orientation of the [BulbGroup] in the traffic light.
    pub fn orientation_traffic_light(&self) -> crate::api::Rotation {
        let rotation = maliput_sys::api::rules::ffi::BulbGroup_orientation_traffic_light(self.bulb_group);
        crate::api::Rotation { r: rotation }
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
    unique_bulb_id: cxx::UniquePtr<maliput_sys::api::rules::ffi::UniqueBulbId>,
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
    /// Returns a [crate::api::LaneSRoute] that represents the zone that the rule applies to.
    ///
    /// # Returns
    /// A [crate::api::LaneSRoute] representing the zone of the rule.
    pub fn zone(&self) -> crate::api::LaneSRoute {
        let lane_s_route = maliput_sys::api::rules::ffi::DiscreteValueRule_zone(&self.discrete_value_rule);
        crate::api::LaneSRoute { lane_s_route }
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
    /// Returns a [crate::api::LaneSRoute] that represents the zone that the rule applies to.
    ///
    /// # Returns
    /// A [crate::api::LaneSRoute] representing the zone of the rule.
    pub fn zone(&self) -> crate::api::LaneSRoute {
        let lane_s_route = maliput_sys::api::rules::ffi::RangeValueRule_zone(&self.range_value_rule);
        crate::api::LaneSRoute { lane_s_route }
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
type PhaseStateProviderQuery = StateProviderQuery<String>;

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

pub struct DiscreteValueRuleStateProvider<'a> {
    pub(super) state_provider: &'a maliput_sys::api::rules::ffi::DiscreteValueRuleStateProvider,
}

impl<'a> DiscreteValueRuleStateProvider<'a> {
    pub fn get_state_by_rule_id(&self, rule_id: &String) -> Option<StateProviderQuery<DiscreteValue>> {
        let query_state =
            maliput_sys::api::rules::ffi::DiscreteValueRuleStateProvider_GetStateById(self.state_provider, rule_id);
        Self::next_state_from_cxx_query(query_state)
    }

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

pub struct RangeValueRuleStateProvider<'a> {
    pub(super) state_provider: &'a maliput_sys::api::rules::ffi::RangeValueRuleStateProvider,
}

impl<'a> RangeValueRuleStateProvider<'a> {
    pub fn get_state_by_rule_id(&self, rule_id: &String) -> Option<StateProviderQuery<Range>> {
        let query_state =
            maliput_sys::api::rules::ffi::RangeValueRuleStateProvider_GetStateById(self.state_provider, rule_id);
        Self::next_state_from_cxx_query(query_state)
    }

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
fn discrete_value_from_discrete_value_cxx(
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
