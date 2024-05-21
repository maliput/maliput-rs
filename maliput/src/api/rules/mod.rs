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

/// Interface for accessing the [TrafficLight] in the [super::RoadNetwork]
pub struct TrafficLightBook<'a> {
    pub(super) traffic_light_book: &'a maliput_sys::api::rules::ffi::TrafficLightBook,
}

impl<'a> TrafficLightBook<'a> {
    /// Get all the [TrafficLight]s in the [TrafficLightBook]
    /// ## Return
    /// A vector of [TrafficLight]s
    pub fn traffic_lights(&self) -> Vec<TrafficLight> {
        let traffic_lights_cpp = maliput_sys::api::rules::ffi::TrafficLightBook_TrafficLights(self.traffic_light_book);
        traffic_lights_cpp
            .into_iter()
            .map(|tl| TrafficLight {
                traffic_light: unsafe { tl.traffic_light.as_ref().expect("") },
            })
            .collect::<Vec<TrafficLight>>()
    }

    /// Get a [TrafficLight] by its id
    /// ## Arguments
    /// * `id` - The id of the [TrafficLight]
    /// ## Return
    /// The [TrafficLight] with the given id.
    /// If no [TrafficLight] is found with the given id, return None.
    pub fn get_traffic_light(&self, id: &String) -> Option<TrafficLight> {
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
    /// ## Return
    /// The id of the [TrafficLight].
    pub fn id(&self) -> String {
        maliput_sys::api::rules::ffi::TrafficLight_id(self.traffic_light)
    }

    /// Get the position of the [TrafficLight] in the road network.
    /// ## Return
    /// An [crate::api::InertialPosition] representing the position of the [TrafficLight] in the road network.
    pub fn position_road_network(&self) -> crate::api::InertialPosition {
        let inertial_position = maliput_sys::api::rules::ffi::TrafficLight_position_road_network(self.traffic_light);
        crate::api::InertialPosition { ip: inertial_position }
    }

    /// Get the orientation of the [TrafficLight] in the road network.
    /// ## Return
    /// An [crate::api::Rotation] representing the orientation of the [TrafficLight] in the road network.
    pub fn orientation_road_network(&self) -> crate::api::Rotation {
        let rotation = maliput_sys::api::rules::ffi::TrafficLight_orientation_road_network(self.traffic_light);
        crate::api::Rotation { r: rotation }
    }

    /// Get the bulb groups of the [TrafficLight].
    /// ## Return
    /// A vector of [BulbGroup]s in the [TrafficLight].
    /// If the [TrafficLight] has no bulb groups, return an empty vector.
    pub fn bulb_groups(&self) -> Vec<BulbGroup> {
        let bulb_groups_cpp = maliput_sys::api::rules::ffi::TrafficLight_bulb_groups(self.traffic_light);
        bulb_groups_cpp
            .into_iter()
            .map(|bg| BulbGroup {
                bulb_group: unsafe { bg.bulb_group.as_ref().expect("") },
            })
            .collect::<Vec<BulbGroup>>()
    }

    /// Get a [BulbGroup] by its id.
    /// ## Arguments
    /// * `id` - The id of the [BulbGroup].
    ///
    /// ## Return
    /// The [BulbGroup] with the given id.
    /// If no [BulbGroup] is found with the given id, return None.
    pub fn get_bulb_group(&self, id: &String) -> Option<BulbGroup> {
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
    pub fn unique_id(&self) -> String {
        unimplemented!()
    }

    /// Get the id of the [Bulb].
    /// ## Return
    /// The id of the [Bulb].
    pub fn id(&self) -> String {
        maliput_sys::api::rules::ffi::Bulb_id(self.bulb)
    }

    /// Get the color of the [Bulb].
    /// ## Return
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
    /// ## Return
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
    /// ## Return
    /// An [crate::api::InertialPosition] representing the position of the [Bulb] in the bulb group.
    pub fn position_bulb_group(&self) -> crate::api::InertialPosition {
        let inertial_position = maliput_sys::api::rules::ffi::Bulb_position_bulb_group(self.bulb);
        crate::api::InertialPosition { ip: inertial_position }
    }

    /// Get the orientation of the [Bulb] in the bulb group.
    /// ## Return
    /// An [crate::api::Rotation] representing the orientation of the [Bulb] in the bulb group.
    pub fn orientation_bulb_group(&self) -> crate::api::Rotation {
        let rotation = maliput_sys::api::rules::ffi::Bulb_orientation_bulb_group(self.bulb);
        crate::api::Rotation { r: rotation }
    }

    /// Returns the arrow's orientation. Only applicable if [Bulb::bulb_type] returns
    /// [BulbType::Arrow].
    pub fn arrow_orientation_rad(&self) -> Option<f64> {
        let arrow_orientation = maliput_sys::api::rules::ffi::Bulb_arrow_orientation_rad(self.bulb);
        if arrow_orientation.is_null() {
            return None;
        }
        Some(arrow_orientation.value)
    }

    /// Get the possible states of the [Bulb].
    pub fn states(&self) -> Vec<BulbState> {
        let states_cpp = maliput_sys::api::rules::ffi::Bulb_states(self.bulb);
        states_cpp
            .into_iter()
            .map(Bulb::_from_cpp_state_to_rust_state)
            .collect::<Vec<BulbState>>()
    }

    /// Get the default state of the [Bulb].
    pub fn get_default_state(&self) -> BulbState {
        let default_state = self.bulb.GetDefaultState();
        Bulb::_from_cpp_state_to_rust_state(&default_state)
    }

    /// Check if the given state is possible valid for the [Bulb].
    pub fn is_valid_state(&self, state: &BulbState) -> bool {
        self.bulb.IsValidState(&Bulb::_from_rust_state_to_cpp_state(state))
    }

    /// Returns the bounding box of the bulb.
    /// ## Return
    /// A tuple containing the minimum and maximum points of the bounding box.
    pub fn bounding_box(&self) -> (crate::math::Vector3, crate::math::Vector3) {
        let min = maliput_sys::api::rules::ffi::Bulb_bounding_box_min(self.bulb);
        let max = maliput_sys::api::rules::ffi::Bulb_bounding_box_max(self.bulb);
        (crate::math::Vector3 { v: min }, crate::math::Vector3 { v: max })
    }

    /// Returns the parent [BulbGroup] of the bulb.
    /// ## Return
    /// The parent [BulbGroup] of the bulb.
    /// If the bulb is not part of any group, return None.
    pub fn bulb_group(&self) -> BulbGroup {
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
    /// ## Arguments
    /// * `cpp_bulb_state` - The C++ BulbState
    /// ## Return
    /// The Rust BulbState
    /// ## Panics
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
    /// ## Arguments
    /// * `rust_bulb_state` - The Rust BulbState
    /// ## Return
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
    pub fn unique_id(&self) -> String {
        unimplemented!()
    }

    /// Get the id of the [BulbGroup].
    /// ## Return
    /// The id of the [BulbGroup].
    pub fn id(&self) -> String {
        maliput_sys::api::rules::ffi::BulbGroup_id(self.bulb_group)
    }

    /// Get the position of the [BulbGroup] in the traffic light.
    /// ## Return
    /// An [crate::api::InertialPosition] representing the position of the [BulbGroup] in the traffic light.
    pub fn position_traffic_light(&self) -> crate::api::InertialPosition {
        let inertial_position = maliput_sys::api::rules::ffi::BulbGroup_position_traffic_light(self.bulb_group);
        crate::api::InertialPosition { ip: inertial_position }
    }

    /// Get the orientation of the [BulbGroup] in the traffic light.
    /// ## Return
    /// An [crate::api::Rotation] representing the orientation of the [BulbGroup] in the traffic light.
    pub fn orientation_traffic_light(&self) -> crate::api::Rotation {
        let rotation = maliput_sys::api::rules::ffi::BulbGroup_orientation_traffic_light(self.bulb_group);
        crate::api::Rotation { r: rotation }
    }

    /// Returns the bulbs in the bulb group.
    /// ## Return
    /// A vector of [Bulb]s in the bulb group.
    pub fn bulbs(&self) -> Vec<Bulb> {
        let bulbs_cpp = maliput_sys::api::rules::ffi::BulbGroup_bulbs(self.bulb_group);
        bulbs_cpp
            .into_iter()
            .map(|b| Bulb {
                bulb: unsafe { b.bulb.as_ref().expect("") },
            })
            .collect::<Vec<Bulb>>()
    }

    /// Get a [Bulb] by its id
    /// ## Arguments
    /// * `id` - The id of the [Bulb].
    ///
    /// ## Return
    /// The [Bulb] with the given id.
    /// If no [Bulb] is found with the given id, return None.
    pub fn get_bulb(&self, id: &String) -> Option<Bulb> {
        let bulb = maliput_sys::api::rules::ffi::BulbGroup_GetBulb(self.bulb_group, id);
        if bulb.is_null() {
            return None;
        }
        Some(Bulb {
            bulb: unsafe { bulb.as_ref().expect("Unable to get underlying bulb pointer") },
        })
    }

    /// Returns the parent [TrafficLight] of the bulb group.
    /// ## Return
    /// The parent [TrafficLight] of the bulb group.
    pub fn traffic_light(&self) -> TrafficLight {
        TrafficLight {
            traffic_light: unsafe {
                maliput_sys::api::rules::ffi::BulbGroup_traffic_light(self.bulb_group)
                    .as_ref()
                    .expect("Unable to get underlying traffic light pointer. The BulbGroup might not be registered to a TrafficLight.")
            },
        }
    }
}
