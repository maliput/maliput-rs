// BSD 3-Clause License
//
// Copyright (c) 2026, Woven by Toyota.
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
use strum_macros::{Display, EnumString};

#[derive(Debug, Copy, Clone, PartialEq, Eq, EnumString, Display)]
/// Defines the possible road object types.
pub enum RoadObjectType {
    Unknown,
    Barrier,
    GuardWall,
    GuardRail,
    Building,
    Gantry,
    Obstacle,
    Pole,
    TrafficIsland,
    Tree,
    Vegetation,
    Pylon,
    Delineator,
}

fn road_object_type_from_cpp(obj_type: maliput_sys::api::objects::ffi::RoadObjectType) -> RoadObjectType {
    match obj_type {
        maliput_sys::api::objects::ffi::RoadObjectType::kUnknown => RoadObjectType::Unknown,
        maliput_sys::api::objects::ffi::RoadObjectType::kBarrier => RoadObjectType::Barrier,
        maliput_sys::api::objects::ffi::RoadObjectType::kGuardWall => RoadObjectType::GuardWall,
        maliput_sys::api::objects::ffi::RoadObjectType::kGuardRail => RoadObjectType::GuardRail,
        maliput_sys::api::objects::ffi::RoadObjectType::kBuilding => RoadObjectType::Building,
        maliput_sys::api::objects::ffi::RoadObjectType::kGantry => RoadObjectType::Gantry,
        maliput_sys::api::objects::ffi::RoadObjectType::kObstacle => RoadObjectType::Obstacle,
        maliput_sys::api::objects::ffi::RoadObjectType::kPole => RoadObjectType::Pole,
        maliput_sys::api::objects::ffi::RoadObjectType::kTrafficIsland => RoadObjectType::TrafficIsland,
        maliput_sys::api::objects::ffi::RoadObjectType::kTree => RoadObjectType::Tree,
        maliput_sys::api::objects::ffi::RoadObjectType::kVegetation => RoadObjectType::Vegetation,
        maliput_sys::api::objects::ffi::RoadObjectType::kPylon => RoadObjectType::Pylon,
        maliput_sys::api::objects::ffi::RoadObjectType::kDelineator => RoadObjectType::Delineator,
        _ => RoadObjectType::Unknown,
    }
}

fn road_object_type_to_cpp(obj_type: &RoadObjectType) -> maliput_sys::api::objects::ffi::RoadObjectType {
    match obj_type {
        RoadObjectType::Unknown => maliput_sys::api::objects::ffi::RoadObjectType::kUnknown,
        RoadObjectType::Barrier => maliput_sys::api::objects::ffi::RoadObjectType::kBarrier,
        RoadObjectType::GuardWall => maliput_sys::api::objects::ffi::RoadObjectType::kGuardWall,
        RoadObjectType::GuardRail => maliput_sys::api::objects::ffi::RoadObjectType::kGuardRail,
        RoadObjectType::Building => maliput_sys::api::objects::ffi::RoadObjectType::kBuilding,
        RoadObjectType::Gantry => maliput_sys::api::objects::ffi::RoadObjectType::kGantry,
        RoadObjectType::Obstacle => maliput_sys::api::objects::ffi::RoadObjectType::kObstacle,
        RoadObjectType::Pole => maliput_sys::api::objects::ffi::RoadObjectType::kPole,
        RoadObjectType::TrafficIsland => maliput_sys::api::objects::ffi::RoadObjectType::kTrafficIsland,
        RoadObjectType::Tree => maliput_sys::api::objects::ffi::RoadObjectType::kTree,
        RoadObjectType::Vegetation => maliput_sys::api::objects::ffi::RoadObjectType::kVegetation,
        RoadObjectType::Pylon => maliput_sys::api::objects::ffi::RoadObjectType::kPylon,
        RoadObjectType::Delineator => maliput_sys::api::objects::ffi::RoadObjectType::kDelineator,
    }
}

/// Represents the position of a [RoadObject] in the road network.
///
/// Always contains an inertial position. May also contain a lane position
/// expressed as `(lane_id, s, r, h)`.
pub struct RoadObjectPosition {
    pub inertial_position: crate::api::InertialPosition,
    pub lane_position: Option<(String, f64, f64, f64)>,
}

/// Represents a single corner point of an [Outline].
///
/// `height` is `Some` when the corner has an explicit height attribute, `None` otherwise.
pub struct OutlineCorner {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub height: Option<f64>,
}

/// Represents an outline (a polygon) of a [RoadObject].
pub struct Outline<'a> {
    pub(crate) outline: &'a maliput_sys::api::objects::ffi::Outline,
}

impl<'a> Outline<'a> {
    /// Returns the unique identifier of this outline.
    pub fn id(&self) -> String {
        maliput_sys::api::objects::ffi::Outline_id(self.outline)
    }

    /// Returns whether this outline is closed (first and last corner coincide).
    pub fn is_closed(&self) -> bool {
        maliput_sys::api::objects::ffi::Outline_is_closed(self.outline)
    }

    /// Returns the number of corners in this outline.
    pub fn num_corners(&self) -> i32 {
        maliput_sys::api::objects::ffi::Outline_num_corners(self.outline)
    }

    /// Returns the corners of this outline.
    pub fn corners(&self) -> Vec<OutlineCorner> {
        maliput_sys::api::objects::ffi::Outline_corners(self.outline)
            .into_iter()
            .map(|c| OutlineCorner {
                x: c.x,
                y: c.y,
                z: c.z,
                height: if c.has_height { Some(c.height) } else { None },
            })
            .collect()
    }
}

/// Models a physical road object in the road network.
///
/// A `RoadObject` has a type, position, orientation, bounding box, and optional
/// outlines and properties.
pub struct RoadObject<'a> {
    pub(crate) road_object: &'a maliput_sys::api::objects::ffi::RoadObject,
}

impl<'a> RoadObject<'a> {
    /// Returns the unique identifier of this road object.
    pub fn id(&self) -> String {
        maliput_sys::api::objects::ffi::RoadObject_id(self.road_object)
    }

    /// Returns the optional human-readable name of this road object.
    pub fn name(&self) -> Option<String> {
        let wrapper = maliput_sys::api::objects::ffi::RoadObject_name(self.road_object);
        if wrapper.is_null() {
            return None;
        }
        Some(wrapper.value.clone())
    }

    /// Returns the [RoadObjectType] of this road object.
    pub fn object_type(&self) -> RoadObjectType {
        let t = maliput_sys::api::objects::ffi::RoadObject_object_type(self.road_object);
        road_object_type_from_cpp(t)
    }

    /// Returns the optional subtype string of this road object.
    pub fn subtype(&self) -> Option<String> {
        let wrapper = maliput_sys::api::objects::ffi::RoadObject_subtype(self.road_object);
        if wrapper.is_null() {
            return None;
        }
        Some(wrapper.value.clone())
    }

    /// Returns the [RoadObjectPosition] of this road object.
    pub fn position(&self) -> RoadObjectPosition {
        let inertial_position = maliput_sys::api::objects::ffi::RoadObject_position_inertial(self.road_object);
        let has_lane = maliput_sys::api::objects::ffi::RoadObject_position_has_lane_position(self.road_object);
        let lane_position = if has_lane {
            Some((
                maliput_sys::api::objects::ffi::RoadObject_position_lane_id(self.road_object),
                maliput_sys::api::objects::ffi::RoadObject_position_lane_s(self.road_object),
                maliput_sys::api::objects::ffi::RoadObject_position_lane_r(self.road_object),
                maliput_sys::api::objects::ffi::RoadObject_position_lane_h(self.road_object),
            ))
        } else {
            None
        };
        RoadObjectPosition {
            inertial_position: crate::api::InertialPosition { ip: inertial_position },
            lane_position,
        }
    }

    /// Returns the orientation of this road object in the inertial frame.
    pub fn orientation(&self) -> crate::api::Rotation {
        let rotation = maliput_sys::api::objects::ffi::RoadObject_orientation(self.road_object);
        crate::api::Rotation { r: rotation }
    }

    /// Returns the bounding box of this road object.
    pub fn bounding_box(&self) -> crate::math::BoundingBox {
        let b = maliput_sys::api::objects::ffi::RoadObject_bounding_box(self.road_object);
        crate::math::BoundingBox { b }
    }

    /// Returns whether this road object is dynamic (i.e., may change position or state).
    pub fn is_dynamic(&self) -> bool {
        maliput_sys::api::objects::ffi::RoadObject::is_dynamic(self.road_object)
    }

    /// Returns whether this road object's position can change.
    pub fn is_movable(&self) -> bool {
        maliput_sys::api::objects::ffi::RoadObject::is_movable(self.road_object)
    }

    /// Returns the IDs of lanes related to this road object.
    pub fn related_lanes(&self) -> Vec<String> {
        maliput_sys::api::objects::ffi::RoadObject_related_lanes(self.road_object)
    }

    /// Returns the number of outlines of this road object.
    pub fn num_outlines(&self) -> i32 {
        maliput_sys::api::objects::ffi::RoadObject::num_outlines(self.road_object)
    }

    /// Returns the outlines of this road object.
    pub fn outlines(&self) -> Vec<Outline<'_>> {
        maliput_sys::api::objects::ffi::RoadObject_outlines(self.road_object)
            .into_iter()
            .map(|ptr| Outline {
                outline: unsafe { ptr.outline.as_ref().expect("Outline pointer is null") },
            })
            .collect()
    }

    /// Returns the key-value properties of this road object.
    pub fn properties(&self) -> HashMap<String, String> {
        maliput_sys::api::objects::ffi::RoadObject_properties(self.road_object)
            .into_iter()
            .map(|p| (p.key, p.value))
            .collect()
    }
}

/// Interface for accessing [RoadObject]s in the road network.
pub struct RoadObjectBook<'a> {
    pub(super) road_object_book: &'a maliput_sys::api::objects::ffi::RoadObjectBook,
}

impl<'a> RoadObjectBook<'a> {
    /// Returns all [RoadObject]s in the book.
    pub fn road_objects(&self) -> Vec<RoadObject<'_>> {
        maliput_sys::api::objects::ffi::RoadObjectBook_RoadObjects(self.road_object_book)
            .into_iter()
            .map(|ptr| RoadObject {
                road_object: unsafe { ptr.road_object.as_ref().expect("RoadObject pointer is null") },
            })
            .collect()
    }

    /// Returns the [RoadObject] with the given `id`, or `None` if not found.
    pub fn get_road_object(&self, id: &String) -> Option<RoadObject<'_>> {
        let ptr = maliput_sys::api::objects::ffi::RoadObjectBook_GetRoadObject(self.road_object_book, id);
        if ptr.is_null() {
            return None;
        }
        Some(RoadObject {
            road_object: unsafe { ptr.as_ref().expect("Unable to get underlying RoadObject pointer") },
        })
    }

    /// Returns all [RoadObject]s of the given [RoadObjectType].
    pub fn find_by_type(&self, obj_type: &RoadObjectType) -> Vec<RoadObject<'_>> {
        let obj_type_ffi = road_object_type_to_cpp(obj_type);
        maliput_sys::api::objects::ffi::RoadObjectBook_FindByType(self.road_object_book, obj_type_ffi)
            .into_iter()
            .map(|ptr| RoadObject {
                road_object: unsafe { ptr.road_object.as_ref().expect("RoadObject pointer is null") },
            })
            .collect()
    }

    /// Returns all [RoadObject]s whose `related_lanes()` includes the given lane ID.
    pub fn find_by_lane(&self, lane_id: &String) -> Vec<RoadObject<'_>> {
        maliput_sys::api::objects::ffi::RoadObjectBook_FindByLane(self.road_object_book, lane_id)
            .into_iter()
            .map(|ptr| RoadObject {
                road_object: unsafe { ptr.road_object.as_ref().expect("RoadObject pointer is null") },
            })
            .collect()
    }

    /// Returns all [RoadObject]s within `radius` meters of position `(x, y, z)`.
    pub fn find_in_radius(&self, x: f64, y: f64, z: f64, radius: f64) -> Vec<RoadObject<'_>> {
        maliput_sys::api::objects::ffi::RoadObjectBook_FindInRadius(self.road_object_book, x, y, z, radius)
            .into_iter()
            .map(|ptr| RoadObject {
                road_object: unsafe { ptr.road_object.as_ref().expect("RoadObject pointer is null") },
            })
            .collect()
    }
}

/// Domain alias for road marking semantic types.
pub type RoadMarkingType = crate::api::rules::TrafficControlDeviceType;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
/// Unit of a [RoadMarkingValue].
pub enum RoadMarkingValueUnit {
    MetersPerSecond,
    KilometersPerHour,
    MilesPerHour,
}

fn road_marking_value_unit_from_cpp(u: maliput_sys::api::objects::ffi::RoadMarkingValueUnit) -> RoadMarkingValueUnit {
    match u {
        maliput_sys::api::objects::ffi::RoadMarkingValueUnit::kMetersPerSecond => RoadMarkingValueUnit::MetersPerSecond,
        maliput_sys::api::objects::ffi::RoadMarkingValueUnit::kKilometersPerHour => {
            RoadMarkingValueUnit::KilometersPerHour
        }
        maliput_sys::api::objects::ffi::RoadMarkingValueUnit::kMilesPerHour => RoadMarkingValueUnit::MilesPerHour,
        _ => RoadMarkingValueUnit::MetersPerSecond,
    }
}

/// Numeric value (with unit) associated with a [RoadMarking] (e.g. a speed limit).
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RoadMarkingValue {
    pub value: f64,
    pub unit: RoadMarkingValueUnit,
}

/// Models a physical road marking (e.g. stop line, crosswalk, arrow) in the road network.
pub struct RoadMarking<'a> {
    pub(crate) road_marking: &'a maliput_sys::api::objects::ffi::RoadMarking,
}

impl<'a> RoadMarking<'a> {
    /// Returns the unique identifier of this road marking.
    pub fn id(&self) -> String {
        maliput_sys::api::objects::ffi::RoadMarking_id(self.road_marking)
    }

    /// Returns the optional human-readable name of this road marking.
    pub fn name(&self) -> Option<String> {
        let wrapper = maliput_sys::api::objects::ffi::RoadMarking_name(self.road_marking);
        if wrapper.is_null() {
            return None;
        }
        Some(wrapper.value.clone())
    }

    /// Returns the [RoadMarkingType] of this road marking.
    pub fn marking_type(&self) -> RoadMarkingType {
        let t = maliput_sys::api::objects::ffi::RoadMarking_marking_type(self.road_marking);
        crate::api::rules::traffic_control_device_type_from_cpp(&t)
    }

    /// Returns the [RoadObjectPosition] of this road marking.
    pub fn position(&self) -> RoadObjectPosition {
        let inertial_position = maliput_sys::api::objects::ffi::RoadMarking_position_inertial(self.road_marking);
        let has_lane = maliput_sys::api::objects::ffi::RoadMarking_position_has_lane_position(self.road_marking);
        let lane_position = if has_lane {
            Some((
                maliput_sys::api::objects::ffi::RoadMarking_position_lane_id(self.road_marking),
                maliput_sys::api::objects::ffi::RoadMarking_position_lane_s(self.road_marking),
                maliput_sys::api::objects::ffi::RoadMarking_position_lane_r(self.road_marking),
                maliput_sys::api::objects::ffi::RoadMarking_position_lane_h(self.road_marking),
            ))
        } else {
            None
        };
        RoadObjectPosition {
            inertial_position: crate::api::InertialPosition { ip: inertial_position },
            lane_position,
        }
    }

    /// Returns the orientation of this road marking in the inertial frame.
    pub fn orientation(&self) -> crate::api::Rotation {
        let rotation = maliput_sys::api::objects::ffi::RoadMarking_orientation(self.road_marking);
        crate::api::Rotation { r: rotation }
    }

    /// Returns the bounding box of this road marking.
    pub fn bounding_box(&self) -> crate::math::BoundingBox {
        let b = maliput_sys::api::objects::ffi::RoadMarking_bounding_box(self.road_marking);
        crate::math::BoundingBox { b }
    }

    /// Returns the IDs of lanes related to this road marking.
    pub fn related_lanes(&self) -> Vec<String> {
        maliput_sys::api::objects::ffi::RoadMarking_related_lanes(self.road_marking)
    }

    /// Returns the number of outlines of this road marking.
    pub fn num_outlines(&self) -> i32 {
        maliput_sys::api::objects::ffi::RoadMarking::num_outlines(self.road_marking)
    }

    /// Returns the outlines of this road marking.
    pub fn outlines(&self) -> Vec<Outline<'_>> {
        maliput_sys::api::objects::ffi::RoadMarking_outlines(self.road_marking)
            .into_iter()
            .map(|ptr| Outline {
                outline: unsafe { ptr.outline.as_ref().expect("Outline pointer is null") },
            })
            .collect()
    }

    /// Returns the numeric value (with unit) associated with this road marking, if any.
    pub fn value(&self) -> Option<RoadMarkingValue> {
        let data = maliput_sys::api::objects::ffi::RoadMarking_value(self.road_marking);
        if !data.has_value {
            return None;
        }
        Some(RoadMarkingValue {
            value: data.value,
            unit: road_marking_value_unit_from_cpp(data.unit),
        })
    }
}

/// Interface for accessing [RoadMarking]s in the road network.
pub struct RoadMarkingBook<'a> {
    pub(super) road_marking_book: &'a maliput_sys::api::objects::ffi::RoadMarkingBook,
}

impl<'a> RoadMarkingBook<'a> {
    /// Returns all [RoadMarking]s in the book.
    pub fn road_markings(&self) -> Vec<RoadMarking<'_>> {
        maliput_sys::api::objects::ffi::RoadMarkingBook_RoadMarkings(self.road_marking_book)
            .into_iter()
            .map(|ptr| RoadMarking {
                road_marking: unsafe { ptr.road_marking.as_ref().expect("RoadMarking pointer is null") },
            })
            .collect()
    }

    /// Returns the [RoadMarking] with the given `id`, or `None` if not found.
    pub fn get_road_marking(&self, id: &String) -> Option<RoadMarking<'_>> {
        let ptr = maliput_sys::api::objects::ffi::RoadMarkingBook_GetRoadMarking(self.road_marking_book, id);
        if ptr.is_null() {
            return None;
        }
        Some(RoadMarking {
            road_marking: unsafe { ptr.as_ref().expect("Unable to get underlying RoadMarking pointer") },
        })
    }

    /// Returns all [RoadMarking]s whose `related_lanes()` includes the given lane ID.
    pub fn find_by_lane(&self, lane_id: &String) -> Vec<RoadMarking<'_>> {
        maliput_sys::api::objects::ffi::RoadMarkingBook_FindByLane(self.road_marking_book, lane_id)
            .into_iter()
            .map(|ptr| RoadMarking {
                road_marking: unsafe { ptr.road_marking.as_ref().expect("RoadMarking pointer is null") },
            })
            .collect()
    }

    /// Returns all [RoadMarking]s of the given [RoadMarkingType].
    pub fn find_by_type(&self, marking_type: &RoadMarkingType) -> Vec<RoadMarking<'_>> {
        let t_ffi = crate::api::rules::traffic_control_device_type_to_cpp(marking_type);
        maliput_sys::api::objects::ffi::RoadMarkingBook_FindByType(self.road_marking_book, t_ffi)
            .into_iter()
            .map(|ptr| RoadMarking {
                road_marking: unsafe { ptr.road_marking.as_ref().expect("RoadMarking pointer is null") },
            })
            .collect()
    }
}
