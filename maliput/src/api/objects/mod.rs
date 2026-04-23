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

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
/// Defines the possible road object types.
pub enum RoadObjectType {
    Unknown,
    Barrier,
    Building,
    Crosswalk,
    Gantry,
    Obstacle,
    ParkingSpace,
    Pole,
    RoadMark,
    RoadSurface,
    StopLine,
    TrafficIsland,
    Tree,
    Vegetation,
}

fn road_object_type_from_cpp(obj_type: maliput_sys::api::objects::ffi::RoadObjectType) -> RoadObjectType {
    match obj_type {
        maliput_sys::api::objects::ffi::RoadObjectType::kUnknown => RoadObjectType::Unknown,
        maliput_sys::api::objects::ffi::RoadObjectType::kBarrier => RoadObjectType::Barrier,
        maliput_sys::api::objects::ffi::RoadObjectType::kBuilding => RoadObjectType::Building,
        maliput_sys::api::objects::ffi::RoadObjectType::kCrosswalk => RoadObjectType::Crosswalk,
        maliput_sys::api::objects::ffi::RoadObjectType::kGantry => RoadObjectType::Gantry,
        maliput_sys::api::objects::ffi::RoadObjectType::kObstacle => RoadObjectType::Obstacle,
        maliput_sys::api::objects::ffi::RoadObjectType::kParkingSpace => RoadObjectType::ParkingSpace,
        maliput_sys::api::objects::ffi::RoadObjectType::kPole => RoadObjectType::Pole,
        maliput_sys::api::objects::ffi::RoadObjectType::kRoadMark => RoadObjectType::RoadMark,
        maliput_sys::api::objects::ffi::RoadObjectType::kRoadSurface => RoadObjectType::RoadSurface,
        maliput_sys::api::objects::ffi::RoadObjectType::kStopLine => RoadObjectType::StopLine,
        maliput_sys::api::objects::ffi::RoadObjectType::kTrafficIsland => RoadObjectType::TrafficIsland,
        maliput_sys::api::objects::ffi::RoadObjectType::kTree => RoadObjectType::Tree,
        maliput_sys::api::objects::ffi::RoadObjectType::kVegetation => RoadObjectType::Vegetation,
        _ => RoadObjectType::Unknown,
    }
}

fn road_object_type_to_cpp(obj_type: &RoadObjectType) -> maliput_sys::api::objects::ffi::RoadObjectType {
    match obj_type {
        RoadObjectType::Unknown => maliput_sys::api::objects::ffi::RoadObjectType::kUnknown,
        RoadObjectType::Barrier => maliput_sys::api::objects::ffi::RoadObjectType::kBarrier,
        RoadObjectType::Building => maliput_sys::api::objects::ffi::RoadObjectType::kBuilding,
        RoadObjectType::Crosswalk => maliput_sys::api::objects::ffi::RoadObjectType::kCrosswalk,
        RoadObjectType::Gantry => maliput_sys::api::objects::ffi::RoadObjectType::kGantry,
        RoadObjectType::Obstacle => maliput_sys::api::objects::ffi::RoadObjectType::kObstacle,
        RoadObjectType::ParkingSpace => maliput_sys::api::objects::ffi::RoadObjectType::kParkingSpace,
        RoadObjectType::Pole => maliput_sys::api::objects::ffi::RoadObjectType::kPole,
        RoadObjectType::RoadMark => maliput_sys::api::objects::ffi::RoadObjectType::kRoadMark,
        RoadObjectType::RoadSurface => maliput_sys::api::objects::ffi::RoadObjectType::kRoadSurface,
        RoadObjectType::StopLine => maliput_sys::api::objects::ffi::RoadObjectType::kStopLine,
        RoadObjectType::TrafficIsland => maliput_sys::api::objects::ffi::RoadObjectType::kTrafficIsland,
        RoadObjectType::Tree => maliput_sys::api::objects::ffi::RoadObjectType::kTree,
        RoadObjectType::Vegetation => maliput_sys::api::objects::ffi::RoadObjectType::kVegetation,
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
