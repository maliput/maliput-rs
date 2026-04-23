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

#[cxx::bridge(namespace = "maliput::api::objects")]
#[allow(clippy::needless_lifetimes)] // Clippy bug: https://github.com/rust-lang/rust-clippy/issues/5787
pub mod ffi {
    /// Shared struct for `RoadObject` pointers.
    /// This is needed because `*const` can't be used directly in the CxxVector collection.
    struct ConstRoadObjectPtr {
        pub road_object: *const RoadObject,
    }
    /// Shared struct for `Outline` pointers.
    /// This is needed because `*const` can't be used directly in the CxxVector collection.
    struct ConstOutlinePtr {
        pub outline: *const Outline,
    }
    /// Shared struct for outline corner data.
    /// This is a flat representation of `OutlineCorner` to avoid a new opaque CXX type.
    /// `height` is 0.0 when `has_height` is false.
    struct OutlineCornerData {
        pub x: f64,
        pub y: f64,
        pub z: f64,
        pub has_height: bool,
        pub height: f64,
    }
    /// Shared struct for pairs in a properties map.
    /// This is needed because maps can't be bound directly.
    struct StringPair {
        pub key: String,
        pub value: String,
    }

    /// Shared enum representing different types of road objects.
    /// This is needed to access the enum variant from Rust API since the C++ enum has an opaque implementation.
    /// The order of these variants must match with the order of the enum class defined in maliput C++ API.
    #[repr(i32)]
    enum RoadObjectType {
        kUnknown = 0,
        kBarrier,
        kBuilding,
        kCrosswalk,
        kGantry,
        kObstacle,
        kParkingSpace,
        kPole,
        kRoadMark,
        kRoadSurface,
        kStopLine,
        kTrafficIsland,
        kTree,
        kVegetation,
    }

    unsafe extern "C++" {
        include!("api/objects/objects.h");
        include!("cxx_utils/error_handling.h");

        // Forward declarations from parent namespaces.
        #[namespace = "maliput::api"]
        type InertialPosition = crate::api::ffi::InertialPosition;
        #[namespace = "maliput::api"]
        type Rotation = crate::api::ffi::Rotation;
        #[namespace = "maliput::math"]
        type Vector3 = crate::math::ffi::Vector3;
        #[namespace = "maliput::math"]
        type BoundingBox = crate::math::ffi::BoundingBox;
        // StringWrapper is reused from the rules bridge to avoid duplication.
        #[namespace = "maliput::api::rules"]
        type StringWrapper = crate::api::rules::ffi::StringWrapper;

        // RoadObjectType opaque type - this is needed to prevent CXX from redefining the enum (it'll use a `using` alias instead).
        type RoadObjectType;

        // RoadObjectBook opaque type and bindings.
        type RoadObjectBook;
        fn RoadObjectBook_RoadObjects(book: &RoadObjectBook) -> UniquePtr<CxxVector<ConstRoadObjectPtr>>;
        fn RoadObjectBook_GetRoadObject(book: &RoadObjectBook, id: &String) -> *const RoadObject;
        fn RoadObjectBook_FindByType(
            book: &RoadObjectBook,
            obj_type: RoadObjectType,
        ) -> UniquePtr<CxxVector<ConstRoadObjectPtr>>;
        fn RoadObjectBook_FindByLane(
            book: &RoadObjectBook,
            lane_id: &String,
        ) -> UniquePtr<CxxVector<ConstRoadObjectPtr>>;
        fn RoadObjectBook_FindInRadius(
            book: &RoadObjectBook,
            x: f64,
            y: f64,
            z: f64,
            radius: f64,
        ) -> UniquePtr<CxxVector<ConstRoadObjectPtr>>;

        // RoadObject opaque type and bindings.
        type RoadObject;
        fn RoadObject_id(obj: &RoadObject) -> String;
        fn RoadObject_name(obj: &RoadObject) -> UniquePtr<StringWrapper>;
        // This method could be bound as `fn type(self: &RoadObject) -> RoadObjectType` but it
        // causes a conflict with the `type` keyword in Rust.
        fn RoadObject_object_type(obj: &RoadObject) -> RoadObjectType;
        fn RoadObject_subtype(obj: &RoadObject) -> UniquePtr<StringWrapper>;
        fn RoadObject_position_inertial(obj: &RoadObject) -> UniquePtr<InertialPosition>;
        fn RoadObject_position_has_lane_position(obj: &RoadObject) -> bool;
        fn RoadObject_position_lane_id(obj: &RoadObject) -> String;
        fn RoadObject_position_lane_s(obj: &RoadObject) -> f64;
        fn RoadObject_position_lane_r(obj: &RoadObject) -> f64;
        fn RoadObject_position_lane_h(obj: &RoadObject) -> f64;
        fn RoadObject_orientation(obj: &RoadObject) -> UniquePtr<Rotation>;
        fn RoadObject_bounding_box(obj: &RoadObject) -> UniquePtr<BoundingBox>;
        fn is_dynamic(self: &RoadObject) -> bool;
        fn RoadObject_related_lanes(obj: &RoadObject) -> Vec<String>;
        fn num_outlines(self: &RoadObject) -> i32;
        fn RoadObject_outlines(obj: &RoadObject) -> UniquePtr<CxxVector<ConstOutlinePtr>>;
        fn RoadObject_properties(obj: &RoadObject) -> Vec<StringPair>;

        // Outline opaque type and bindings.
        type Outline;
        fn Outline_id(outline: &Outline) -> String;
        fn Outline_is_closed(outline: &Outline) -> bool;
        fn Outline_num_corners(outline: &Outline) -> i32;
        fn Outline_corners(outline: &Outline) -> Vec<OutlineCornerData>;
    }
}
