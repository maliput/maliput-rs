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

#[cxx::bridge(namespace = "maliput::api::rules")]
#[allow(clippy::needless_lifetimes)] // Clippy bug: https://github.com/rust-lang/rust-clippy/issues/5787
pub mod ffi {
    /// Shared struct for `TrafficLight` pointers.
    /// This is needed because `*const` can't be used directly in the CxxVector collection.
    struct ConstTrafficLightPtr {
        pub traffic_light: *const TrafficLight,
    }
    /// Shared struct for `BulbGroup` pointers.
    /// This is needed because `*const` can't be used directly in the CxxVector collection.
    struct ConstBulbGroupPtr {
        pub bulb_group: *const BulbGroup,
    }
    /// Shared struct for `Bulb` pointers.
    /// This is needed because `*const` can't be used directly in the CxxVector collection.
    struct ConstBulbPtr {
        pub bulb: *const Bulb,
    }
    /// Shared struct for `BulbState` references.
    /// This is needed because `&f` can't be used directly in the CxxVector collection.
    struct ConstBulbStateRef<'a> {
        pub bulb_state: &'a BulbState,
    }
    /// Shared struct for floats types.
    /// This is needed because `f64` can't be used directly in the UniquePtr type.
    struct FloatWrapper {
        pub value: f64,
    }
    /// Shared struct for pairs in a RelatedRules collection.
    ///  - key: Group name of the rules.
    ///  - value: Rule ids.
    /// This is needed because maps can't be binded directly.
    struct RelatedRule {
        pub group_name: String,
        pub rule_ids: Vec<String>,
    }
    /// Shared struct for pairs in a RelatedRules collection.
    ///  - key: Group name.
    ///  - value: Unique Ids.
    /// This is needed because maps can't be binded directly.
    struct RelatedUniqueId {
        pub group_name: String,
        pub unique_ids: Vec<String>,
    }

    /// Shared struct for `LaneSRange` constant reference.
    /// Interestingly this was done at maliput::api module but
    /// couldn't reference to that so it was necessary to
    /// redefine it here.
    struct ConstLaneSRangeRef<'a> {
        pub lane_s_range: &'a LaneSRange,
    }

    #[repr(i32)]
    enum BulbColor {
        kRed = 0,
        kYellow,
        kGreen,
    }

    #[repr(i32)]
    enum BulbType {
        kRound = 0,
        kArrow,
    }

    #[repr(i32)]
    enum BulbState {
        kOff = 0,
        kOn,
        kBlinking,
    }

    unsafe extern "C++" {
        include!("api/rules/rules.h");
        include!("api/rules/aliases.h");

        // Forward declarations
        #[namespace = "maliput::api"]
        type InertialPosition = crate::api::ffi::InertialPosition;
        #[namespace = "maliput::api"]
        type Rotation = crate::api::ffi::Rotation;
        #[namespace = "maliput::api"]
        type LaneSRange = crate::api::ffi::LaneSRange;
        #[namespace = "maliput::api"]
        type LaneSRoute = crate::api::ffi::LaneSRoute;
        #[namespace = "maliput::math"]
        type Vector3 = crate::math::ffi::Vector3;

        // TrafficLightBook bindings definitions.
        type TrafficLightBook;
        fn TrafficLightBook_TrafficLights(book: &TrafficLightBook) -> UniquePtr<CxxVector<ConstTrafficLightPtr>>;
        fn TrafficLightBook_GetTrafficLight(book: &TrafficLightBook, id: &String) -> *const TrafficLight;

        // TrafficLight bindings definitions.
        type TrafficLight;
        fn TrafficLight_id(traffic_light: &TrafficLight) -> String;
        fn TrafficLight_position_road_network(traffic_light: &TrafficLight) -> UniquePtr<InertialPosition>;
        fn TrafficLight_orientation_road_network(traffic_light: &TrafficLight) -> UniquePtr<Rotation>;
        fn TrafficLight_bulb_groups(traffic_light: &TrafficLight) -> UniquePtr<CxxVector<ConstBulbGroupPtr>>;
        fn TrafficLight_GetBulbGroup(traffic_light: &TrafficLight, id: &String) -> *const BulbGroup;

        type BulbColor;
        type BulbState;
        type BulbType;
        // Bulb bindings definitions.
        type Bulb;
        fn Bulb_id(bulb: &Bulb) -> String;
        fn Bulb_unique_id(bulb: &Bulb) -> UniquePtr<UniqueBulbId>;
        fn Bulb_position_bulb_group(bulb: &Bulb) -> UniquePtr<InertialPosition>;
        fn Bulb_orientation_bulb_group(bulb: &Bulb) -> UniquePtr<Rotation>;
        fn color(self: &Bulb) -> &BulbColor;
        // We can't automatically use the name `type` as it is a reserved keyword in Rust.
        fn Bulb_type(bulb: &Bulb) -> &BulbType;
        fn Bulb_arrow_orientation_rad(bulb: &Bulb) -> UniquePtr<FloatWrapper>;
        fn Bulb_states(bulb: &Bulb) -> UniquePtr<CxxVector<BulbState>>;
        fn GetDefaultState(self: &Bulb) -> BulbState;
        fn IsValidState(self: &Bulb, state: &BulbState) -> bool;
        fn Bulb_bounding_box_min(bulb: &Bulb) -> UniquePtr<Vector3>;
        fn Bulb_bounding_box_max(bulb: &Bulb) -> UniquePtr<Vector3>;
        fn Bulb_bulb_group(bulb: &Bulb) -> *const BulbGroup;

        // BulbGroup bindings definitions.
        type BulbGroup;
        fn BulbGroup_id(bulb_group: &BulbGroup) -> String;
        fn BulbGroup_unique_id(bulb: &BulbGroup) -> UniquePtr<UniqueBulbGroupId>;
        fn BulbGroup_position_traffic_light(bulb_group: &BulbGroup) -> UniquePtr<InertialPosition>;
        fn BulbGroup_orientation_traffic_light(bulb_group: &BulbGroup) -> UniquePtr<Rotation>;
        fn BulbGroup_bulbs(bulb_group: &BulbGroup) -> UniquePtr<CxxVector<ConstBulbPtr>>;
        fn BulbGroup_GetBulb(bulb_group: &BulbGroup, id: &String) -> *const Bulb;
        fn BulbGroup_traffic_light(bulb_group: &BulbGroup) -> *const TrafficLight;

        // UniqueBulbId bindings definitions.
        type UniqueBulbId;
        fn string(self: &UniqueBulbId) -> &CxxString;
        fn UniqueBulbId_traffic_light_id(id: &UniqueBulbId) -> String;
        fn UniqueBulbId_bulb_group_id(id: &UniqueBulbId) -> String;
        fn UniqueBulbId_bulb_id(id: &UniqueBulbId) -> String;

        // UniqueBulbGroupId bindings definitions.
        type UniqueBulbGroupId;
        fn string(self: &UniqueBulbGroupId) -> &CxxString;
        fn UniqueBulbGroupId_traffic_light_id(id: &UniqueBulbGroupId) -> String;
        fn UniqueBulbGroupId_bulb_group_id(id: &UniqueBulbGroupId) -> String;

        // QueryResults bindings definitions.
        type QueryResults;
        fn QueryResults_discrete_value_rules(query_results: &QueryResults) -> Vec<String>;
        fn QueryResults_range_value_rules(query_results: &QueryResults) -> Vec<String>;

        // RoadRulebook bindings definitions.
        type RoadRulebook;
        fn RoadRulebook_GetDiscreteValueRule(book: &RoadRulebook, rule_id: &String) -> UniquePtr<DiscreteValueRule>;
        fn RoadRulebook_GetRangeValueRule(book: &RoadRulebook, rule_id: &String) -> UniquePtr<RangeValueRule>;
        fn RoadRulebook_Rules(book: &RoadRulebook) -> UniquePtr<QueryResults>;
        #[allow(clippy::needless_lifetimes)]
        fn RoadRulebook_FindRules(
            book: &RoadRulebook,
            ranges: &Vec<ConstLaneSRangeRef>,
            tolerance: f64,
        ) -> UniquePtr<QueryResults>;

        // DiscreteValueRule::DiscreteValue bindings definitions.
        type DiscreteValueRuleDiscreteValue;
        fn DiscreteValueRuleDiscreteValue_value(value: &DiscreteValueRuleDiscreteValue) -> String;
        fn DiscreteValueRuleDiscreteValue_severity(value: &DiscreteValueRuleDiscreteValue) -> i32;
        fn DiscreteValueRuleDiscreteValue_related_rules(
            value: &DiscreteValueRuleDiscreteValue,
        ) -> UniquePtr<CxxVector<RelatedRule>>;
        fn DiscreteValueRuleDiscreteValue_related_unique_ids(
            value: &DiscreteValueRuleDiscreteValue,
        ) -> UniquePtr<CxxVector<RelatedUniqueId>>;

        // DiscreteValueRule bindings definitions.
        type DiscreteValueRule;
        fn states(self: &DiscreteValueRule) -> &CxxVector<DiscreteValueRuleDiscreteValue>;
        fn DiscreteValueRule_id(rule: &DiscreteValueRule) -> String;
        fn DiscreteValueRule_type_id(rule: &DiscreteValueRule) -> String;
        fn DiscreteValueRule_zone(rule: &DiscreteValueRule) -> UniquePtr<LaneSRoute>;

        // RangeValueRule::Range bindings definitions.
        type RangeValueRuleRange;
        fn RangeValueRuleRange_description(range: &RangeValueRuleRange) -> String;
        fn RangeValueRuleRange_min(range: &RangeValueRuleRange) -> f64;
        fn RangeValueRuleRange_max(range: &RangeValueRuleRange) -> f64;
        fn RangeValueRuleRange_severity(range: &RangeValueRuleRange) -> i32;
        fn RangeValueRuleRange_related_rules(range: &RangeValueRuleRange) -> UniquePtr<CxxVector<RelatedRule>>;
        fn RangeValueRuleRange_related_unique_ids(range: &RangeValueRuleRange)
            -> UniquePtr<CxxVector<RelatedUniqueId>>;
        // RangeValueRule::Range bindings definitions.
        type RangeValueRule;
        fn RangeValueRule_id(rule: &RangeValueRule) -> String;
        fn RangeValueRule_type_id(rule: &RangeValueRule) -> String;
        fn RangeValueRule_zone(rule: &RangeValueRule) -> UniquePtr<LaneSRoute>;
        fn states(self: &RangeValueRule) -> &CxxVector<RangeValueRuleRange>;
    }
}
