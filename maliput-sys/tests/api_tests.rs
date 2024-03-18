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

#[cfg(test)]
mod api_test {
    mod lane_position_test {
        use maliput_sys::api::ffi::LanePosition_new;
        use maliput_sys::api::ffi::LanePosition_to_str;

        use maliput_sys::math::ffi::Vector3_new;

        #[test]
        fn laneposition_new() {
            let lane_pos = LanePosition_new(1.0, 2.0, 3.0);
            assert_eq!(lane_pos.s(), 1.0);
            assert_eq!(lane_pos.r(), 2.0);
            assert_eq!(lane_pos.h(), 3.0);
        }

        #[test]
        fn srh() {
            let lane_pos = LanePosition_new(1.0, 2.0, 3.0);
            assert_eq!(lane_pos.srh().norm(), 3.7416573867739413);
        }

        #[test]
        fn set_s() {
            let mut lane_pos = LanePosition_new(1.0, 2.0, 3.0);
            lane_pos.as_mut().expect("").set_s(4.0);
            assert_eq!(lane_pos.s(), 4.0);
        }

        #[test]
        fn set_r() {
            let mut lane_pos = LanePosition_new(1.0, 2.0, 3.0);
            lane_pos.as_mut().expect("").set_r(4.0);
            assert_eq!(lane_pos.r(), 4.0);
        }

        #[test]
        fn set_h() {
            let mut lane_pos = LanePosition_new(1.0, 2.0, 3.0);
            lane_pos.as_mut().expect("").set_h(4.0);
            assert_eq!(lane_pos.h(), 4.0);
        }

        #[test]
        fn set_srh() {
            let mut lane_pos = LanePosition_new(1.0, 2.0, 3.0);
            let vector = Vector3_new(4.0, 5.0, 6.0);
            lane_pos.as_mut().expect("").set_srh(&vector);
            assert_eq!(lane_pos.s(), 4.0);
            assert_eq!(lane_pos.r(), 5.0);
            assert_eq!(lane_pos.h(), 6.0);
        }

        #[test]
        fn lane_position_to_str() {
            let lane_pos = LanePosition_new(1.0, 2.0, 3.0);
            assert_eq!(LanePosition_to_str(&lane_pos), "(s = 1, r = 2, h = 3)");
        }
    }

    mod inertial_position_test {
        use maliput_sys::api::ffi::InertialPosition_new;
        use maliput_sys::api::ffi::InertialPosition_operator_eq;
        use maliput_sys::api::ffi::InertialPosition_operator_mul_scalar;
        use maliput_sys::api::ffi::InertialPosition_operator_sub;
        use maliput_sys::api::ffi::InertialPosition_operator_sum;
        use maliput_sys::api::ffi::InertialPosition_to_str;

        use maliput_sys::math::ffi::Vector3_new;

        #[test]
        fn inertial_position_new() {
            let inertial_pos = InertialPosition_new(1.0, 2.0, 3.0);
            assert_eq!(inertial_pos.x(), 1.0);
            assert_eq!(inertial_pos.y(), 2.0);
            assert_eq!(inertial_pos.z(), 3.0);
        }

        #[test]
        fn inertial_position_setters() {
            let mut inertial_pos = InertialPosition_new(1.0, 2.0, 3.0);
            inertial_pos.as_mut().expect("").set_x(4.0);
            inertial_pos.as_mut().expect("").set_y(5.0);
            inertial_pos.as_mut().expect("").set_z(6.0);
            assert_eq!(inertial_pos.x(), 4.0);
            assert_eq!(inertial_pos.y(), 5.0);
            assert_eq!(inertial_pos.z(), 6.0);
            inertial_pos.as_mut().expect("").set_xyz(&Vector3_new(7.0, 8.0, 9.0));
            assert_eq!(inertial_pos.x(), 7.0);
            assert_eq!(inertial_pos.y(), 8.0);
            assert_eq!(inertial_pos.z(), 9.0);
        }

        #[test]
        fn inertial_position_xyz() {
            let inertial_pos = InertialPosition_new(1.0, 2.0, 3.0);
            assert_eq!(inertial_pos.xyz().x(), 1.0);
            assert_eq!(inertial_pos.xyz().y(), 2.0);
            assert_eq!(inertial_pos.xyz().z(), 3.0);
        }

        #[test]
        fn inertial_position_length() {
            let inertial_pos = InertialPosition_new(1.0, 2.0, 3.0);
            assert_eq!(inertial_pos.length(), 3.7416573867739413);
        }

        #[test]
        fn inertial_position_distance() {
            let inertial_pos1 = InertialPosition_new(1.0, 1.0, 1.0);
            let inertial_pos2 = InertialPosition_new(5.0, 1.0, 1.0);
            assert_eq!(inertial_pos1.Distance(&inertial_pos2), 4.0);
        }

        #[test]
        fn inertial_position_operator_eq() {
            let inertial_pos1 = InertialPosition_new(1.0, 2.0, 3.0);
            let inertial_pos2 = InertialPosition_new(1.0, 2.0, 3.0);
            assert!(InertialPosition_operator_eq(&inertial_pos1, &inertial_pos2));
            let inertial_pos3 = InertialPosition_new(4.0, 5.0, 6.0);
            assert!(!InertialPosition_operator_eq(&inertial_pos1, &inertial_pos3));
        }

        #[test]
        fn inertial_position_operator_sum() {
            let inertial_pos1 = InertialPosition_new(1.0, 2.0, 3.0);
            let inertial_pos2 = InertialPosition_new(4.0, 5.0, 6.0);
            let inertial_pos3 = InertialPosition_operator_sum(&inertial_pos1, &inertial_pos2);
            assert_eq!(inertial_pos3.x(), 5.0);
            assert_eq!(inertial_pos3.y(), 7.0);
            assert_eq!(inertial_pos3.z(), 9.0);
        }

        #[test]
        fn inertial_position_operator_sub() {
            let inertial_pos1 = InertialPosition_new(1.0, 2.0, 3.0);
            let inertial_pos2 = InertialPosition_new(4.0, 5.0, 6.0);
            let inertial_pos3 = InertialPosition_operator_sub(&inertial_pos1, &inertial_pos2);
            assert_eq!(inertial_pos3.x(), -3.0);
            assert_eq!(inertial_pos3.y(), -3.0);
            assert_eq!(inertial_pos3.z(), -3.0);
        }

        #[test]
        fn inertial_position_operator_mul_scalar() {
            let inertial_pos1 = InertialPosition_new(1.0, 2.0, 3.0);
            let inertial_pos2 = InertialPosition_operator_mul_scalar(&inertial_pos1, 2.0);
            assert_eq!(inertial_pos2.x(), 2.0);
            assert_eq!(inertial_pos2.y(), 4.0);
            assert_eq!(inertial_pos2.z(), 6.0);
        }
        #[test]
        fn inertial_position_to_str() {
            let inertial_pos = InertialPosition_new(1.0, 2.0, 3.0);
            assert_eq!(InertialPosition_to_str(&inertial_pos), "(x = 1, y = 2, z = 3)");
        }
    }
}
