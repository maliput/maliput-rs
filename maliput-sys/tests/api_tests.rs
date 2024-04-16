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
    mod rotation_test {
        use std::f64::consts::PI;

        use maliput_sys::api::ffi::InertialPosition_new;
        use maliput_sys::api::ffi::Rotation;
        use maliput_sys::api::ffi::Rotation_Apply;
        use maliput_sys::api::ffi::Rotation_Reverse;
        use maliput_sys::api::ffi::Rotation_from_quat;
        use maliput_sys::api::ffi::Rotation_from_rpy;
        use maliput_sys::api::ffi::Rotation_matrix;
        use maliput_sys::api::ffi::Rotation_new;
        use maliput_sys::api::ffi::Rotation_set_quat;

        use maliput_sys::math::ffi::Matrix3;
        use maliput_sys::math::ffi::Matrix3_new;
        use maliput_sys::math::ffi::Matrix3_row;
        use maliput_sys::math::ffi::Quaternion_new;
        use maliput_sys::math::ffi::RollPitchYaw_new;
        use maliput_sys::math::ffi::Vector3_equals;

        #[allow(clippy::too_many_arguments)]
        fn check_all_rotation_accessors(
            rotation: &Rotation,
            roll: f64,
            pitch: f64,
            yaw: f64,
            w: f64,
            x: f64,
            y: f64,
            z: f64,
            matrix: &Matrix3,
        ) {
            assert_eq!(rotation.roll(), roll);
            assert_eq!(rotation.pitch(), pitch);
            assert_eq!(rotation.yaw(), yaw);
            assert_eq!(rotation.quat().w(), w);
            assert_eq!(rotation.quat().x(), x);
            assert_eq!(rotation.quat().y(), y);
            assert_eq!(rotation.quat().z(), z);
            let rot_matrix = Rotation_matrix(rotation);
            assert!(Vector3_equals(&Matrix3_row(&rot_matrix, 0), &Matrix3_row(matrix, 0)));
        }

        #[test]
        fn rotation_constructors() {
            let rotation = Rotation_new();
            check_all_rotation_accessors(
                &rotation,
                0.,
                0.,
                0.,
                1.,
                0.,
                0.,
                0.,
                &Matrix3_new(1., 0., 0., 0., 1., 0., 0., 0., 1.),
            );
            let rpy = RollPitchYaw_new(1.0, 0.0, 0.0);
            let rotation_from_rpy = Rotation_from_rpy(&rpy);
            assert_eq!(rotation_from_rpy.roll(), 1.);
            let quat = Quaternion_new(1.0, 0.0, 0.0, 0.0);
            let rotation_from_quat = Rotation_from_quat(&quat);
            assert_eq!(rotation_from_quat.roll(), 0.);
        }

        #[test]
        fn rotation_set_quat() {
            let mut rotation = Rotation_new();
            let quat = Quaternion_new(1.0, 0.0, 0.0, 0.0);
            Rotation_set_quat(rotation.pin_mut(), &quat);
            check_all_rotation_accessors(
                &rotation,
                0.,
                0.,
                0.,
                1.,
                0.,
                0.,
                0.,
                &Matrix3_new(1., 0., 0., 0., 1., 0., 0., 0., 1.),
            );
        }

        #[test]
        fn rotation_distance() {
            let rotation1 = Rotation_new();
            let rotation2 = Rotation_new();
            assert_eq!(rotation1.Distance(&rotation2), 0.);
        }

        #[test]
        fn rotation_apply() {
            let rotation = Rotation_new();
            let inertial_pos = InertialPosition_new(1., 0., 0.);
            let rotated_vector = Rotation_Apply(&rotation, &inertial_pos);
            assert_eq!(rotated_vector.x(), 1.);
            assert_eq!(rotated_vector.y(), 0.);
            assert_eq!(rotated_vector.z(), 0.);
        }

        #[test]
        fn rotation_reverse() {
            let tol = 1e-10;
            let rotation = Rotation_new();
            let reversed_rotation = Rotation_Reverse(&rotation);
            assert!((reversed_rotation.yaw() - PI).abs() < tol);
        }
    }

    mod s_range_test {
        use maliput_sys::api::ffi::SRange_GetIntersection;
        use maliput_sys::api::ffi::SRange_new;

        #[test]
        fn s_range_api() {
            let mut s_range = SRange_new(1.0, 11.0);
            assert_eq!(s_range.s0(), 1.0);
            assert_eq!(s_range.s1(), 11.0);
            assert_eq!(s_range.size(), 10.0);
            s_range.as_mut().expect("").set_s0(2.0);
            s_range.as_mut().expect("").set_s1(12.0);
            assert_eq!(s_range.s0(), 2.0);
            assert_eq!(s_range.s1(), 12.0);
            assert!(s_range.WithS());

            let s_range_2 = SRange_new(5.0, 20.0);
            assert!(s_range.Intersects(&s_range_2, 1e-3));
            assert!(!s_range.Contains(&s_range_2, 1e-3));

            let intersection = SRange_GetIntersection(&s_range, &s_range_2, 1e-3);
            assert!(!intersection.is_null());
            assert_eq!(intersection.s0(), 5.0);
            assert_eq!(intersection.s1(), 12.0);

            let non_intersected_s_range = SRange_new(150.0, 200.0);
            assert!(!s_range.Intersects(&non_intersected_s_range, 1e-3));
            let intersection = SRange_GetIntersection(&s_range, &non_intersected_s_range, 1e-3);
            assert!(intersection.is_null());
        }
    }

    mod lane_s_range_test {
        use maliput_sys::api::ffi::LaneSRange_GetIntersection;
        use maliput_sys::api::ffi::LaneSRange_lane_id;
        use maliput_sys::api::ffi::LaneSRange_new;
        use maliput_sys::api::ffi::LaneSRange_s_range;
        use maliput_sys::api::ffi::SRange_new;

        #[test]
        fn lane_s_range_api() {
            let expected_s_range = SRange_new(1.0, 2.0);
            let expected_lane_id = String::from("0_0_1");
            let lane_s_range = LaneSRange_new(&expected_lane_id, &expected_s_range);
            assert_eq!(LaneSRange_lane_id(&lane_s_range), expected_lane_id);
            assert_eq!(lane_s_range.length(), 1.0);
            let s_range = LaneSRange_s_range(&lane_s_range);
            assert_eq!(s_range.s0(), expected_s_range.s0());
            assert_eq!(s_range.s1(), expected_s_range.s1());
            let lane_s_range_2 = LaneSRange_new(&expected_lane_id, &SRange_new(1.5, 2.5));
            assert!(lane_s_range.Intersects(&lane_s_range_2, 1e-3));
            assert!(!lane_s_range.Contains(&lane_s_range_2, 1e-3));
            let intersection = LaneSRange_GetIntersection(&lane_s_range, &lane_s_range_2, 1e-3);
            assert!(!intersection.is_null());
            let s_range = LaneSRange_s_range(&intersection);
            assert_eq!(s_range.s0(), 1.5);
            assert_eq!(s_range.s1(), 2.0);
        }
    }
}
