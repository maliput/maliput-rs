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

#[cxx::bridge(namespace = "maliput::math")]
pub mod ffi {
    unsafe extern "C++" {
        include!("math/math.h");

        #[namespace = "maliput::math"]
        type Vector3;
        fn Vector3_new(x: f64, y: f64, z: f64) -> UniquePtr<Vector3>;
        fn x(self: &Vector3) -> f64;
        fn y(self: &Vector3) -> f64;
        fn z(self: &Vector3) -> f64;
        fn norm(self: &Vector3) -> f64;
        fn normalize(self: Pin<&mut Vector3>);
        fn Vector3_dot(v: &Vector3, w: &Vector3) -> f64;
        fn Vector3_cross(v: &Vector3, w: &Vector3) -> UniquePtr<Vector3>;
        fn Vector3_equals(v: &Vector3, w: &Vector3) -> bool;
        fn Vector3_to_str(v: &Vector3) -> String;

        type Vector4;
        fn Vector4_new(x: f64, y: f64, z: f64, w: f64) -> UniquePtr<Vector4>;
        fn x(self: &Vector4) -> f64;
        fn y(self: &Vector4) -> f64;
        fn z(self: &Vector4) -> f64;
        fn w(self: &Vector4) -> f64;
        fn norm(self: &Vector4) -> f64;
        fn normalize(self: Pin<&mut Vector4>);
        fn Vector4_dot(v: &Vector4, w: &Vector4) -> f64;
        fn Vector4_equals(v: &Vector4, w: &Vector4) -> bool;
        fn Vector4_to_str(v: &Vector4) -> String;

        type Matrix3;
        #[allow(clippy::too_many_arguments)]
        fn Matrix3_new(
            m00: f64,
            m01: f64,
            m02: f64,
            m10: f64,
            m11: f64,
            m12: f64,
            m20: f64,
            m21: f64,
            m22: f64,
        ) -> UniquePtr<Matrix3>;
        fn cofactor(self: &Matrix3, row: u64, col: u64) -> f64;
        fn determinant(self: &Matrix3) -> f64;
        fn is_singular(self: &Matrix3) -> bool;
        fn Matrix3_row(m: &Matrix3, index: u64) -> UniquePtr<Vector3>;
        fn Matrix3_col(m: &Matrix3, index: u64) -> UniquePtr<Vector3>;
        fn Matrix3_cofactor_matrix(m: &Matrix3) -> UniquePtr<Matrix3>;
        fn Matrix3_transpose(m: &Matrix3) -> UniquePtr<Matrix3>;
        fn Matrix3_equals(m: &Matrix3, other: &Matrix3) -> bool;
        fn Matrix3_adjoint(m: &Matrix3) -> UniquePtr<Matrix3>;
        fn Matrix3_operator_mul(m: &Matrix3, other: &Matrix3) -> UniquePtr<Matrix3>;
        fn Matrix3_inverse(m: &Matrix3) -> UniquePtr<Matrix3>;
        fn Matrix3_operator_sum(m: &Matrix3, other: &Matrix3) -> UniquePtr<Matrix3>;
        fn Matrix3_operator_sub(m: &Matrix3, other: &Matrix3) -> UniquePtr<Matrix3>;
        fn Matrix3_operator_divide_by_scalar(m: &Matrix3, scalar: f64) -> UniquePtr<Matrix3>;
        fn Matrix3_to_str(m: &Matrix3) -> String;

        type Quaternion;
        fn Quaternion_new(w: f64, x: f64, y: f64, z: f64) -> UniquePtr<Quaternion>;
        fn w(self: &Quaternion) -> f64;
        fn x(self: &Quaternion) -> f64;
        fn y(self: &Quaternion) -> f64;
        fn z(self: &Quaternion) -> f64;
        fn dot(self: &Quaternion, other: &Quaternion) -> f64;
        fn AngularDistance(self: &Quaternion, other: &Quaternion) -> f64;
        fn norm(self: &Quaternion) -> f64;
        fn normalize(self: Pin<&mut Quaternion>);
        fn squared_norm(self: &Quaternion) -> f64;
        fn Quaternion_vec(q: &Quaternion) -> UniquePtr<Vector3>;
        fn Quaternion_coeffs(q: &Quaternion) -> UniquePtr<Vector4>;
        fn Quaternion_Inverse(q: &Quaternion) -> UniquePtr<Quaternion>;
        fn Quaternion_conjugate(q: &Quaternion) -> UniquePtr<Quaternion>;
        fn Quaternion_ToRotationMatrix(q: &Quaternion) -> UniquePtr<Matrix3>;
        fn Quaternion_TransformVector(q: &Quaternion, v: &Vector3) -> UniquePtr<Vector3>;
        fn Quaternion_IsApprox(q: &Quaternion, other: &Quaternion, precision: f64) -> bool;
        fn Quaternion_equals(q: &Quaternion, other: &Quaternion) -> bool;
        fn Quaternion_to_str(q: &Quaternion) -> String;

        type RollPitchYaw;
        fn RollPitchYaw_new(roll: f64, pitch: f64, yaw: f64) -> UniquePtr<RollPitchYaw>;
        fn roll_angle(self: &RollPitchYaw) -> f64;
        fn pitch_angle(self: &RollPitchYaw) -> f64;
        fn yaw_angle(self: &RollPitchYaw) -> f64;
        fn vector(self: &RollPitchYaw) -> &Vector3;
        fn RollPitchYaw_set(rpy: Pin<&mut RollPitchYaw>, roll: f64, pitch: f64, yaw: f64);
        fn RollPitchYaw_SetFromQuaternion(rpy: Pin<&mut RollPitchYaw>, q: &Quaternion);
        fn RollPitchYaw_ToQuaternion(rpy: &RollPitchYaw) -> UniquePtr<Quaternion>;
        fn RollPitchYaw_ToMatrix(rpy: &RollPitchYaw) -> UniquePtr<Matrix3>;
        fn RollPitchYaw_CalcRotationMatrixDt(rpy: &RollPitchYaw, rpyDt: &Vector3) -> UniquePtr<Matrix3>;
    }
}
