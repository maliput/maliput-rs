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

#pragma once

#include <memory>
#include <sstream>

#include <maliput/math/vector.h>
#include <maliput/math/matrix.h>
#include <maliput/math/quaternion.h>
#include <maliput/math/roll_pitch_yaw.h>

#include <rust/cxx.h>

namespace maliput {
namespace math {

// Note
// Primitives types are aliased at rust/cxx header file.
// See https://github.com/dtolnay/cxx/blob/131fbc4a14c154a998eb9fc83b9dbb82a521fb1c/include/cxx.h#L455-L465
// @{
// using u8 = std::uint8_t;
// using u16 = std::uint16_t;
// using u32 = std::uint32_t;
// using u64 = std::uint64_t;
// using usize = std::size_t;
// using i8 = std::int8_t;
// using i16 = std::int16_t;
// using i32 = std::int32_t;
// using i64 = std::int64_t;
// using f32 = float;
// using f64 = double;
// }@

/// Creates a new maliput::math::Vector3.
/// Forwads to maliput::math::Vector3(double x, double y, double z) constructor.
std::unique_ptr<Vector3> Vector3_new(rust::f64 x, rust::f64 y, rust::f64 z) {
  return std::make_unique<Vector3>(x, y, z);
}

/// Returns the dot product of two vectors.
/// Forwards to maliput::math::Vector3::dot(const Vector3& other) method.
rust::f64 Vector3_dot(const Vector3& self, const Vector3& other) {
  return self.dot(other);
}

/// Returns the cross product of two vectors.
/// Forwards to maliput::math::Vector3::cross(const Vector3& other) method.
std::unique_ptr<Vector3> Vector3_cross(const Vector3& self, const Vector3& other) {
  return std::make_unique<Vector3>(self.cross(other));
}

/// Returns true if the two vectors are equal.
/// Forwards to maliput::math::Vector3::operator==(const Vector3& other) method.
bool Vector3_equals(const Vector3& self, const Vector3& other) {
  return self == other;
}

/// Returns the string representation of the vector.
/// Forwards to maliput::math::Vector3::to_str() method.
rust::String Vector3_to_str(const Vector3& self) {
  return {self.to_str()};
}

/// Creates a new maliput::math::Vector4.
/// Forwads to maliput::math::Vector4(double x, double y, double z, double w) constructor.
std::unique_ptr<Vector4> Vector4_new(rust::f64 x, rust::f64 y, rust::f64 z, rust::f64 w) {
  return std::make_unique<Vector4>(x, y, z, w);
}

rust::f64 Vector4_dot(const Vector4& self, const Vector4& other) {
  return self.dot(other);
}

bool Vector4_equals(const Vector4& self, const Vector4& other) {
  return self == other;
}

rust::String Vector4_to_str(const Vector4& self) {
  return {self.to_str()};
}

std::unique_ptr<Matrix3> Matrix3_new(double m00, double m01, double m02,
                                     double m10, double m11, double m12,
                                     double m20, double m21, double m22) {
  return std::make_unique<Matrix3>(std::initializer_list<double>{m00, m01, m02, m10, m11, m12, m20, m21, m22});
}

std::unique_ptr<Vector3> Matrix3_row(const Matrix3& self, rust::usize index) {
  return std::make_unique<Vector3>(self.row(index));
}

std::unique_ptr<Vector3> Matrix3_col(const Matrix3& self, rust::usize index) {
  return std::make_unique<Vector3>(self.col(index));
}

std::unique_ptr<Matrix3> Matrix3_cofactor_matrix(const Matrix3& self) {
  return std::make_unique<Matrix3>(self.cofactor());
}

std::unique_ptr<Matrix3> Matrix3_transpose(const Matrix3& self) {
  return std::make_unique<Matrix3>(self.transpose());
}

std::unique_ptr<Matrix3> Matrix3_adjoint(const Matrix3& self) {
  return std::make_unique<Matrix3>(self.adjoint());
}

std::unique_ptr<Matrix3> Matrix3_inverse(const Matrix3& self) {
  return std::make_unique<Matrix3>(self.inverse());
}

bool Matrix3_equals(const Matrix3& self, const Matrix3& other) {
  return self == other;
}

std::unique_ptr<Matrix3> Matrix3_operator_mul(const Matrix3& self, const Matrix3& other) {
  return std::make_unique<Matrix3>(self * other);
}

std::unique_ptr<Matrix3> Matrix3_operator_sum(const Matrix3& self, const Matrix3& other) {
  return std::make_unique<Matrix3>(self + other);
}

std::unique_ptr<Matrix3> Matrix3_operator_sub(const Matrix3& self, const Matrix3& other) {
  return std::make_unique<Matrix3>(self - other);
}

std::unique_ptr<Matrix3> Matrix3_operator_divide_by_scalar(const Matrix3& self, rust::f64 scalar) {
  return std::make_unique<Matrix3>(self / scalar);
}

rust::String Matrix3_to_str(const Matrix3& self) {
  return {self.to_str()};
}

std::unique_ptr<Quaternion> Quaternion_new(rust::f64 w, rust::f64 x, rust::f64 y, rust::f64 z) {
  return std::make_unique<Quaternion>(w, x, y, z);
}

std::unique_ptr<Vector3> Quaternion_vec(const Quaternion& self) {
  return std::make_unique<Vector3>(self.vec());
}

std::unique_ptr<Vector4> Quaternion_coeffs(const Quaternion& self) {
  return std::make_unique<Vector4>(self.coeffs());
}

std::unique_ptr<Quaternion> Quaternion_Inverse(const Quaternion& self) {
  return std::make_unique<Quaternion>(self.Inverse());
}

std::unique_ptr<Quaternion> Quaternion_conjugate(const Quaternion& self) {
  return std::make_unique<Quaternion>(self.conjugate());
}

std::unique_ptr<Matrix3> Quaternion_ToRotationMatrix(const Quaternion& self) {
  return std::make_unique<Matrix3>(self.ToRotationMatrix());
}

std::unique_ptr<Vector3> Quaternion_TransformVector(const Quaternion& self, const Vector3& vec) {
  return std::make_unique<Vector3>(self.TransformVector(vec));
}

bool Quaternion_IsApprox(const Quaternion& self, const Quaternion& other, rust::f64 precision) {
  return self.IsApprox(other, precision);
}

bool Quaternion_equals(const Quaternion& self, const Quaternion& other) {
  return self == other;
}

rust::String Quaternion_to_str(const Quaternion& self) {
  std::stringstream ss;
  ss << self;
  return {ss.str()};
}

std::unique_ptr<RollPitchYaw> RollPitchYaw_new(rust::f64 roll, rust::f64 pitch, rust::f64 yaw) {
  return std::make_unique<RollPitchYaw>(roll, pitch, yaw);
}

void RollPitchYaw_set(RollPitchYaw& self, rust::f64 roll, rust::f64 pitch, rust::f64 yaw) {
  self.set(roll, pitch, yaw);
}

void RollPitchYaw_SetFromQuaternion(RollPitchYaw& self, const Quaternion& q) {
  self.SetFromQuaternion(q);
}

std::unique_ptr<Quaternion> RollPitchYaw_ToQuaternion(const RollPitchYaw& self) {
  return std::make_unique<Quaternion>(self.ToQuaternion());
}

std::unique_ptr<Matrix3> RollPitchYaw_ToMatrix(const RollPitchYaw& self) {
  return std::make_unique<Matrix3>(self.ToMatrix());
}

std::unique_ptr<Matrix3> RollPitchYaw_CalcRotationMatrixDt(const RollPitchYaw& self, const Vector3& w) {
  return std::make_unique<Matrix3>(self.CalcRotationMatrixDt(w));
}

} // namespace math
}  // namespace maliput
