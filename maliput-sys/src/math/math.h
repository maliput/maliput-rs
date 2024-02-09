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

#include <maliput/math/vector.h>

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

/// Returns the norm of the vector.
/// Forwards to maliput::math::Vector3::norm() method.
rust::f64 Vector3_norm(const Vector3& self) {
  return self.norm();
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

} // namespace math
}  // namespace maliput
