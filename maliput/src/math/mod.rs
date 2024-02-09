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

/// A 3D vector.
/// Wrapper around C++ implementation `maliput::math::Vector3`.
///
/// ## Example
///
/// ```rust, no_run
/// use maliput::math::Vector3;
///
/// fn main() {
///     let v = Vector3::new(1.0, 2.0, 3.0);
///     println!("v = {}", v);
///     assert_eq!(v.x(), 1.0);
///     assert_eq!(v.y(), 2.0);
///     assert_eq!(v.z(), 3.0);
///     assert_eq!(v.norm(), (1.0_f64.powi(2) + 2.0_f64.powi(2) + 3.0_f64.powi(2)).sqrt());
///     assert_eq!(v.dot(&v), 1.0_f64.powi(2) + 2.0_f64.powi(2) + 3.0_f64.powi(2));
///     assert_eq!(v.cross(&v), Vector3::new(0.0, 0.0, 0.0));
/// }
/// ```
pub struct Vector3 {
    v: cxx::UniquePtr<maliput_sys::math::ffi::Vector3>,
}

impl Vector3 {
    /// Create a new `Vector3` with the given `x`, `y`, and `z` components.
    pub fn new(x: f64, y: f64, z: f64) -> Vector3 {
        Vector3 {
            v: maliput_sys::math::ffi::Vector3_new(x, y, z)
        }
    }
    /// Get the `x` component of the `Vector3`.
    pub fn x(&self) -> f64 {
        self.v.x()
    }
    /// Get the `y` component of the `Vector3`.
    pub fn y(&self) -> f64 {
        self.v.y()
    }
    /// Get the `z` component of the `Vector3`.
    pub fn z(&self) -> f64 {
        self.v.z()
    }
    /// Get the norm of the `Vector3`.
    pub fn norm(&self) -> f64 {
        maliput_sys::math::ffi::Vector3_norm(&self.v)
    }
    /// Get the dot product of the `Vector3` with another `Vector3`.
    pub fn dot(&self, w: &Vector3) -> f64 {
        maliput_sys::math::ffi::Vector3_dot(&self.v, &w.v)
    }
    /// Get the cross product of the `Vector3` with another `Vector3`.
    pub fn cross(&self, w: &Vector3) -> Vector3 {
        Vector3 {
            v: maliput_sys::math::ffi::Vector3_cross(&self.v, &w.v)
        }
    }
}

impl PartialEq for Vector3 {
    fn eq(&self, other: &Self) -> bool {
        maliput_sys::math::ffi::Vector3_equals(&self.v, &other.v)
    }

}

impl Eq for Vector3 {}

impl std::fmt::Display for Vector3 {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{}", maliput_sys::math::ffi::Vector3_to_str(&self.v))
    }
}

impl std::fmt::Debug for Vector3 {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        f.debug_struct("Vector3")
            .field("x", &self.x())
            .field("y", &self.y())
            .field("z", &self.z())
            .finish()
    }
}

mod tests {
    #[test]
    fn vector3_new() {
        let v = super::Vector3::new(1.0, 2.0, 3.0);
        assert_eq!(v.x(), 1.0);
        assert_eq!(v.y(), 2.0);
        assert_eq!(v.z(), 3.0);
    }

    #[test]
    fn equality() {
        let v = super::Vector3::new(1.0, 2.0, 3.0);
        let w = super::Vector3::new(1.0, 2.0, 3.0);
        assert_eq!(v, w);
        let z = super::Vector3::new(4.0, 5.0, 6.0);
        assert_ne!(v, z);
    }

    #[test]
    fn vector3_norm() {
        let v = super::Vector3::new(1.0, 2.0, 3.0);
        assert_eq!(v.norm(), (v.x()*v.x() + v.y()*v.y() + v.z()*v.z()).sqrt());
    }

    #[test]
    fn vector3_dot() {
        let v = super::Vector3::new(1.0, 2.0, 3.0);
        let w = super::Vector3::new(4.0, 5.0, 6.0);
        assert_eq!(v.dot(&w), v.x()*w.x() + v.y()*w.y() + v.z()*w.z());
    }

    #[test]
    fn vector3_cross() {
        let v = super::Vector3::new(1.0, 2.0, 3.0);
        let w = super::Vector3::new(4.0, 5.0, 6.0);
        let cross = v.cross(&w);
        assert_eq!(cross.x(), v.y()*w.z() - v.z()*w.y());
        assert_eq!(cross.y(), v.z()*w.x() - v.x()*w.z());
        assert_eq!(cross.z(), v.x()*w.y() - v.y()*w.x());
    }
}