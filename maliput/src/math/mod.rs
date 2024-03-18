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
/// let v = Vector3::new(1.0, 2.0, 3.0);
/// println!("v = {}", v);
/// assert_eq!(v.x(), 1.0);
/// assert_eq!(v.y(), 2.0);
/// assert_eq!(v.z(), 3.0);
/// assert_eq!(v.norm(), (1.0_f64.powi(2) + 2.0_f64.powi(2) + 3.0_f64.powi(2)).sqrt());
/// assert_eq!(v.dot(&v), 1.0_f64.powi(2) + 2.0_f64.powi(2) + 3.0_f64.powi(2));
/// assert_eq!(v.cross(&v), Vector3::new(0.0, 0.0, 0.0));
/// ```
pub struct Vector3 {
    v: cxx::UniquePtr<maliput_sys::math::ffi::Vector3>,
}

impl Vector3 {
    /// Create a new `Vector3` with the given `x`, `y`, and `z` components.
    pub fn new(x: f64, y: f64, z: f64) -> Vector3 {
        Vector3 {
            v: maliput_sys::math::ffi::Vector3_new(x, y, z),
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
        self.v.norm()
    }
    /// Normalize the `Vector3`.
    pub fn normalize(&mut self) {
        self.v.as_mut().expect("Unexpected error").normalize();
    }
    /// Get the dot product of the `Vector3` with another `Vector3`.
    pub fn dot(&self, w: &Vector3) -> f64 {
        maliput_sys::math::ffi::Vector3_dot(&self.v, &w.v)
    }
    /// Get the cross product of the `Vector3` with another `Vector3`.
    pub fn cross(&self, w: &Vector3) -> Vector3 {
        Vector3 {
            v: maliput_sys::math::ffi::Vector3_cross(&self.v, &w.v),
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

/// A 4D vector.
/// Wrapper around C++ implementation `maliput::math::Vector4`.
///
/// ## Example
///
/// ```rust, no_run
/// use maliput::math::Vector4;
///
/// let v = Vector4::new(1.0, 2.0, 3.0, 4.0);
/// println!("v = {}", v);
/// assert_eq!(v.x(), 1.0);
/// assert_eq!(v.y(), 2.0);
/// assert_eq!(v.z(), 3.0);
/// assert_eq!(v.w(), 4.0);
/// assert_eq!(v.norm(), (1.0_f64.powi(2) + 2.0_f64.powi(2) + 3.0_f64.powi(2) + 4.0_f64.powi(2)).sqrt());
/// ```
pub struct Vector4 {
    v: cxx::UniquePtr<maliput_sys::math::ffi::Vector4>,
}

impl Vector4 {
    /// Create a new `Vector4` with the given `x`, `y`, `z` and `w` components.
    pub fn new(x: f64, y: f64, z: f64, w: f64) -> Vector4 {
        Vector4 {
            v: maliput_sys::math::ffi::Vector4_new(x, y, z, w),
        }
    }
    /// Get the `x` component of the `Vector4`.
    pub fn x(&self) -> f64 {
        self.v.x()
    }
    /// Get the `y` component of the `Vector4`.
    pub fn y(&self) -> f64 {
        self.v.y()
    }
    /// Get the `z` component of the `Vector4`.
    pub fn z(&self) -> f64 {
        self.v.z()
    }
    /// Get the `w` component of the `Vector4`.
    pub fn w(&self) -> f64 {
        self.v.w()
    }
    /// Get the norm of the `Vector4`.
    pub fn norm(&self) -> f64 {
        self.v.norm()
    }
    /// Get the dot product of the `Vector4` with another `Vector4`.
    pub fn dot(&self, w: &Vector4) -> f64 {
        maliput_sys::math::ffi::Vector4_dot(&self.v, &w.v)
    }
    /// Normalize the `Vector4`.
    pub fn normalize(&mut self) {
        self.v.as_mut().expect("Unexpected error").normalize();
    }
}

impl PartialEq for Vector4 {
    fn eq(&self, other: &Self) -> bool {
        maliput_sys::math::ffi::Vector4_equals(&self.v, &other.v)
    }
}

impl Eq for Vector4 {}

impl std::fmt::Display for Vector4 {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{}", maliput_sys::math::ffi::Vector4_to_str(&self.v))
    }
}

impl std::fmt::Debug for Vector4 {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        f.debug_struct("Vector4")
            .field("x", &self.x())
            .field("y", &self.y())
            .field("z", &self.z())
            .field("w", &self.z())
            .finish()
    }
}

/// A 3x3 matrix.
/// Wrapper around C++ implementation `maliput::math::Matrix3`.
///
/// ## Example
///
/// ```rust, no_run
/// use maliput::math::Matrix3;
/// use maliput::math::Vector3;
///
/// let row_1 = Vector3::new(1.0, 2.0, 3.0);
/// let row_2 = Vector3::new(4.0, 5.0, 6.0);
/// let row_3 = Vector3::new(7.0, 8.0, 9.0);
/// let m = Matrix3::new(row_1, row_2, row_3);
/// ```
pub struct Matrix3 {
    m: cxx::UniquePtr<maliput_sys::math::ffi::Matrix3>,
}

impl Matrix3 {
    /// Create a new `Matrix3` with the given `row1`, `row2`, and `row3`.
    pub fn new(row1: Vector3, row2: Vector3, row3: Vector3) -> Matrix3 {
        Matrix3 {
            m: maliput_sys::math::ffi::Matrix3_new(
                row1.x(),
                row1.y(),
                row1.z(),
                row2.x(),
                row2.y(),
                row2.z(),
                row3.x(),
                row3.y(),
                row3.z(),
            ),
        }
    }
    /// Get the cofactor of the `Matrix3` at the given `row` and `col`.
    pub fn cofactor(&self, row: u64, col: u64) -> f64 {
        self.m.cofactor(row, col)
    }
    /// Get the cofactor matrix of the `Matrix3`.
    pub fn cofactor_matrix(&self) -> Matrix3 {
        Matrix3 {
            m: maliput_sys::math::ffi::Matrix3_cofactor_matrix(&self.m),
        }
    }
    /// Get the determinant of the `Matrix3`.
    pub fn determinant(&self) -> f64 {
        self.m.determinant()
    }
    /// Check if the `Matrix3` is singular.
    pub fn is_singular(&self) -> bool {
        self.m.is_singular()
    }
    /// Get the row at the given `index`.
    pub fn row(&self, index: u64) -> Vector3 {
        Vector3 {
            v: maliput_sys::math::ffi::Matrix3_row(&self.m, index),
        }
    }
    /// Get the column at the given `index`.
    pub fn col(&self, index: u64) -> Vector3 {
        Vector3 {
            v: maliput_sys::math::ffi::Matrix3_col(&self.m, index),
        }
    }
    /// Get the transpose of the `Matrix3`.
    pub fn transpose(&self) -> Matrix3 {
        Matrix3 {
            m: maliput_sys::math::ffi::Matrix3_transpose(&self.m),
        }
    }
    /// Get the adjoint of the `Matrix3`.
    pub fn adjoint(&self) -> Matrix3 {
        Matrix3 {
            m: maliput_sys::math::ffi::Matrix3_adjoint(&self.m),
        }
    }
    /// Get the inverse of the `Matrix3`.
    pub fn inverse(&self) -> Matrix3 {
        Matrix3 {
            m: maliput_sys::math::ffi::Matrix3_inverse(&self.m),
        }
    }
}

impl PartialEq for Matrix3 {
    fn eq(&self, other: &Self) -> bool {
        maliput_sys::math::ffi::Matrix3_equals(&self.m, &other.m)
    }
}

impl Eq for Matrix3 {}

impl std::fmt::Display for Matrix3 {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{}", maliput_sys::math::ffi::Matrix3_to_str(&self.m))
    }
}

impl std::fmt::Debug for Matrix3 {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        f.debug_struct("Matrix3")
            .field("row1", &self.row(0))
            .field("row2", &self.row(1))
            .field("row3", &self.row(2))
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
    fn vector3_equality() {
        let v = super::Vector3::new(1.0, 2.0, 3.0);
        let w = super::Vector3::new(1.0, 2.0, 3.0);
        assert_eq!(v, w);
        let z = super::Vector3::new(4.0, 5.0, 6.0);
        assert_ne!(v, z);
    }

    #[test]
    fn vector3_norm() {
        let v = super::Vector3::new(1.0, 2.0, 3.0);
        assert_eq!(v.norm(), (v.x() * v.x() + v.y() * v.y() + v.z() * v.z()).sqrt());
    }

    #[test]
    fn vector3_normalize() {
        let mut v = super::Vector3::new(1.0, 2.0, 3.0);
        let norm = v.norm();
        v.normalize();
        assert_eq!(v.x(), 1.0 / norm);
        assert_eq!(v.y(), 2.0 / norm);
        assert_eq!(v.z(), 3.0 / norm);
    }

    #[test]
    fn vector3_dot() {
        let v = super::Vector3::new(1.0, 2.0, 3.0);
        let w = super::Vector3::new(4.0, 5.0, 6.0);
        assert_eq!(v.dot(&w), v.x() * w.x() + v.y() * w.y() + v.z() * w.z());
    }

    #[test]
    fn vector3_cross() {
        let v = super::Vector3::new(1.0, 2.0, 3.0);
        let w = super::Vector3::new(4.0, 5.0, 6.0);
        let cross = v.cross(&w);
        assert_eq!(cross.x(), v.y() * w.z() - v.z() * w.y());
        assert_eq!(cross.y(), v.z() * w.x() - v.x() * w.z());
        assert_eq!(cross.z(), v.x() * w.y() - v.y() * w.x());
    }

    #[test]
    fn vector4_new() {
        let v = super::Vector4::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(v.x(), 1.0);
        assert_eq!(v.y(), 2.0);
        assert_eq!(v.z(), 3.0);
        assert_eq!(v.w(), 4.0);
    }

    #[test]
    fn vector4_equality() {
        let v = super::Vector4::new(1.0, 2.0, 3.0, 4.0);
        let w = super::Vector4::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(v, w);
        let z = super::Vector4::new(4.0, 5.0, 6.0, 7.0);
        assert_ne!(v, z);
    }

    #[test]
    fn vector4_norm() {
        let v = super::Vector4::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            v.norm(),
            (v.x() * v.x() + v.y() * v.y() + v.z() * v.z() + v.w() * v.w()).sqrt()
        );
    }

    #[test]
    fn vector4_dot() {
        let v = super::Vector4::new(1.0, 2.0, 3.0, 4.0);
        let w = super::Vector4::new(4.0, 5.0, 6.0, 7.0);
        assert_eq!(v.dot(&w), v.x() * w.x() + v.y() * w.y() + v.z() * w.z() + v.w() * w.w());
    }

    #[test]
    fn vector4_normalize() {
        let mut v = super::Vector4::new(1.0, 2.0, 3.0, 4.0);
        let norm = v.norm();
        v.normalize();
        assert_eq!(v.x(), 1.0 / norm);
        assert_eq!(v.y(), 2.0 / norm);
        assert_eq!(v.z(), 3.0 / norm);
        assert_eq!(v.w(), 4.0 / norm);
    }

    #[test]
    fn matrix4_tests() {
        let row_1 = super::Vector3::new(1.0, 2.0, 3.0);
        let row_2 = super::Vector3::new(4.0, 5.0, 6.0);
        let row_3 = super::Vector3::new(7.0, 8.0, 9.0);
        let _m = super::Matrix3::new(row_1, row_2, row_3);
        assert_eq!(_m.row(0).x(), 1.0);
        assert_eq!(_m.col(2).z(), 9.0);
        // TODO(francocipollone): Add tests for the rest of the API.
    }
}
