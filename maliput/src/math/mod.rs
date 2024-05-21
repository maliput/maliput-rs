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
    pub(crate) v: cxx::UniquePtr<maliput_sys::math::ffi::Vector3>,
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

/// A quaternion.
/// Wrapper around C++ implementation `maliput::math::Quaternion`.
pub struct Quaternion {
    q: cxx::UniquePtr<maliput_sys::math::ffi::Quaternion>,
}

impl Quaternion {
    /// Create a new `Quaternion` with the given `w`, `x`, `y`, and `z` components.
    /// The `w` component is the real part of the quaternion.
    /// The `x`, `y`, and `z` components are the imaginary parts of the quaternion.
    pub fn new(w: f64, x: f64, y: f64, z: f64) -> Quaternion {
        Quaternion {
            q: maliput_sys::math::ffi::Quaternion_new(w, x, y, z),
        }
    }

    /// Get the `w` component of the `Quaternion`.
    pub fn w(&self) -> f64 {
        self.q.w()
    }

    /// Get the `x` component of the `Quaternion`.
    pub fn x(&self) -> f64 {
        self.q.x()
    }
    /// Get the `y` component of the `Quaternion`.
    pub fn y(&self) -> f64 {
        self.q.y()
    }
    /// Get the `z` component of the `Quaternion`.
    pub fn z(&self) -> f64 {
        self.q.z()
    }
    /// Returns this quaternion Vector.
    pub fn vec(&self) -> Vector3 {
        Vector3 {
            v: maliput_sys::math::ffi::Quaternion_vec(&self.q),
        }
    }
    /// Returns this quaternion coefficients.
    pub fn coeffs(&self) -> Vector4 {
        Vector4 {
            v: maliput_sys::math::ffi::Quaternion_coeffs(&self.q),
        }
    }
    /// Get the dot product of the `Quaternion` with another `Quaternion`.
    pub fn dot(&self, other: &Quaternion) -> f64 {
        self.q.dot(&other.q)
    }
    /// Get the angular distance between the `Quaternion` and another `Quaternion`.
    pub fn angular_distance(&self, other: &Quaternion) -> f64 {
        self.q.AngularDistance(&other.q)
    }
    /// Get the norm of the `Quaternion`.
    pub fn norm(&self) -> f64 {
        self.q.norm()
    }
    /// Normalize the `Quaternion`.
    pub fn normalize(&mut self) {
        self.q.as_mut().expect("Unexpected error").normalize();
    }
    /// Get the squared norm of the `Quaternion`.
    pub fn squared_norm(&self) -> f64 {
        self.q.squared_norm()
    }
    /// Get the inverse of the `Quaternion`.
    pub fn inverse(&self) -> Quaternion {
        Quaternion {
            q: maliput_sys::math::ffi::Quaternion_Inverse(&self.q),
        }
    }
    /// Get the conjugate of the `Quaternion`.
    pub fn conjugate(&self) -> Quaternion {
        Quaternion {
            q: maliput_sys::math::ffi::Quaternion_conjugate(&self.q),
        }
    }
    /// Get the rotation matrix representation of the `Quaternion`.
    pub fn to_rotation_matrix(&self) -> Matrix3 {
        let q = maliput_sys::math::ffi::Quaternion_ToRotationMatrix(&self.q);
        Matrix3 { m: q }
    }
    /// Apply the `Quaternion` to a `Vector3`.
    pub fn transform_vector(&self, v: &Vector3) -> Vector3 {
        let q = maliput_sys::math::ffi::Quaternion_TransformVector(&self.q, &v.v);
        Vector3 { v: q }
    }
}

impl PartialEq for Quaternion {
    fn eq(&self, other: &Self) -> bool {
        maliput_sys::math::ffi::Quaternion_equals(&self.q, &other.q)
    }
}

impl Eq for Quaternion {}

impl std::fmt::Display for Quaternion {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{}", maliput_sys::math::ffi::Quaternion_to_str(&self.q))
    }
}
impl std::fmt::Debug for Quaternion {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        f.debug_struct("Quaternion")
            .field("w", &self.w())
            .field("x", &self.x())
            .field("y", &self.y())
            .field("z", &self.z())
            .finish()
    }
}

/// This class represents the orientation between two arbitrary frames A and D
/// associated with a Space-fixed (extrinsic) X-Y-Z rotation by "roll-pitch-yaw"
/// angles `[r, p, y]`, which is equivalent to a Body-fixed (intrinsic) Z-Y-X
/// rotation by "yaw-pitch-roll" angles `[y, p, r]`.  The rotation matrix `R_AD`
/// associated with this roll-pitch-yaw `[r, p, y]` rotation sequence is equal
/// to the matrix multiplication shown below.
/// ```md, no_run
///        ⎡cos(y) -sin(y)  0⎤   ⎡ cos(p)  0  sin(p)⎤   ⎡1      0        0 ⎤
/// R_AD = ⎢sin(y)  cos(y)  0⎥ * ⎢     0   1      0 ⎥ * ⎢0  cos(r)  -sin(r)⎥
///        ⎣    0       0   1⎦   ⎣-sin(p)  0  cos(p)⎦   ⎣0  sin(r)   cos(r)⎦
///      =       R_AB          *        R_BC          *        R_CD
/// ```
///
/// Wrapper of C++ implementation `maliput::math::RollPitchYaw`.
pub struct RollPitchYaw {
    rpy: cxx::UniquePtr<maliput_sys::math::ffi::RollPitchYaw>,
}

impl RollPitchYaw {
    /// Create a new `RollPitchYaw` with the given `roll`, `pitch`, and `yaw` angles.
    pub fn new(roll: f64, pitch: f64, yaw: f64) -> RollPitchYaw {
        RollPitchYaw {
            rpy: maliput_sys::math::ffi::RollPitchYaw_new(roll, pitch, yaw),
        }
    }
    /// Get the roll angle of the `RollPitchYaw`.
    pub fn roll_angle(&self) -> f64 {
        self.rpy.roll_angle()
    }
    /// Get the pitch angle of the `RollPitchYaw`.
    pub fn pitch_angle(&self) -> f64 {
        self.rpy.pitch_angle()
    }
    /// Get the yaw angle of the `RollPitchYaw`.
    pub fn yaw_angle(&self) -> f64 {
        self.rpy.yaw_angle()
    }
    /// Get the vector of the `RollPitchYaw`.
    pub fn vector(&self) -> Vector3 {
        Vector3::new(self.rpy.vector().x(), self.rpy.vector().y(), self.rpy.vector().z())
    }
    /// Set the `RollPitchYaw` with the given `roll`, `pitch`, and `yaw` angles.
    pub fn set(&mut self, roll: f64, pitch: f64, yaw: f64) {
        maliput_sys::math::ffi::RollPitchYaw_set(self.rpy.as_mut().expect("Unexpected Error"), roll, pitch, yaw);
    }
    /// Set the `RollPitchYaw` from the given `Quaternion`.
    pub fn set_from_quaternion(&mut self, q: &Quaternion) {
        maliput_sys::math::ffi::RollPitchYaw_SetFromQuaternion(self.rpy.as_mut().expect("Unexpected Error"), &q.q);
    }
    /// Get the `Quaternion` representation of the `RollPitchYaw`.
    pub fn to_quaternion(&self) -> Quaternion {
        Quaternion {
            q: maliput_sys::math::ffi::RollPitchYaw_ToQuaternion(&self.rpy),
        }
    }
    /// Get the rotation matrix representation of the `RollPitchYaw`.
    pub fn to_matrix(&self) -> Matrix3 {
        Matrix3 {
            m: maliput_sys::math::ffi::RollPitchYaw_ToMatrix(&self.rpy),
        }
    }
    /// Get the rotation matrix representation of the `RollPitchYaw` with the given `rpy_dt`.
    /// ## Description
    /// Forms Ṙ, the ordinary derivative of the RotationMatrix `R` with respect
    /// to an independent variable `t` (`t` usually denotes time) and `R` is the
    /// RotationMatrix formed by `this` `RollPitchYaw`.  The roll-pitch-yaw angles
    /// r, p, y are regarded as functions of `t` [i.e., r(t), p(t), y(t)].
    /// The param `rpy_dt` Ordinary derivative of rpy with respect to an independent
    /// variable `t` (`t` usually denotes time, but not necessarily).
    /// Returns `Ṙ``, the ordinary derivative of `R` with respect to `t`, calculated
    /// as `Ṙ = ∂R/∂r * ṙ + ∂R/∂p * ṗ + ∂R/∂y * ẏ`.  In other words, the returned
    /// (i, j) element is `∂Rij/∂r * ṙ + ∂Rij/∂p * ṗ + ∂Rij/∂y * ẏ``.
    ///
    /// Note: Docs are taken from the C++ implementation.
    /// ## Args
    /// - `rpy_dt` Ordinary derivative of rpy with respect to an independent variable `t`.
    /// ## Returns
    /// Ordinary derivative of the RotationMatrix `R` with respect to an independent variable `t`.
    pub fn calc_rotation_matrix_dt(&self, rpy_dt: &Vector3) -> Matrix3 {
        Matrix3 {
            m: maliput_sys::math::ffi::RollPitchYaw_CalcRotationMatrixDt(&self.rpy, &rpy_dt.v),
        }
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

    #[test]
    fn quaternion_tests() {
        let q = super::Quaternion::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(q.w(), 1.0);
        assert_eq!(q.x(), 2.0);
        assert_eq!(q.y(), 3.0);
        assert_eq!(q.z(), 4.0);
        // TODO(francocipollone): Add tests for the rest of the API.
    }

    #[test]
    fn roll_pitch_yaw_tests() {
        let rpy = super::RollPitchYaw::new(1.0, 2.0, 3.0);
        assert_eq!(rpy.roll_angle(), 1.0);
        assert_eq!(rpy.pitch_angle(), 2.0);
        assert_eq!(rpy.yaw_angle(), 3.0);
        // TODO(francocipollone): Add tests for the rest of the API.
    }
}
