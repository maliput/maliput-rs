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
mod math_test {
    use maliput_sys::math::ffi::Vector3_cross;
    use maliput_sys::math::ffi::Vector3_dot;
    use maliput_sys::math::ffi::Vector3_equals;
    use maliput_sys::math::ffi::Vector3_new;

    use maliput_sys::math::ffi::Vector4_dot;
    use maliput_sys::math::ffi::Vector4_equals;
    use maliput_sys::math::ffi::Vector4_new;

    use maliput_sys::math::ffi::Matrix3_adjoint;
    use maliput_sys::math::ffi::Matrix3_cofactor_matrix;
    use maliput_sys::math::ffi::Matrix3_col;
    use maliput_sys::math::ffi::Matrix3_equals;
    use maliput_sys::math::ffi::Matrix3_inverse;
    use maliput_sys::math::ffi::Matrix3_new;
    use maliput_sys::math::ffi::Matrix3_operator_divide_by_scalar;
    use maliput_sys::math::ffi::Matrix3_operator_mul;
    use maliput_sys::math::ffi::Matrix3_operator_sub;
    use maliput_sys::math::ffi::Matrix3_operator_sum;
    use maliput_sys::math::ffi::Matrix3_row;
    use maliput_sys::math::ffi::Matrix3_to_str;
    use maliput_sys::math::ffi::Matrix3_transpose;

    use maliput_sys::math::ffi::Quaternion_Inverse;
    use maliput_sys::math::ffi::Quaternion_IsApprox;
    use maliput_sys::math::ffi::Quaternion_ToRotationMatrix;
    use maliput_sys::math::ffi::Quaternion_TransformVector;
    use maliput_sys::math::ffi::Quaternion_coeffs;
    use maliput_sys::math::ffi::Quaternion_conjugate;
    use maliput_sys::math::ffi::Quaternion_equals;
    use maliput_sys::math::ffi::Quaternion_new;
    use maliput_sys::math::ffi::Quaternion_to_str;
    use maliput_sys::math::ffi::Quaternion_vec;

    #[test]
    fn vector3_new() {
        let v = Vector3_new(1.0, 2.0, 3.0);
        assert_eq!(v.x(), 1.0);
        assert_eq!(v.y(), 2.0);
        assert_eq!(v.z(), 3.0);
    }

    #[test]
    fn vector3_norm() {
        let v = Vector3_new(1.0, 2.0, 3.0);
        assert_eq!(v.norm(), (v.x() * v.x() + v.y() * v.y() + v.z() * v.z()).sqrt());
    }

    #[test]
    fn vector3_normalize() {
        let mut v = Vector3_new(1.0, 2.0, 3.0);
        let norm = v.norm();
        v.as_mut().expect("").normalize();
        assert_eq!(v.x(), 1.0 / norm);
        assert_eq!(v.y(), 2.0 / norm);
        assert_eq!(v.z(), 3.0 / norm);
    }

    #[test]
    fn vector3_dot() {
        let v = Vector3_new(1.0, 2.0, 3.0);
        let w = Vector3_new(4.0, 5.0, 6.0);
        assert_eq!(Vector3_dot(&v, &w), v.x() * w.x() + v.y() * w.y() + v.z() * w.z());
    }

    #[test]
    fn vector3_cross() {
        let v = Vector3_new(1.0, 2.0, 3.0);
        let w = Vector3_new(4.0, 5.0, 6.0);
        let cross = Vector3_cross(&v, &w);
        assert_eq!(cross.x(), v.y() * w.z() - v.z() * w.y());
        assert_eq!(cross.y(), v.z() * w.x() - v.x() * w.z());
        assert_eq!(cross.z(), v.x() * w.y() - v.y() * w.x());
    }

    #[test]
    fn vector3_equals() {
        let v = Vector3_new(1.0, 2.0, 3.0);
        let w = Vector3_new(1.0, 2.0, 3.0);
        assert!(Vector3_equals(&v, &w));
    }

    #[test]
    fn vector4_new() {
        let v = Vector4_new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(v.x(), 1.0);
        assert_eq!(v.y(), 2.0);
        assert_eq!(v.z(), 3.0);
        assert_eq!(v.w(), 4.0);
    }

    #[test]
    fn vector4_norm() {
        let v = Vector4_new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            v.norm(),
            (v.x() * v.x() + v.y() * v.y() + v.z() * v.z() + v.w() * v.w()).sqrt()
        );
    }

    #[test]
    fn vector4_normalize() {
        let mut v = Vector4_new(1.0, 2.0, 3.0, 4.0);
        let norm = v.norm();
        v.as_mut().expect("").normalize();
        assert_eq!(v.x(), 1.0 / norm);
        assert_eq!(v.y(), 2.0 / norm);
        assert_eq!(v.z(), 3.0 / norm);
        assert_eq!(v.w(), 4.0 / norm);
    }

    #[test]
    fn vector4_dot() {
        let v = Vector4_new(1.0, 2.0, 3.0, 4.0);
        let w = Vector4_new(5.0, 6.0, 7.0, 8.0);
        assert_eq!(
            Vector4_dot(&v, &w),
            v.x() * w.x() + v.y() * w.y() + v.z() * w.z() + v.w() * w.w()
        );
    }

    #[test]
    fn vector4_equals() {
        let v = Vector4_new(1.0, 2.0, 3.0, 4.0);
        let w = Vector4_new(1.0, 2.0, 3.0, 4.0);
        assert!(Vector4_equals(&v, &w));
    }

    #[test]
    fn matrix3_new() {
        let m = Matrix3_new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let row1 = Matrix3_row(&m, 0);
        let row2 = Matrix3_row(&m, 1);
        let row3 = Matrix3_row(&m, 2);
        assert_eq!(row1.x(), 1.0);
        assert_eq!(row1.y(), 2.0);
        assert_eq!(row1.z(), 3.0);
        assert_eq!(row2.x(), 4.0);
        assert_eq!(row2.y(), 5.0);
        assert_eq!(row2.z(), 6.0);
        assert_eq!(row3.x(), 7.0);
        assert_eq!(row3.y(), 8.0);
        assert_eq!(row3.z(), 9.0);
    }

    #[test]
    fn matrix3_col() {
        let m = Matrix3_new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let col1 = Matrix3_col(&m, 0);
        let col2 = Matrix3_col(&m, 1);
        let col3 = Matrix3_col(&m, 2);
        assert_eq!(col1.x(), 1.0);
        assert_eq!(col1.y(), 4.0);
        assert_eq!(col1.z(), 7.0);
        assert_eq!(col2.x(), 2.0);
        assert_eq!(col2.y(), 5.0);
        assert_eq!(col2.z(), 8.0);
        assert_eq!(col3.x(), 3.0);
        assert_eq!(col3.y(), 6.0);
        assert_eq!(col3.z(), 9.0);
    }

    #[test]
    fn matrix3_cofactor() {
        let m = Matrix3_new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        assert_eq!(m.cofactor(0, 0), -3.0);
        assert_eq!(m.cofactor(0, 1), 6.0);
        assert_eq!(m.cofactor(0, 2), -3.0);
        assert_eq!(m.cofactor(1, 0), 6.0);
        assert_eq!(m.cofactor(1, 1), -12.0);
        assert_eq!(m.cofactor(1, 2), 6.0);
        assert_eq!(m.cofactor(2, 0), -3.0);
        assert_eq!(m.cofactor(2, 1), 6.0);
        assert_eq!(m.cofactor(2, 2), -3.0);
    }

    #[test]
    fn matrix3_cofactor_matrix() {
        let m = Matrix3_new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let cofactor_matrix = Matrix3_cofactor_matrix(&m);
        assert_eq!(Matrix3_row(&cofactor_matrix, 0).x(), m.cofactor(0, 0));
    }

    #[test]
    fn matrix3_determinant() {
        let m = Matrix3_new(6.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        assert_eq!(m.determinant(), -15.);
    }

    #[test]
    fn matrix3_is_singular() {
        let m = Matrix3_new(6.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        assert_eq!(m.is_singular(), m.determinant() == 0.0);
    }

    #[test]
    fn matrix3_transpose() {
        let m = Matrix3_new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let t = Matrix3_transpose(&m);
        assert_eq!(Matrix3_row(&m, 1).z(), Matrix3_row(&t, 2).y());
    }

    #[test]
    fn matrix3_equals() {
        let m = Matrix3_new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let n = Matrix3_new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        assert!(Matrix3_equals(&m, &n));
    }

    #[test]
    fn matrix3_adjoint() {
        let m = Matrix3_new(6.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let adj = Matrix3_adjoint(&m);
        assert!(Matrix3_equals(&Matrix3_transpose(&Matrix3_cofactor_matrix(&m)), &adj));
    }

    #[test]
    fn matrix3_operator_mul() {
        let m = Matrix3_new(1., 2., 3., 4., 5., 6., 7., 8., 9.);
        let n = Matrix3_new(9., 8., 7., 6., 5., 4., 3., 2., 1.);
        let p = Matrix3_new(30., 24., 18., 84., 69., 54., 138., 114., 90.);
        assert!(Matrix3_equals(&p, &Matrix3_operator_mul(&m, &n)));
    }

    #[test]
    fn matrix3_inverse() {
        let k_tolerance = 1e-10;
        let m = Matrix3_new(1., 12., 3., 8., 5., 3., 6., 14., 9.);
        let inv = Matrix3_inverse(&m);
        let identity = Matrix3_new(1., 0., 0., 0., 1., 0., 0., 0., 1.);
        let product = Matrix3_operator_mul(&m, &inv);
        assert!((Matrix3_row(&product, 0).x() - Matrix3_row(&identity, 0).x()).abs() < k_tolerance);
        assert!((Matrix3_row(&product, 0).y() - Matrix3_row(&identity, 0).y()).abs() < k_tolerance);
        assert!((Matrix3_row(&product, 0).z() - Matrix3_row(&identity, 0).z()).abs() < k_tolerance);
        assert!((Matrix3_row(&product, 1).x() - Matrix3_row(&identity, 1).x()).abs() < k_tolerance);
        assert!((Matrix3_row(&product, 1).y() - Matrix3_row(&identity, 1).y()).abs() < k_tolerance);
        assert!((Matrix3_row(&product, 1).z() - Matrix3_row(&identity, 1).z()).abs() < k_tolerance);
        assert!((Matrix3_row(&product, 2).x() - Matrix3_row(&identity, 2).x()).abs() < k_tolerance);
        assert!((Matrix3_row(&product, 2).y() - Matrix3_row(&identity, 2).y()).abs() < k_tolerance);
        assert!((Matrix3_row(&product, 2).z() - Matrix3_row(&identity, 2).z()).abs() < k_tolerance);
    }

    #[test]
    fn matrix3_operator_sum() {
        let m = Matrix3_new(1., 2., 3., 4., 5., 6., 7., 8., 9.);
        let n = Matrix3_new(9., 8., 7., 6., 5., 4., 3., 2., 1.);
        let p = Matrix3_new(10., 10., 10., 10., 10., 10., 10., 10., 10.);
        assert!(Matrix3_equals(&p, &Matrix3_operator_sum(&m, &n)));
    }

    #[test]
    fn matrix3_operator_sub() {
        let m = Matrix3_new(1., 2., 3., 4., 5., 6., 7., 8., 9.);
        let n = Matrix3_new(9., 8., 7., 6., 5., 4., 3., 2., 1.);
        let p = Matrix3_new(-8., -6., -4., -2., 0., 2., 4., 6., 8.);
        assert!(Matrix3_equals(&p, &Matrix3_operator_sub(&m, &n)));
    }

    #[test]
    fn matrix3_operator_divide_by_scalar() {
        let m = Matrix3_new(1., 2., 3., 4., 5., 6., 7., 8., 9.);
        let n = 2.0;
        let p = Matrix3_new(0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5);
        assert!(Matrix3_equals(&p, &Matrix3_operator_divide_by_scalar(&m, n)));
    }

    #[test]
    fn matrix3_to_str() {
        let m = Matrix3_new(1., 2., 3., 4., 5., 6., 7., 8., 9.);
        assert_eq!(Matrix3_to_str(&m), "{{1, 2, 3},\n{4, 5, 6},\n{7, 8, 9}}");
    }

    #[test]
    fn quaternion_new() {
        let q = Quaternion_new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(q.w(), 1.0);
        assert_eq!(q.x(), 2.0);
        assert_eq!(q.y(), 3.0);
        assert_eq!(q.z(), 4.0);
    }

    #[test]
    fn quaternion_dot() {
        let q = Quaternion_new(1.0, 2.0, 3.0, 4.0);
        let r = Quaternion_new(5.0, 6.0, 7.0, 8.0);
        assert_eq!(q.dot(&r), q.w() * r.w() + q.x() * r.x() + q.y() * r.y() + q.z() * r.z());
    }

    #[test]
    fn quaternion_angular_distance() {
        let q = Quaternion_new(1.0, 2.0, 3.0, 4.0);
        let r = Quaternion_new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(q.AngularDistance(&r), 0.);
    }

    #[test]
    fn quaternion_norm() {
        let q = Quaternion_new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            q.norm(),
            (q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z()).sqrt()
        );
    }

    #[test]
    fn quaternion_normalize() {
        let mut q = Quaternion_new(1.0, 2.0, 3.0, 4.0);
        let norm = q.norm();
        q.as_mut().expect("").normalize();
        assert_eq!(q.w(), 1.0 / norm);
        assert_eq!(q.x(), 2.0 / norm);
        assert_eq!(q.y(), 3.0 / norm);
        assert_eq!(q.z(), 4.0 / norm);
    }

    #[test]
    fn quaternion_squared_norm() {
        let q = Quaternion_new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            q.squared_norm(),
            q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z()
        );
    }

    #[test]
    fn quaternion_vec() {
        let q = Quaternion_new(1.0, 2.0, 3.0, 4.0);
        let v = Quaternion_vec(&q);
        assert_eq!(v.x(), q.x());
        assert_eq!(v.y(), q.y());
        assert_eq!(v.z(), q.z());
    }

    #[test]
    fn quaternion_coeffs() {
        let q = Quaternion_new(1.0, 2.0, 3.0, 4.0);
        let v = Quaternion_coeffs(&q);
        assert_eq!(v.x(), q.w());
        assert_eq!(v.y(), q.x());
        assert_eq!(v.z(), q.y());
        assert_eq!(v.w(), q.z());
    }

    #[test]
    fn quaternion_inverse() {
        let q = Quaternion_new(1.0, 0.0, 0.0, 0.0);
        let inv = Quaternion_Inverse(&q);
        assert!(Quaternion_equals(&inv, &Quaternion_Inverse(&q)));
    }

    #[test]
    fn quaternion_equals() {
        let q = Quaternion_new(1.0, 2.0, 3.0, 4.0);
        let r = Quaternion_new(1.0, 2.0, 3.0, 4.0);
        assert!(Quaternion_equals(&q, &r));
    }

    #[test]
    fn quaternion_conjugate() {
        let q = Quaternion_new(1.0, 2.0, 3.0, 4.0);
        let conj = Quaternion_conjugate(&q);
        assert_eq!(conj.w(), q.w());
        assert_eq!(conj.x(), -q.x());
        assert_eq!(conj.y(), -q.y());
        assert_eq!(conj.z(), -q.z());
    }

    #[test]
    fn quaternion_to_rotation_matrix() {
        let q = Quaternion_new(1.0, 0.0, 0.0, 0.0);
        let r = Quaternion_ToRotationMatrix(&q);
        assert_eq!(Matrix3_row(&r, 0).x(), 1.0);
        assert_eq!(Matrix3_row(&r, 1).y(), 1.0);
        assert_eq!(Matrix3_row(&r, 2).z(), 1.0);
    }

    #[test]
    fn quaternion_transform_vector() {
        let q = Quaternion_new(1.0, 0.0, 0.0, 0.0);
        let v = Vector3_new(1.0, 2.0, 3.0);
        let w = Quaternion_TransformVector(&q, &v);
        assert!(Vector3_equals(&w, &v));
    }

    #[test]
    fn quaternion_is_approx() {
        let q = Quaternion_new(1.0, 2.0, 3.0, 4.0);
        let r = Quaternion_new(1.0, 2.0, 3.0, 4.0);
        assert!(Quaternion_IsApprox(&q, &r, 1e-10));
    }

    #[test]
    fn quaternion_to_str() {
        let q = Quaternion_new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(Quaternion_to_str(&q), "(w: 1, x: 2, y: 3, z: 4)");
    }
}
