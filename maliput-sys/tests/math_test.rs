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
    use maliput_sys::math::ffi::Vector3_new;

    use maliput_sys::math::ffi::Vector3_cross;
    use maliput_sys::math::ffi::Vector3_dot;
    use maliput_sys::math::ffi::Vector3_equals;
    use maliput_sys::math::ffi::Vector3_norm;

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
        assert_eq!(Vector3_norm(&v), (v.x() * v.x() + v.y() * v.y() + v.z() * v.z()).sqrt());
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
}
