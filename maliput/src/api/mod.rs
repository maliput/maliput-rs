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

/// A Lane Position.
/// Wrapper around C++ implementation `maliput::api::LanePosition`.
///
/// ## Example
///
/// ```rust, no_run
/// use maliput::api::LanePosition;
///
/// let lane_pos = LanePosition::new(1.0, 2.0, 3.0);
/// println!("lane_pos = {}", lane_pos);
/// assert_eq!(lane_pos.s(), 1.0);
/// assert_eq!(lane_pos.r(), 2.0);
/// assert_eq!(lane_pos.h(), 3.0);
/// ```
pub struct LanePosition {
    lp: cxx::UniquePtr<maliput_sys::api::ffi::LanePosition>,
}

use crate::math::Vector3;

impl LanePosition {
    /// Create a new `LanePosition` with the given `s`, `r`, and `h` components.
    pub fn new(s: f64, r: f64, h: f64) -> LanePosition {
        LanePosition {
            lp: maliput_sys::api::ffi::LanePosition_new(s, r, h),
        }
    }
    /// Get the `s` component of the `LanePosition`.
    pub fn s(&self) -> f64 {
        self.lp.s()
    }
    /// Get the `r` component of the `LanePosition`.
    pub fn r(&self) -> f64 {
        self.lp.r()
    }
    /// Get the `h` component of the `LanePosition`.
    pub fn h(&self) -> f64 {
        self.lp.h()
    }

    /// Returns all components as 3-vector `[s, r, h]`.
    pub fn srh(&self) -> Vector3 {
        let srh = self.lp.srh();
        Vector3::new(srh.x(), srh.y(), srh.z())
    }

    /// Set the `s` component of the `LanePosition`.
    pub fn set_s(&mut self, s: f64) {
        self.lp.as_mut().expect("Underlying LanePosition is null").set_s(s);
    }

    /// Set the `r` component of the `LanePosition`.
    pub fn set_r(&mut self, r: f64) {
        self.lp.as_mut().expect("Underlying LanePosition is null").set_r(r);
    }

    /// Set the `h` component of the `LanePosition`.
    pub fn set_h(&mut self, h: f64) {
        self.lp.as_mut().expect("Underlying LanePosition is null").set_h(h);
    }

    /// Set all components from 3-vector `[s, r, h]`.
    pub fn set_srh(&mut self, srh: &Vector3) {
        let ffi_vec = maliput_sys::math::ffi::Vector3_new(srh.x(), srh.y(), srh.z());
        self.lp
            .as_mut()
            .expect("Underlying LanePosition is null")
            .set_srh(&ffi_vec);
    }
}

impl PartialEq for LanePosition {
    fn eq(&self, other: &Self) -> bool {
        self.srh() == other.srh()
    }
}

impl Eq for LanePosition {}

impl std::fmt::Display for LanePosition {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{}", maliput_sys::api::ffi::LanePosition_to_str(&self.lp))
    }
}

impl std::fmt::Debug for LanePosition {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        f.debug_struct("LanePosition")
            .field("s", &self.s())
            .field("r", &self.r())
            .field("h", &self.h())
            .finish()
    }
}

mod tests {
    #[test]
    fn lane_position_new() {
        let lane_pos = super::LanePosition::new(1.0, 2.0, 3.0);
        assert_eq!(lane_pos.s(), 1.0);
        assert_eq!(lane_pos.r(), 2.0);
        assert_eq!(lane_pos.h(), 3.0);
    }

    #[test]
    fn equality() {
        let v = super::LanePosition::new(1.0, 2.0, 3.0);
        let w = super::LanePosition::new(1.0, 2.0, 3.0);
        assert_eq!(v, w);
        let z = super::LanePosition::new(4.0, 5.0, 6.0);
        assert_ne!(v, z);
    }

    #[test]
    fn set_s() {
        let mut lane_pos = super::LanePosition::new(1.0, 2.0, 3.0);
        lane_pos.set_s(4.0);
        assert_eq!(lane_pos.s(), 4.0);
    }

    #[test]
    fn set_r() {
        let mut lane_pos = super::LanePosition::new(1.0, 2.0, 3.0);
        lane_pos.set_r(4.0);
        assert_eq!(lane_pos.r(), 4.0);
    }

    #[test]
    fn set_h() {
        let mut lane_pos = super::LanePosition::new(1.0, 2.0, 3.0);
        lane_pos.set_h(4.0);
        assert_eq!(lane_pos.h(), 4.0);
    }

    #[test]
    fn set_srh() {
        use crate::math::Vector3;
        let mut lane_pos = super::LanePosition::new(1.0, 2.0, 3.0);
        let vector = Vector3::new(4.0, 5.0, 6.0);
        lane_pos.set_srh(&vector);
        assert_eq!(lane_pos.s(), 4.0);
        assert_eq!(lane_pos.r(), 5.0);
        assert_eq!(lane_pos.h(), 6.0);
    }
}
