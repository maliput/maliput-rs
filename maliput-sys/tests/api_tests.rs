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
    // use maliput_sys::api::ffi::LanePosition;
    use maliput_sys::api::ffi::LanePosition_new;
    use maliput_sys::api::ffi::LanePosition_srh;

    // use maliput_sys::math::ffi::Vector3;
    use maliput_sys::math::ffi::Vector3_new;
    use maliput_sys::math::ffi::Vector3_norm;

    #[test]
    fn laneposition_new() {
        let lane_pos = LanePosition_new(1.0, 2.0, 3.0);
        assert_eq!(lane_pos.s(), 1.0);
        assert_eq!(lane_pos.r(), 2.0);
        assert_eq!(lane_pos.h(), 3.0);
    }

    #[test]
    fn laneposition_srh() {
        let lane_pos = LanePosition_new(1.0, 2.0, 3.0);
        assert_eq!(Vector3_norm(&LanePosition_srh(&lane_pos)), 3.7416573867739413);
    }

    #[test]
    fn set_srh() {
        let mut lane_pos = LanePosition_new(1.0, 2.0, 3.0);
        let vector = Vector3_new(4.0, 5.0, 6.0);
        lane_pos.as_mut().expect("").set_srh(&vector);
    }
}
