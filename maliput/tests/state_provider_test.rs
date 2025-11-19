// BSD 3-Clause License
//
// Copyright (c) 2025, Woven by Toyota.
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

mod common;

#[test]
fn test_phase_provider() {
    let mut road_network = common::create_t_shape_road_network_with_books();
    let phase_ring_book = road_network.phase_ring_book();
    assert_eq!(phase_ring_book.get_phase_rings_ids().len(), 1);
    let phase_ring_id = &phase_ring_book.get_phase_rings_ids()[0];
    let phase_provider = road_network.phase_provider();
    let state_provider = phase_provider.get_phase(phase_ring_id);
    assert!(state_provider.is_some());
    let state_provider = phase_provider.get_phase(&"TIntersectionPhaseRing".to_string());
    assert!(state_provider.is_some());
    let state_provider = state_provider.unwrap();
    assert_eq!(state_provider.state, "AllGo".to_string());
    assert!(state_provider.next.is_some());
    let next_state = state_provider.next.unwrap();
    assert_eq!(next_state.next_state, "AllStop".to_string());
    assert!(next_state.duration_until.is_some());
    let duration_until = next_state.duration_until.unwrap();
    assert_eq!(duration_until, 45.);
}
