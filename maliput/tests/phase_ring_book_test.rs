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
fn phase_ring_book_test() {
    let road_network = common::create_loop_road_pedestrian_crosswalk_road_network_with_books();

    let phase_ring_book = road_network.phase_ring_book();
    let phase_ring_ids = phase_ring_book.get_phase_rings_ids();
    assert_eq!(phase_ring_ids.len(), 2);

    let phase_ring_id = "PedestrianCrosswalkIntersectionSouth".to_string();
    assert!(phase_ring_ids.contains(&phase_ring_id));
    let phase_ring = phase_ring_book.get_phase_ring(&phase_ring_id);
    assert!(phase_ring.is_some());
    let phase_ring = phase_ring.unwrap();
    assert_eq!(phase_ring.id(), phase_ring_id);

    let phase_ids = phase_ring.phases();
    assert_eq!(phase_ids.len(), 2);
    assert!(phase_ids.contains(&"AllGoPhase".to_string()));
    assert!(phase_ids.contains(&"AllStopPhase".to_string()));

    let phase = phase_ring.get_phase(&"AllGoPhase".to_string());
    assert!(phase.is_some());
    let phase = phase.unwrap();
    assert_eq!(phase.id(), "AllGoPhase".to_string());
}
