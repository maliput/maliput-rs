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

#include "maliput/api/road_network.h"
#include "maliput/plugin/create_road_network.h"

#include <map>
#include <string>

#include <rust/cxx.h>

namespace maliput {
namespace plugin {

/// Creates a wrapper around maliput::plugin::CreateRoadNetwork.
/// cxx crate can't handle std::map, so we use std::vector instead.
/// The properties vector is a list of key-value pairs, where the key and value are strings.
/// The key and value are separated by a colon, e.g. "key:value".
std::unique_ptr<maliput::api::RoadNetwork> CreateRoadNetwork(const rust::String& road_network_loader_id,
                                                             const rust::Vec<rust::String>& properties) {
  std::map<std::string, std::string> map_properties;
    for (const auto& property : properties) {
        const std::string prop = std::string(property);
        const auto colon_index = prop.find(':');
        if (colon_index == std::string::npos) {
            throw std::runtime_error("maliput-sys: invalid property: " + prop);
        }
        const auto key = prop.substr(0, colon_index);
        const auto value = prop.substr(colon_index + 1);
        map_properties[key] = value;
    }
   std::unique_ptr<maliput::api::RoadNetwork> rn = maliput::plugin::CreateRoadNetwork(std::string(road_network_loader_id), map_properties);
  return rn;
}

} // namespace plugin
}  // namespace maliput
