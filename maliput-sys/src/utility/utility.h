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
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <rust/cxx.h>

#include <string>

#include "maliput/api/road_network.h"
#include "maliput/utility/generate_obj.h"

#include "maliput-sys/src/utility/mod.rs.h"

namespace maliput {
namespace utility {

/// Creates a wrapper around maliput::utility::GenerateObjFile.
void Utility_GenerateObjFile(const maliput::api::RoadNetwork* road_network,
                                 const rust::String& dirpath,
                                 const rust::String& fileroot,
                                 const Features& features) {

  ObjFeatures obj_features{};
  obj_features.max_grid_unit = features.max_grid_unit;
  obj_features.min_grid_resolution = features.min_grid_resolution;
  obj_features.draw_stripes = features.draw_stripes;
  obj_features.draw_arrows = features.draw_arrows;
  obj_features.draw_lane_haze = features.draw_lane_haze;
  obj_features.draw_branch_points = features.draw_branch_points;
  obj_features.draw_elevation_bounds = features.draw_elevation_bounds;
  obj_features.off_grid_mesh_generation = features.off_grid_mesh_generation;
  obj_features.simplify_mesh_threshold = features.simplify_mesh_threshold;
  obj_features.stripe_width = features.stripe_width;
  obj_features.stripe_elevation = features.stripe_elevation;
  obj_features.arrow_elevation = features.arrow_elevation;
  obj_features.lane_haze_elevation = features.lane_haze_elevation;
  obj_features.branch_point_elevation = features.branch_point_elevation;
  obj_features.branch_point_height = features.branch_point_height;
  obj_features.origin = api::InertialPosition{features.origin[0], features.origin[1], features.origin[2]};

  maliput::utility::GenerateObjFile(road_network, std::string(dirpath),
                                    std::string(fileroot), obj_features);
}


}  // namespace utility
}  // namespace maliput
