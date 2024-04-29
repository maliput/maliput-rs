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
#![allow(rustdoc::bare_urls)]
#![doc = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/README.md"))]

pub mod api;
pub mod math;

/// ### ResourceManager
///
/// Convenient method for getting the path to the resources of the backends.
///
/// ### Backends
///  - maliput_malidrive: Resources from maliput_malidrive are brought by maliput-sdk package.
///                       All the maliput_malidrive resources are stored in the same directory:
///                       (See https://github.com/maliput/maliput_malidrive/tree/main/resources)
/// ### Example
///
/// How to get all the resources from the maliput_malidrive backend:
///
/// ```rust, no_run
/// let resource_manager = maliput::ResourceManager::new();
/// let all_resources_in_malidrive = resource_manager.get_all_resources_by_backend("maliput_malidrive");
/// ```
///
/// How to get the path to the TShapeRoad.xodr file from the maliput_malidrive backend:
/// ```rust, no_run
/// let resource_manager = maliput::ResourceManager::new();
/// let t_shape_road_xodr_path = resource_manager.get_resource_path_by_name("maliput_malidrive", "TShapeRoad.xodr");
/// assert!(t_shape_road_xodr_path.is_some());
/// assert!(t_shape_road_xodr_path.unwrap().exists());
/// ```
pub struct ResourceManager {
    resource_paths: std::collections::HashMap<String, Vec<std::path::PathBuf>>,
}

impl Default for ResourceManager {
    fn default() -> Self {
        Self::new()
    }
}

impl ResourceManager {
    /// Creates a new ResourceManager.
    pub fn new() -> ResourceManager {
        let mut resource_paths = std::collections::HashMap::new();
        maliput_sdk::sdk_resources()
            .iter()
            .for_each(|(backend_name, resource_path)| {
                // All the files in the path are added to the resource manager
                let files = std::fs::read_dir(resource_path).unwrap();
                let mut paths = vec![];
                files.filter(|file| file.is_ok()).for_each(|file| {
                    paths.push(file.unwrap().path());
                });
                resource_paths.insert(backend_name.clone(), paths);
            });
        ResourceManager { resource_paths }
    }

    /// Obtains the path to a resource by its name.
    /// ### Arguments
    ///
    /// * `backend_name` - The name of the backend.
    /// * `resource_name` - The name of the resource.
    ///
    /// ### Returns
    /// The path to the resource if it exists, otherwise None.
    pub fn get_resource_path_by_name(&self, backend_name: &str, resource_name: &str) -> Option<std::path::PathBuf> {
        let paths = self.resource_paths.get(backend_name).unwrap();
        for path in paths {
            if path.to_str().unwrap().contains(resource_name) {
                return Some(path.clone());
            }
        }
        None
    }

    /// Obtains all the resources from a backend.
    /// ### Arguments
    ///
    /// * `backend_name` - The name of the backend.
    ///
    /// ### Returns
    /// A vector with all the resources from the backend if it exists, otherwise None.
    pub fn get_all_resources_by_backend(&self, backend_name: &str) -> Option<&Vec<std::path::PathBuf>> {
        self.resource_paths.get(backend_name)
    }

    /// Get the underlying collection that stores all the resources for each backend.
    pub fn get_all_resources(&self) -> &std::collections::HashMap<String, Vec<std::path::PathBuf>> {
        &self.resource_paths
    }
}

// test

mod tests {

    #[test]
    fn test_maliput_malidrive_resources() {
        let resource_manager = crate::ResourceManager::new();

        let t_shape_road_xodr_path = resource_manager.get_resource_path_by_name("maliput_malidrive", "TShapeRoad.xodr");
        assert!(t_shape_road_xodr_path.is_some());
        assert!(t_shape_road_xodr_path.unwrap().exists());

        let wrong_file_path = resource_manager.get_resource_path_by_name("maliput_malidrive", "wrong_file");
        assert!(wrong_file_path.is_none());

        let all_resources_in_malidrive = resource_manager.get_all_resources_by_backend("maliput_malidrive");
        assert!(all_resources_in_malidrive.is_some());
        assert!(all_resources_in_malidrive.unwrap().len() > 10);

        let all_resources_in_wrong_backend = resource_manager.get_all_resources_by_backend("wrong_backend");
        assert!(all_resources_in_wrong_backend.is_none());

        let all_resources = resource_manager.get_all_resources();
        // There is only one backend supported at the moment.
        assert_eq!(all_resources.len(), 1);
    }
}
