# maliput

[<img alt="github" src="https://img.shields.io/badge/github-maliput/maliput-rs?style=for-the-badge&labelColor=555555&logo=github" height="20">](https://github.com/maliput/maliput-rs/maliput)
[<img alt="crates.io" src="https://img.shields.io/crates/v/maliput.svg?style=for-the-badge&color=fc8d62&logo=rust" height="20">](https://crates.io/crates/maliput)
[<img alt="docs.rs" src="https://img.shields.io/badge/docs.rs-maliput?style=for-the-badge&labelColor=555555&logo=docs.rs" height="20">](https://docs.rs/maliput)
[<img alt="build status" src="https://img.shields.io/github/actions/workflow/status/maliput/maliput-rs/build.yaml?branch=main&style=for-the-badge" height="20">](https://github.com/maliput/maliput-rs/actions?query=branch%3Amain)

Creates Rustacean API for [maliput](https://maliput.readthedocs.org).
It is implemented on top of [`maliput-sys`](https://crates.io/crates/maliput-sys) package.

_Note: What is maliput? Refer to https://maliput.readthedocs.org._

## Description

`maliput` provides a Rust API implemented on top of FFI bindings provided by [`maliput-sys`](https://crates.io/crates/maliput-sys) package.


## Usage


```rust
  use maliput::api::RoadNetwork;
  use std::collections::HashMap;

  fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Get location of odr resources
    let package_location = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let xodr_path = format!("{}/data/xodr/TShapeRoad.xodr", package_location);

    let road_network_properties = HashMap::from([
        ("road_geometry_id", "my_rg_from_rust"),
        ("opendrive_file", xodr_path.as_str()),
    ]);

    let road_network = RoadNetwork::new("maliput_malidrive", &road_network_properties)?;
    let road_geometry = road_network.road_geometry();

    // Exercise the RoadGeometry API.
    println!("linear_tolerance: {}", road_geometry.linear_tolerance());
    println!("angular_tolerance: {}", road_geometry.angular_tolerance());
    println!("num_junctions: {}", road_geometry.num_junctions());

    let lanes = road_geometry.get_lanes();
    println!("num_lanes: {}", lanes.len());
    println!("lanes: ");
    for lane in lanes {
        println!("\tlane id: {}", lane.id());
    }
    Ok(())
  }
```

## Apps

 - `maliput_query`: A command-line tool for interactively querying a road network. It loads a road network from an OpenDRIVE file and provides a set of commands to inspect its geometric properties and perform coordinate transformations.

   To run the application:
   ```bash
   cargo run --bin maliput_query -- <path_to_xodr_file>
   ```
   For example, using one of the provided XODR files:
   ```bash
   cargo run --bin maliput_query -- data/xodr/TShapeRoad.xodr
   ```

## Examples

 - Load `maliput::api::RoadNetwork` and perform some basic queries against the Road Geometry.
    ```bash
    cargo run --example 01_road_geometry
    ```

## Benches

 - Evaluate the execution of `maliput::api::RoadGeometry::to_road_position` method.
    ```bash
    cargo bench to_road_position
    ```

## License

Licensed under [BSD 3-Clause](https://github.com/maliput/maliput-rs/blob/main/LICENSE).
