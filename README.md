# maliput-rs

Provides a Rust API for maliput.

## Packages

* [`maliput-sdk`](./maliput-sdk/): Brings binaries of maliput ecosystem to cargo workspace.
* [`maliput-sys`](./maliput-sys/): Provides ffi Rust bindings for the [`maliput`](https://github.com/maliput/maliput) API.
* [`maliput-rs`]: Provides a rustacean API for maliput on top of `maliput-sys`. (Not implemented yet.)

## Prerequisites

* OS: Ubuntu 20.04
* Bazel

## Installation


```
cargo build
```


## Usage

TODO

## Examples

### maliput-sys
 - Load `maliput::api::RoadNetwork` and perform some basic queries
    ```
    cargo run --examples create_rn
    ```

    _Note: RoadNetworks are loaded via [maliput plugin architecture](https://maliput.readthedocs.io/en/latest/html/deps/maliput/html/maliput_plugin_architecture.html), therefore a valid `MALIPUT_PLUGIN_PATH` env var must be set. It will be handleded automatically in the future when maliput_malidrive's binaries are also brought._
