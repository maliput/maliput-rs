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

## Executables

### maliput-sdk

 - Print libraries being vendored
   ```sh
   cargo run --bin maliput-sdk
   ```

 - Get location of maliput_malidrive's plugin library:
   ```sh
   cargo run --bin maliput_malidrive_plugin_path
   ```

## Examples

### maliput-sys

 - Load `maliput::api::RoadNetwork` and perform some basic queries
    ```
    cargo run --example create_rn
    ```
