# maliput-rs

Provides a Rust API for maliput.

## Packages

* [`maliput-sdk`](./maliput-sdk/): Brings binaries of maliput ecosystem to cargo workspace.
* [`maliput-sys`](./maliput-sys/): Provides ffi Rust bindings for the [`maliput`](https://github.com/maliput/maliput) API.
* [`maliput`]: Provides a Rustacean API for maliput on top of `maliput-sys`.

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

## Developer Guidelines

1. A `devcontainer` is provided to easy setup the workspace. Refer to [.devcontainer](.devcontainer/README.md)

2. `pre-commit` is configured at this repository. Execute pre-commit locally before creating a pull request.
    ```
    pre-commit run --all-files
    ```
    _Note: For installing `pre-commit` tool: `pip3 install pre-commit`_
