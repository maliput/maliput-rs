# maliput-sdk

[<img alt="github" src="https://img.shields.io/badge/github-maliput/maliput-rs?style=for-the-badge&labelColor=555555&logo=github" height="20">](https://github.com/maliput/maliput-rs/maliput-sdk)
[<img alt="crates.io" src="https://img.shields.io/crates/v/maliput-sdk.svg?style=for-the-badge&color=fc8d62&logo=rust" height="20">](https://crates.io/crates/maliput-sdk)
[<img alt="docs.rs" src="https://img.shields.io/badge/docs.rs-maliput-sdk?style=for-the-badge&labelColor=555555&logo=docs.rs" height="20">](https://docs.rs/maliput-sdk)
[<img alt="build status" src="https://img.shields.io/github/actions/workflow/status/maliput/maliput-rs/build.yaml?branch=main&style=for-the-badge" height="20">](https://github.com/maliput/maliput-rs/actions?query=branch%3Amain)

Brings [maliput](https://maliput.readthedocs.io/en/latest/) binaries to Rust land.

_Note: What is maliput? Refer to https://maliput.readthedocs.org._

## Prerequisites

* OS: Ubuntu 20.04
* Bazel 6.4.0

## Description

`maliput-sdk` package relies on maliput releases on the [BCR](https://registry.bazel.build/).

| BCR Module | Current version |
|------------|---------|
| [maliput](https://registry.bazel.build/modules/maliput)    | 1.13.2 |
| [maliput_malidrive](https://registry.bazel.build/modules/maliput_malidrive) | 0.18.0 |
| [maliput_geopackage](https://registry.bazel.build/modules/maliput_geopackage) | 0.0.1 |

## Features

Backends are exposed as Cargo features:

| Feature | Default | Description |
|---------|---------|-------------|
| `maliput_malidrive` | ✅ | OpenDRIVE (`.xodr`) backend |
| `maliput_geopackage` | ❌ | GeoPackage (`.gpkg`) backend |
| `all` | ❌ | Enables both backends |

```sh
# Default (maliput_malidrive only)
cargo build

# Both backends
cargo build --features all

# Only maliput_geopackage
cargo build --no-default-features --features maliput_geopackage
```

## Usage

This package brings the maliput ecosystem and provides the path to where the installation is located.

 - For accessing it via `build.rs` file, some env vars are provided (always available):
   - `DEP_MALIPUT_SDK_BIN_PATH`: Path to maliput-sdk's bazel binaries.
   - `DEP_MALIPUT_SDK_MALIPUT_BIN_PATH`: Path to maliput binaries.
   - `DEP_MALIPUT_SDK_ROOT`: Path to the build output root.
   - `DEP_MALIPUT_SDK_SDK_LIB_NAME`: Name of the SDK shared library (varies by features).
 - Conditional on `maliput_malidrive` feature:
   - `DEP_MALIPUT_SDK_MALIPUT_MALIDRIVE_BIN_PATH`: Path to maliput_malidrive binaries.
   - `DEP_MALIPUT_SDK_MALIPUT_MALIDRIVE_PLUGIN_PATH`: Path to maliput_malidrive road network plugin.
 - Conditional on `maliput_geopackage` feature:
   - `DEP_MALIPUT_SDK_MALIPUT_GEOPACKAGE_BIN_PATH`: Path to maliput_geopackage binaries.
   - `DEP_MALIPUT_SDK_MALIPUT_GEOPACKAGE_PLUGIN_PATH`: Path to maliput_geopackage road network plugin.
 - For accessing it via a library:
   - `maliput_sdk::sdk_libraries` — Returns vendored library paths.
   - `maliput_sdk::sdk_resources` — Returns vendored resource paths.
   - `maliput_sdk::get_maliput_malidrive_plugin_path` — _(requires feature `maliput_malidrive`)_
   - `maliput_sdk::get_maliput_geopackage_plugin_path` — _(requires feature `maliput_geopackage`)_

## Executables

 - Print libraries being vendored
   ```sh
   cargo run --bin maliput-sdk
   ```

## License

Licensed under [BSD 3-Clause](https://github.com/maliput/maliput-rs/blob/main/LICENSE).
