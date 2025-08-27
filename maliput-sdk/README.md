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
| [maliput](https://registry.bazel.build/modules/maliput)    | 1.7.0 |
| [maliput_malidrive](https://registry.bazel.build/modules/maliput_malidrive) | 0.10.0 |

## Usage

This package brings maliput-ecosystem and provides the path to where the installation is located.

 - For accessing it via `build.rs` file, some env var are provided:
   - `MALIPUT_SDK_BIN_PATH`: Path to maliput-sdk's bazel binaries.
   - `MALIPUT_SDK_MALIPUT_BIN_PATH`: Path to maliput binaries.
   - `MALIPUT_SDK_MALIPUT_MALIDRIVE_BIN_PATH`: Path to maliput_malidrive binaries.
   - `MALIPUT_SDK_MALIPUT_MALIDRIVE_PLUGIN_PATH`: Path to maliput_malidrive road network plugin.
 - For accessing it via a library:
   - `maliput-sdk::sdk_libraries`
   - `maliput-sdk::get_maliput_malidrive_plugin_path`

## Executables

 - Print libraries being vendored
   ```sh
   cargo run --bin maliput-sdk
   ```

## License

Licensed under [BSD 3-Clause](https://github.com/maliput/maliput-rs/blob/main/LICENSE).
