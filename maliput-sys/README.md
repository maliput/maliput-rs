# maliput-sys

[<img alt="github" src="https://img.shields.io/badge/github-maliput/maliput-rs?style=for-the-badge&labelColor=555555&logo=github" height="20">](https://github.com/maliput/maliput-rs/maliput-sys)
[<img alt="crates.io" src="https://img.shields.io/crates/v/maliput-sys.svg?style=for-the-badge&color=fc8d62&logo=rust" height="20">](https://crates.io/crates/maliput-sys)
[<img alt="docs.rs" src="https://img.shields.io/badge/docs.rs-maliput-sys?style=for-the-badge&labelColor=555555&logo=docs.rs" height="20">](https://docs.rs/maliput-sys)
[<img alt="build status" src="https://img.shields.io/github/actions/workflow/status/maliput/maliput-rs/build.yaml?branch=main&style=for-the-badge" height="20">](https://github.com/maliput/maliput-rs/actions?query=branch%3Amain)

Creates `FFI` bindings using [`cxx`](https://crates.io/crates/cxx) of [maliput](https://maliput.readthedocs.org).
It relies on [maliput-sdk](https://crates.io/crates/maliput-sdk) to bring the maliput-ecosystem to Rust land.

_Note: What is maliput? Refer to https://maliput.readthedocs.org._

## Description

`maliput-sys` provides FFI bindings on top of `maliput-sdk` package.


## Usage

The `maliput` namespace is respected and positioned behind a `ffi` namespace.
For example:

| C++ Namespace | Rust Namespace |
| -------------- | ------------- |
| maliput::api::Lane   | maliput_sys::api::ffi::Lane |
| maliput::api::RoadGeometry   | maliput_sys::api::ffi::RoadGeometry |
| maliput::api::RoadNetwork   | maliput_sys::api::ffi::RoadNetwork |
| maliput::math::Vector3   | maliput_sys::math::ffi::Vector3 |

## Examples

 - Load `maliput::api::RoadNetwork` and perform some basic queries.
    ```bash
    cargo run --example create_rn
    ```

## License

Licensed under [BSD 3-Clause](https://github.com/maliput/maliput-rs/blob/main/LICENSE).
