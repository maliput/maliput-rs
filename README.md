# maliput-rs

Provides a Rust API for maliput.

## Packages

* [<img alt="crates.io" src="https://img.shields.io/crates/v/maliput-sdk.svg?style=for-the-badge&color=fc8d62&logo=rust" height="20">](https://crates.io/crates/maliput-sdk) -> [`maliput-sdk`](./maliput-sdk/): Brings binaries of maliput ecosystem to cargo workspace.
* [<img alt="crates.io" src="https://img.shields.io/crates/v/maliput-sys.svg?style=for-the-badge&color=fc8d62&logo=rust" height="20">](https://crates.io/crates/maliput-sys) -> [`maliput-sys`](./maliput-sys/): Provides ffi Rust bindings for the [`maliput`](https://github.com/maliput/maliput) API.
* [<img alt="crates.io" src="https://img.shields.io/crates/v/maliput.svg?style=for-the-badge&color=fc8d62&logo=rust" height="20">](https://crates.io/crates/maliput)  -> [`maliput`](./maliput/) : Provides a Rustacean API for maliput on top of `maliput-sys`.

## Prerequisites

* OS: Ubuntu 20.04
* Bazel

## Developer Guidelines

1. A `devcontainer` is provided to easy setup the workspace. Refer to [.devcontainer](.devcontainer/README.md)

2. `pre-commit` is configured at this repository. Execute pre-commit locally before creating a pull request.
    ```
    pre-commit run --all-files
    ```
    _Note: For installing `pre-commit` tool: `pip3 install pre-commit`_

3. A hook will automatically added to run `pre-commit` every time a `git commit` call is made.
  In order to skip this behavior you can `PRE_COMMIT_ALLOW_NO_CONFIG=1 git commit`.

## Cargo build

Build the workspace

```
cargo build
```

## Usage

Rely on each package's readme for understanding their issue.

## Releasing process over crates.io

The releasing process is done by following the suggestions at https://doc.rust-lang.org/cargo/reference/publishing.html

Each package in the workspace is published separately.

### Steps:

 - Identify which package is wanted to be released.
 - Bumps `Cargo.toml`'s version value.
   - Keep in mind [the SemVer rules](https://doc.rust-lang.org/cargo/reference/semver.html) which provide guidelines on what is a compatible change.
   - Keep in mind packages like `maliput-sdk` which also keeps a version for the bazel module.
 - Run `cargo publish` in dry-run mode to verify its packaging.
   - `cargo publish -p <package> --dry-run`
 - Manually publish the crate.
   - `cargo publish -p <package>`
   - In order to push you need to be whitelisted. Check the
     owners of the packages in crates.io to ask for permissions.
