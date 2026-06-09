# GitHub Copilot Instructions for maliput-rs

## Repository Overview

**maliput-rs** provides Rust bindings for the [maliput](https://github.com/maliput/maliput) C++ road network API. It uses [cxx](https://cxx.rs/) for FFI bridging.

**Rust toolchain:** 1.89.0 (see `rust-toolchain.toml`)

## Workspace Crates

| Crate | Role |
|-------|------|
| `maliput-sdk` | Brings pre-built maliput C++ binaries (via Bazel) into the cargo workspace |
| `maliput-sys` | Low-level FFI bindings using `cxx::bridge` — defines the C++/Rust boundary |
| `maliput` | High-level Rustacean API wrapping `maliput-sys` with safe types and error handling |

**Dependency flow:** `maliput` → `maliput-sys` → `maliput-sdk` (provides C++ libs + headers)

## Build, Test, and Lint

```bash
# Build
cargo build

# Test all
cargo test

# Test a single test
cargo test --package maliput -- road_network_test::road_network_new

# Lint (clippy)
cargo clippy --workspace --all-targets --all-features -- -D warnings

# Format check
cargo fmt --all -- --check

# Build docs
cargo doc --all-features --no-deps

# Run all pre-commit checks (format + clippy + docs)
pre-commit run --all-files

# Run benchmarks
cargo bench --package maliput
```

### Feature Flags

- `maliput_malidrive` (default) — enables the OpenDRIVE backend
- `maliput_geopackage` (opt-in) — enables the GeoPackage backend
- `tui` — enables the `maliput_query` TUI binary
- `all` — enables all backends

## Architecture: FFI Binding Pattern

Adding a new maliput API binding follows this layered pattern:

### 1. `maliput-sys` — Define the FFI boundary

- **`src/<module>/mod.rs`**: Contains `#[cxx::bridge]` definitions declaring C++ types, functions, and shared structs.
- **`src/<module>/<module>.h`**: C++ header with free functions that adapt C++ APIs for cxx compatibility (e.g., returning `unique_ptr`, converting types).
- **`src/<module>/<module>.cc`** (if needed): C++ implementations for complex adapters.
- **`build.rs`**: Registers source files for cxx-build compilation. Must list new `.h`/`.cc` files.

Shared structs (e.g., `ConstLanePtr`) are used when cxx cannot directly handle pointer types in collections.

### 2. `maliput` — Safe Rust wrapper

- Wraps raw `cxx::UniquePtr<ffi::Type>` in safe Rust structs.
- Uses `MaliputError` (via `thiserror`) for fallible operations instead of panics.
- Mirrors the C++ namespace structure: `maliput::api`, `maliput::math`, etc.
- Enums like `RoadNetworkBackend` use `strum` for string conversion matching C++ plugin IDs.

### 3. Tests

- `maliput-sys/tests/` — FFI-level integration tests validating the C++ bridge works.
- `maliput/tests/` — Higher-level tests exercising the Rust API. Each test file typically loads a road network from `maliput/data/xodr/` using `CARGO_MANIFEST_DIR`.

## Key Conventions

- **Line width:** 120 characters (`rustfmt.toml`)
- **Clippy:** `absolute-paths-max-segments = 2` (`.clippy.toml`)
- **License header:** BSD 3-Clause on every source file (see existing files)
- **Error handling:** Return `Result<T, MaliputError>` from public APIs; don't panic
- **Module structure:** Mirrors maliput C++ namespaces (`api/`, `math/`, `common/`, `utility/`)
- **Namespace in cxx bridges:** `#[cxx::bridge(namespace = "maliput::api")]`
- **Shared pointer structs:** When cxx can't handle `*const T` in `CxxVector`, define a shared struct wrapper (e.g., `ConstLanePtr`)
- **`build.rs` in `maliput-sys`:** Must declare `cargo:rerun-if-changed` for all FFI source files
- **`build.rs` in `maliput`:** Passes `DEP_*` env vars (from `maliput-sdk`) to dependents via the `links = "maliput"` key

## CI

CI runs in a custom container (`ghcr.io/<repo>-rust-ci:latest`) with maliput C++ dependencies pre-installed. Locally, use the devcontainer (`.devcontainer/`) for equivalent setup.

---

**Trust these instructions.** Only perform additional exploration if information is missing or incorrect.
