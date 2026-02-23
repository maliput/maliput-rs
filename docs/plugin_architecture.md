# Maliput Plugin Architecture in maliput-rs

This document explains how the maliput C++ plugin architecture is handled from Rust in the maliput-rs project.

## Overview

Maliput has a plugin architecture where backends like `maliput_malidrive` and `maliput_geopackage` are loaded as plugins. The C++ plugin system uses:

- **`MALIPUT_PLUGIN_PATH`**: Environment variable pointing to directories containing plugin `.so` files
- **`MaliputPluginManager`**: Discovers and loads plugins from `MALIPUT_PLUGIN_PATH`
- **`RoadNetworkLoader`**: Interface that plugins implement to create `RoadNetwork` instances

Reference: [Maliput Plugin Architecture Documentation](https://maliput.readthedocs.io/en/latest/html/deps/maliput/html/maliput_plugin_architecture.html)

## Cargo Feature Flags

Backends are exposed as **Cargo features** across all three crates. This controls which plugins are compiled, linked, and available at runtime.

| Feature | Default? | Description |
|---------|----------|-------------|
| `maliput_malidrive` | ✅ Yes | OpenDRIVE (`.xodr`) backend |
| `maliput_geopackage` | ❌ No | GeoPackage (`.gpkg`) backend |
| `all` | ❌ No | Convenience: enables both backends |

Features propagate through the crate dependency chain:

```
maliput (features: maliput_malidrive, maliput_geopackage, all)
  ├── maliput-sys (features forwarded from maliput)
  │     └── maliput-sdk (features forwarded from maliput-sys)
  └── maliput-sdk (features forwarded directly from maliput)
```

Each crate's `Cargo.toml` forwards its feature flags to its dependencies:

```toml
# maliput/Cargo.toml
[features]
default = ["maliput_malidrive"]
all = ["maliput_malidrive", "maliput_geopackage"]
maliput_malidrive = ["maliput-sys/maliput_malidrive", "maliput-sdk/maliput_malidrive"]
maliput_geopackage = ["maliput-sys/maliput_geopackage", "maliput-sdk/maliput_geopackage"]
```

### Build Examples

```bash
# Default (maliput_malidrive only)
cargo build

# Both backends
cargo build --features all
# or equivalently
cargo build --all-features

# Only maliput_geopackage
cargo build --no-default-features --features maliput_geopackage
```

## How maliput-rs Handles the Plugin Architecture

### 1. Plugin Discovery Path Management

The `MALIPUT_PLUGIN_PATH` environment variable is managed automatically in `RoadNetwork::new()` (`maliput/src/api/mod.rs`). Only paths for **enabled features** are included:

```rust
let new_path = match std::env::var_os("MALIPUT_PLUGIN_PATH") {
    Some(current_path) => {
        let mut new_paths = vec![];
        #[cfg(feature = "maliput_malidrive")]
        new_paths.push(maliput_sdk::get_maliput_malidrive_plugin_path());
        #[cfg(feature = "maliput_geopackage")]
        new_paths.push(maliput_sdk::get_maliput_geopackage_plugin_path());
        new_paths.extend(std::env::split_paths(&current_path).collect::<Vec<_>>());
        std::env::join_paths(new_paths).unwrap()
    }
    None => {
        let mut paths = vec![];
        #[cfg(feature = "maliput_malidrive")]
        paths.push(maliput_sdk::get_maliput_malidrive_plugin_path());
        #[cfg(feature = "maliput_geopackage")]
        paths.push(maliput_sdk::get_maliput_geopackage_plugin_path());
        std::env::join_paths(paths).unwrap()
    }
};
std::env::set_var("MALIPUT_PLUGIN_PATH", new_path);
```

The `get_maliput_malidrive_plugin_path()` and `get_maliput_geopackage_plugin_path()` functions in `maliput-sdk` are themselves gated behind `#[cfg(feature = ...)]`, so they only exist when their respective feature is enabled.

### 2. FFI Bridge to C++ Plugin System

The `maliput-sys` crate wraps the C++ `maliput::plugin::CreateRoadNetwork` function (`maliput-sys/src/plugin/plugin.h`):

- Converts Rust `Vec<String>` properties to C++ `std::map<std::string, std::string>`
- Calls the native `maliput::plugin::CreateRoadNetwork()` which internally:
  1. Creates a `MaliputPluginManager`
  2. Searches `MALIPUT_PLUGIN_PATH` for plugins
  3. Loads the requested plugin (e.g., `maliput_malidrive`)
  4. Invokes the `RoadNetworkLoader` interface

The `maliput-sys` build script (`build.rs`) dynamically links against the correct SDK library variant and builds the `MALIPUT_PLUGIN_PATH` from the enabled backends:

```rust
// Library name comes from maliput-sdk's build.rs, varies by enabled features
let maliput_sdk_lib_name = env::var("DEP_MALIPUT_SDK_SDK_LIB_NAME").expect("...");
println!("cargo:rustc-link-lib={}", maliput_sdk_lib_name);

// Build plugin path from enabled backends only
let mut plugin_paths: Vec<PathBuf> = Vec::new();
if let Ok(malidrive_plugin_path) = env::var("DEP_MALIPUT_SDK_MALIPUT_MALIDRIVE_PLUGIN_PATH") {
    plugin_paths.push(PathBuf::from(malidrive_plugin_path));
}
if let Ok(geopackage_plugin_path) = env::var("DEP_MALIPUT_SDK_MALIPUT_GEOPACKAGE_PLUGIN_PATH") {
    plugin_paths.push(PathBuf::from(geopackage_plugin_path));
}
```

### 3. Plugin Binary Vendoring (`maliput-sdk`)

The `maliput-sdk` crate uses Bazel to fetch and build the C++ maliput libraries. The Bazel `BUILD.bazel` defines **separate `cc_binary` targets** for each backend combination:

**BUILD.bazel:**
```bazel
cc_import(
    name = "malidrive_plugin",
    shared_library = "@maliput_malidrive//:maliput_plugins/libmaliput_malidrive_road_network.so",
)

cc_import(
    name = "geopackage_plugin",
    shared_library = "@maliput_geopackage//:maliput_plugins/libmaliput_geopackage_road_network.so",
)

_MALIPUT_CORE_DEPS = [
    "@maliput//:api",
    "@maliput//:base",
    "@maliput//:common",
    "@maliput//:drake",
    "@maliput//:math",
    "@maliput//:geometry_base",
    "@maliput//:plugin",
    "@maliput//:utility",
]

# maliput_sdk with only maliput_malidrive backend.
cc_binary(
    name = "maliput_sdk_malidrive",
    deps = _MALIPUT_CORE_DEPS + [":malidrive_plugin"],
    linkshared = True,
    linkstatic = True,
)

# maliput_sdk with only maliput_geopackage backend.
cc_binary(
    name = "maliput_sdk_geopackage",
    deps = _MALIPUT_CORE_DEPS + [":geopackage_plugin"],
    linkshared = True,
    linkstatic = True,
)

# maliput_sdk with all backends.
cc_binary(
    name = "maliput_sdk",
    deps = _MALIPUT_CORE_DEPS + [":malidrive_plugin", ":geopackage_plugin"],
    linkshared = True,
    linkstatic = True,
)
```

The `maliput-sdk/build.rs` selects the correct Bazel target based on enabled Cargo features:

```rust
let sdk_lib_name = match (malidrive_enabled, geopackage_enabled) {
    (true, true)   => "maliput_sdk",              // Both backends
    (true, false)  => "maliput_sdk_malidrive",     // Only malidrive
    (false, true)  => "maliput_sdk_geopackage",    // Only geopackage
    (false, false) => panic!("At least one backend feature must be enabled"),
};
```

Only the selected Bazel targets are built, avoiding unnecessary compilation of unused backends.

The `build.rs` conditionally exports paths as environment variables — only for enabled backends:
- `MALIPUT_MALIDRIVE_PLUGIN_PATH` → Directory containing `libmaliput_malidrive_road_network.so` (when `maliput_malidrive` feature is enabled)
- `MALIPUT_GEOPACKAGE_PLUGIN_PATH` → Directory containing `libmaliput_geopackage_road_network.so` (when `maliput_geopackage` feature is enabled)
- `DEP_MALIPUT_SDK_SDK_LIB_NAME` → The selected SDK library name, forwarded to `maliput-sys`

### 4. High-Level Rust API

The `RoadNetworkBackend` enum variants are gated behind feature flags:

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RoadNetworkBackend {
    #[cfg(feature = "maliput_malidrive")]
    MaliputMalidrive,
    #[cfg(feature = "maliput_geopackage")]
    MaliputGeopackage,
}
```

Users specify the backend and properties:

```rust
use maliput::api::{RoadNetwork, RoadNetworkBackend};
use std::collections::HashMap;

// Using the maliput_malidrive backend (OpenDRIVE files)
// Requires: feature "maliput_malidrive" (enabled by default)
let props = HashMap::from([
    ("road_geometry_id", "my_road"),
    ("opendrive_file", "/path/to/file.xodr"),
]);
let road_network = RoadNetwork::new(RoadNetworkBackend::MaliputMalidrive, &props)?;

// Using the maliput_geopackage backend (GeoPackage files)
// Requires: feature "maliput_geopackage" (opt-in)
let props = HashMap::from([
    ("road_geometry_id", "my_road"),
    ("geopackage_file", "/path/to/file.gpkg"),
]);
let road_network = RoadNetwork::new(RoadNetworkBackend::MaliputGeopackage, &props)?;
```

### Summary Flow

```
┌──────────────────┐    ┌─────────────────┐    ┌──────────────────────────┐
│ RoadNetwork::new │───►│ Sets env var    │───►│ maliput_sys::plugin::ffi │
│ (maliput crate)  │    │ MALIPUT_PLUGIN_ │    │ ::CreateRoadNetwork()    │
│                  │    │ PATH (only for  │    │                          │
│                  │    │ enabled backends│    │                          │
│                  │    │ via #[cfg])     │    │                          │
└──────────────────┘    └─────────────────┘    └────────────┬─────────────┘
                                                            │
                                                            ▼
┌──────────────────┐    ┌─────────────────┐    ┌──────────────────────────┐
│ maliput-sdk      │◄───│ Plugin .so path │◄───│ C++ MaliputPluginManager │
│ (feature-gated)  │    │ resolution      │    │ loads plugin from        │
│ get_maliput_     │    │                 │    │ MALIPUT_PLUGIN_PATH      │
│ malidrive_       │    │                 │    │                          │
│ plugin_path()    │    │                 │    │                          │
│ get_maliput_     │    │                 │    │                          │
│ geopackage_      │    │                 │    │                          │
│ plugin_path()    │    │                 │    │                          │
└──────────────────┘    └─────────────────┘    └──────────────────────────┘
```

---

## Shared Libraries Involved

There are **three kinds of shared libraries** in this architecture. Which ones are built and linked depends on the enabled Cargo features.

### 1. `libmaliput_sdk_<variant>.so` (Bundled Maliput Core)

This is the **main shared library** that `maliput-sys` links against. It's built by Bazel as a single bundled `.so` containing all the core maliput C++ libraries plus only the enabled backend plugins.

The library name varies based on enabled features:

| Enabled Features | Library Name | Backend Plugins Linked |
|---|---|---|
| `maliput_malidrive` only (default) | `libmaliput_sdk_malidrive.so` | malidrive only |
| `maliput_geopackage` only | `libmaliput_sdk_geopackage.so` | geopackage only |
| Both (`all` feature) | `libmaliput_sdk.so` | malidrive + geopackage |

All variants include these core maliput components:

| Maliput Component | Purpose |
|---|---|
| `@maliput//:api` | Core API (RoadNetwork, RoadGeometry, Lane, etc.) |
| `@maliput//:base` | Base implementations |
| `@maliput//:common` | Common utilities |
| `@maliput//:drake` | Drake math utilities |
| `@maliput//:math` | Math types (Vector3, Quaternion, etc.) |
| `@maliput//:geometry_base` | Geometry base classes |
| `@maliput//:plugin` | **Plugin architecture** (MaliputPluginManager, CreateRoadNetwork) |
| `@maliput//:utility` | Utility functions |

From `maliput-sys/build.rs`:
```rust
// Library name is dynamically determined by maliput-sdk based on enabled features
let maliput_sdk_lib_name = env::var("DEP_MALIPUT_SDK_SDK_LIB_NAME").expect("...");
println!("cargo:rustc-link-search=native={}", maliput_sdk_bin_path.display());
println!("cargo:rustc-link-lib={}", maliput_sdk_lib_name);
```

### 2. `libmaliput_malidrive_road_network.so` (Plugin)

This is the **maliput_malidrive backend plugin**. Only built and linked when the `maliput_malidrive` feature is enabled.

```
maliput_plugins/libmaliput_malidrive_road_network.so
```

This plugin:
- Implements the `RoadNetworkLoader` interface
- Provides OpenDRIVE file parsing and road network creation

### 3. `libmaliput_geopackage_road_network.so` (Plugin)

This is the **maliput_geopackage backend plugin**. Only built and linked when the `maliput_geopackage` feature is enabled.

```
maliput_plugins/libmaliput_geopackage_road_network.so
```

This plugin:
- Implements the `RoadNetworkLoader` interface
- Provides GeoPackage file parsing and road network creation via `maliput_sparse`

### Library Summary

| Library | Type | When Built | When Loaded | Purpose |
|---------|------|------------|-------------|---------|
| `libmaliput_sdk_malidrive.so` | Bundled | feature `maliput_malidrive` only | Compile-time link | Core maliput API + malidrive plugin |
| `libmaliput_sdk_geopackage.so` | Bundled | feature `maliput_geopackage` only | Compile-time link | Core maliput API + geopackage plugin |
| `libmaliput_sdk.so` | Bundled | features `all` (both) | Compile-time link | Core maliput API + both plugins |
| `libmaliput_malidrive_road_network.so` | Plugin | feature `maliput_malidrive` | ELF dependency + runtime `dlopen` | OpenDRIVE backend |
| `libmaliput_geopackage_road_network.so` | Plugin | feature `maliput_geopackage` | ELF dependency + runtime `dlopen` | GeoPackage backend |

---

## When Are the Libraries Actually Loaded?

### Key Finding

The plugin `.so` files are **direct ELF dependencies** of the SDK shared library. For example, with default features (malidrive only):

From `readelf -d libmaliput_sdk_malidrive.so`:
```
NEEDED: libmaliput_malidrive_road_network.so
```

With all features enabled, `readelf -d libmaliput_sdk.so` shows:
```
NEEDED: libmaliput_malidrive_road_network.so
NEEDED: libmaliput_geopackage_road_network.so
```

This means the plugin `.so` files are not just loaded later via `dlopen()` — they are required at program startup by the dynamic linker.

### Loading Timeline

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  1. Program Startup (e.g., maliput_query)                                   │
│                                                                             │
│     Linux dynamic linker (ld.so) reads NEEDED entries:                      │
│       - maliput_query NEEDS libmaliput_sdk_<variant>.so                     │
│       - libmaliput_sdk_<variant>.so NEEDS the enabled plugin .so files      │
│                                                                             │
│     → ONLY the enabled backend .so files are loaded at program start        │
│       (before main() even runs!)                                            │
│                                                                             │
│     Example (default features):                                             │
│       - libmaliput_sdk_malidrive.so NEEDS libmaliput_malidrive_road_network │
│       - libmaliput_geopackage_road_network.so is NOT present at all         │
│                                                                             │
│     Example (all features):                                                 │
│       - libmaliput_sdk.so NEEDS both plugin .so files                       │
│       - Both are loaded at startup                                          │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│  2. Later: RoadNetwork::new(RoadNetworkBackend::MaliputMalidrive, ...) is   │
│     called (or RoadNetwork::new(RoadNetworkBackend::MaliputGeopackage, ...))│
│                                                                             │
│     MaliputPluginManager scans MALIPUT_PLUGIN_PATH and does dlopen()        │
│     on the requested plugin .so                                             │
│                                                                             │
│     → BUT the library is ALREADY in memory!                                 │
│     → dlopen() returns a handle to the SAME loaded instance                 │
│       (Linux doesn't load the same .so twice)                               │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Answers to Common Questions

| Question | Answer |
|----------|--------|
| **When is the SDK `.so` loaded?** | At program startup, before `main()` |
| **Which SDK `.so` is used?** | Depends on features: `libmaliput_sdk_malidrive.so` (default), `libmaliput_sdk_geopackage.so`, or `libmaliput_sdk.so` (both) |
| **Are the plugin `.so` files loaded at startup too?** | **Yes!** Because they are `NEEDED` dependencies of the SDK `.so` |
| **Are unused backends loaded?** | **No.** Only backends selected via Cargo features are linked into the SDK `.so` |
| **Are they loaded again via plugin architecture?** | `dlopen()` is called, but Linux returns a handle to the **already-loaded** library (reference count increases) |
| **Are they loaded twice?** | **No.** Linux's dynamic linker ensures each `.so` is only mapped once into memory |

### Why This Design?

The `cc_import` + dependency in `BUILD.bazel`:
```bazel
# Only the enabled plugins are included as deps
cc_binary(
    name = "maliput_sdk_malidrive",
    deps = _MALIPUT_CORE_DEPS + [":malidrive_plugin"],
)
```

This ensures:
1. **Only needed plugin .so files are built** by Bazel (the target is selected based on Cargo features)
2. **The RUNPATH is set up correctly** so the linker can find the plugins
3. **Downstream packages** know there's a dependency on the enabled backends

### Trade-offs

This approach means:
- ✅ **Only enabled backends are compiled and linked** — Cargo features control which Bazel targets are built
- ✅ **Simpler deployment** — no need to worry about `MALIPUT_PLUGIN_PATH` for the libraries to load
- ✅ **Symbols are available** at startup
- ✅ **Faster builds** when only one backend is needed (avoids fetching/compiling the other)
- ⚠️ **With `all` features, both plugins are always loaded** even if you only use one backend at runtime
- ⚠️ **Loses some "plugin flexibility"** — in a pure plugin architecture, you'd only load backends you actually use

The plugin architecture via `MALIPUT_PLUGIN_PATH` + `dlopen()` still works, but for the vendored plugins it's effectively a no-op since the libraries are already loaded.

---

## Library Loading Flow Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                      Compile Time (static linking)              │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   maliput-sys.rlib ──links──► libmaliput_sdk_<variant>.so       │
│                                (contains maliput core +         │
│                                 only enabled backend plugins)   │
│                                                                 │
│   Variant is selected by Cargo features:                        │
│     default → libmaliput_sdk_malidrive.so                       │
│     geopackage → libmaliput_sdk_geopackage.so                   │
│     all → libmaliput_sdk.so                                     │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                      Runtime (dynamic loading via dlopen)       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   MaliputPluginManager ───dlopen()──► (enabled plugin .so)      │
│   (inside the SDK .so)                                          │
│                                                                 │
│   With default features:                                        │
│     └─dlopen()──► libmaliput_malidrive_road_network.so          │
│                   (already loaded as ELF dep, dlopen is no-op)  │
│                                                                 │
│   With all features:                                            │
│     ├─dlopen()──► libmaliput_malidrive_road_network.so          │
│     └─dlopen()──► libmaliput_geopackage_road_network.so         │
│                   (both already loaded as ELF deps)             │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## ELF Analysis

### SDK Library Dependencies (per variant)

**`libmaliput_sdk_malidrive.so`** (default features — malidrive only):
```
$ readelf -d libmaliput_sdk_malidrive.so | grep NEEDED

NEEDED: libmaliput_malidrive_road_network.so
NEEDED: libdl.so.2
NEEDED: libpthread.so.0
NEEDED: libstdc++.so.6
NEEDED: libm.so.6
NEEDED: libgcc_s.so.1
NEEDED: libc.so.6
NEEDED: ld-linux-x86-64.so.2
```

**`libmaliput_sdk_geopackage.so`** (geopackage only):
```
$ readelf -d libmaliput_sdk_geopackage.so | grep NEEDED

NEEDED: libmaliput_geopackage_road_network.so
NEEDED: libdl.so.2
NEEDED: libpthread.so.0
NEEDED: libstdc++.so.6
NEEDED: libm.so.6
NEEDED: libgcc_s.so.1
NEEDED: libc.so.6
NEEDED: ld-linux-x86-64.so.2
```

**`libmaliput_sdk.so`** (all features — both backends):
```
$ readelf -d libmaliput_sdk.so | grep NEEDED

NEEDED: libmaliput_malidrive_road_network.so
NEEDED: libmaliput_geopackage_road_network.so
NEEDED: libdl.so.2
NEEDED: libpthread.so.0
NEEDED: libstdc++.so.6
NEEDED: libm.so.6
NEEDED: libgcc_s.so.1
NEEDED: libc.so.6
NEEDED: ld-linux-x86-64.so.2
```

### maliput_query Binary Dependencies (default features)

```
$ readelf -d maliput_query | grep -E "(NEEDED|RUNPATH)"

NEEDED: libmaliput_sdk_malidrive.so
NEEDED: libstdc++.so.6
NEEDED: libgcc_s.so.1
NEEDED: libpthread.so.0
NEEDED: libdl.so.2
NEEDED: libc.so.6
NEEDED: ld-linux-x86-64.so.2
RUNPATH: .../bazel-bin:.../bazel-bin/external/maliput_malidrive~0.18.0/maliput_plugins
```

Note: With `all` features the binary would link against `libmaliput_sdk.so` and the RUNPATH would include both plugin directories.

### libmaliput_malidrive_road_network.so Dependencies

```
$ readelf -d libmaliput_malidrive_road_network.so | grep NEEDED

NEEDED: libpthread.so.0
NEEDED: libdl.so.2
NEEDED: libstdc++.so.6
NEEDED: libm.so.6
NEEDED: libgcc_s.so.1
NEEDED: libc.so.6
NEEDED: ld-linux-x86-64.so.2
```

---

## References

- [Maliput Plugin Architecture Documentation](https://maliput.readthedocs.io/en/latest/html/deps/maliput/html/maliput_plugin_architecture.html)
- [maliput-rs GitHub Repository](https://github.com/maliput/maliput-rs)
- [maliput_malidrive GitHub Repository](https://github.com/maliput/maliput_malidrive)
- [maliput_geopackage GitHub Repository](https://github.com/maliput/maliput_geopackage)
