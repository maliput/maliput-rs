# Maliput Plugin Architecture in maliput-rs

This document explains how the maliput C++ plugin architecture is handled from Rust in the maliput-rs project.

## Overview

Maliput has a plugin architecture where backends like `maliput_malidrive` and `maliput_geopackage` are loaded as plugins. The C++ plugin system uses:

- **`MALIPUT_PLUGIN_PATH`**: Environment variable pointing to directories containing plugin `.so` files
- **`MaliputPluginManager`**: Discovers and loads plugins from `MALIPUT_PLUGIN_PATH`
- **`RoadNetworkLoader`**: Interface that plugins implement to create `RoadNetwork` instances

Reference: [Maliput Plugin Architecture Documentation](https://maliput.readthedocs.io/en/latest/html/deps/maliput/html/maliput_plugin_architecture.html)

## How maliput-rs Handles the Plugin Architecture

### 1. Plugin Discovery Path Management

The `MALIPUT_PLUGIN_PATH` environment variable is managed automatically in `RoadNetwork::new()` (`maliput/src/api/mod.rs`):

```rust
let new_path = match std::env::var_os("MALIPUT_PLUGIN_PATH") {
    Some(current_path) => {
        // Prepend maliput_sdk's plugin paths to existing paths
        let mut new_paths = vec![
            maliput_sdk::get_maliput_malidrive_plugin_path(),
            maliput_sdk::get_maliput_geopackage_plugin_path(),
        ];
        new_paths.extend(std::env::split_paths(&current_path).collect::<Vec<_>>());
        std::env::join_paths(new_paths).unwrap()
    }
    None => {
        std::env::join_paths([
            maliput_sdk::get_maliput_malidrive_plugin_path(),
            maliput_sdk::get_maliput_geopackage_plugin_path(),
        ])
        .unwrap()
    }
};
std::env::set_var("MALIPUT_PLUGIN_PATH", new_path);
```

### 2. FFI Bridge to C++ Plugin System

The `maliput-sys` crate wraps the C++ `maliput::plugin::CreateRoadNetwork` function (`maliput-sys/src/plugin/plugin.h`):

- Converts Rust `Vec<String>` properties to C++ `std::map<std::string, std::string>`
- Calls the native `maliput::plugin::CreateRoadNetwork()` which internally:
  1. Creates a `MaliputPluginManager`
  2. Searches `MALIPUT_PLUGIN_PATH` for plugins
  3. Loads the requested plugin (e.g., `maliput_malidrive`)
  4. Invokes the `RoadNetworkLoader` interface

### 3. Plugin Binary Vendoring (`maliput-sdk`)

The `maliput-sdk` crate uses Bazel to fetch and build the C++ maliput libraries:

**BUILD.bazel:**
```bazel
cc_import(
    name = "malidrive_plugin",
    shared_library = "@maliput_malidrive//:maliput_plugins/libmaliput_malidrive_road_network.so",
    visibility = ["//visibility:public"],
)

cc_import(
    name = "geopackage_plugin",
    shared_library = "@maliput_geopackage//:maliput_plugins/libmaliput_geopackage_road_network.so",
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "maliput_sdk",
    visibility = ["//visibility:public"],
    deps = [
        "@maliput//:api",
        "@maliput//:base",
        "@maliput//:common",
        "@maliput//:drake",
        "@maliput//:math",
        "@maliput//:geometry_base",
        "@maliput//:plugin",
        "@maliput//:utility",
        ":malidrive_plugin",
        ":geopackage_plugin",
    ],
    linkshared = True,
    linkstatic = True,
)
```

The `build.rs` exports paths as environment variables:
- `MALIPUT_MALIDRIVE_PLUGIN_PATH` → Directory containing `libmaliput_malidrive_road_network.so`
- `MALIPUT_GEOPACKAGE_PLUGIN_PATH` → Directory containing `libmaliput_geopackage_road_network.so`

### 4. High-Level Rust API

Users simply specify the plugin name and properties:

```rust
use maliput::api::{RoadNetwork, RoadNetworkBackend};
use std::collections::HashMap;

// Using the maliput_malidrive backend (OpenDRIVE files)
let props = HashMap::from([
    ("road_geometry_id", "my_road"),
    ("opendrive_file", "/path/to/file.xodr"),
]);
let road_network = RoadNetwork::new(RoadNetworkBackend::MaliputMalidrive, &props)?;

// Using the maliput_geopackage backend (GeoPackage files)
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
│                  │    │ PATH (malidrive │    │                          │
│                  │    │  + geopackage)  │    │                          │
└──────────────────┘    └─────────────────┘    └────────────┬─────────────┘
                                                            │
                                                            ▼
┌──────────────────┐    ┌─────────────────┐    ┌──────────────────────────┐
│ maliput-sdk      │◄───│ Plugin .so path │◄───│ C++ MaliputPluginManager │
│ get_maliput_     │    │ resolution      │    │ loads plugin from        │
│ malidrive_       │    │                 │    │ MALIPUT_PLUGIN_PATH      │
│ plugin_path()    │    │                 │    │                          │
│ get_maliput_     │    │                 │    │                          │
│ geopackage_      │    │                 │    │                          │
│ plugin_path()    │    │                 │    │                          │
└──────────────────┘    └─────────────────┘    └──────────────────────────┘
```

---

## Shared Libraries Involved

There are **three main shared libraries** in this architecture:

### 1. `libmaliput_sdk.so` (Bundled Maliput Core)

This is the **main shared library** that `maliput-sys` links against. It's built by Bazel as a single bundled `.so` that contains all the core maliput C++ libraries:

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
println!("cargo:rustc-link-search=native={}", maliput_sdk_bin_path.display());
println!("cargo:rustc-link-lib=maliput_sdk");
```

### 2. `libmaliput_malidrive_road_network.so` (Plugin)

This is the **maliput_malidrive backend plugin**:

```
maliput_plugins/libmaliput_malidrive_road_network.so
```

This plugin:
- Implements the `RoadNetworkLoader` interface
- Provides OpenDRIVE file parsing and road network creation

### 3. `libmaliput_geopackage_road_network.so` (Plugin)

This is the **maliput_geopackage backend plugin**:

```
maliput_plugins/libmaliput_geopackage_road_network.so
```

This plugin:
- Implements the `RoadNetworkLoader` interface
- Provides GeoPackage file parsing and road network creation via `maliput_sparse`

### Library Summary

| Library | Type | When Loaded | Purpose |
|---------|------|-------------|---------|
| `libmaliput_sdk.so` | Bundled | Compile-time link | Core maliput API + plugin manager |
| `libmaliput_malidrive_road_network.so` | Plugin | Runtime (`dlopen`) | OpenDRIVE backend implementation |
| `libmaliput_geopackage_road_network.so` | Plugin | Runtime (`dlopen`) | GeoPackage backend implementation |

---

## When Are the Libraries Actually Loaded?

### Key Finding

From `readelf -d libmaliput_sdk.so`:
```
NEEDED: libmaliput_malidrive_road_network.so
NEEDED: libmaliput_geopackage_road_network.so
```

This means **the plugin `.so` files are direct ELF dependencies** of `libmaliput_sdk.so`, not just plugins loaded later via `dlopen()`.

### Loading Timeline

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  1. Program Startup (e.g., maliput_query)                                   │
│                                                                             │
│     Linux dynamic linker (ld.so) reads NEEDED entries:                      │
│       - maliput_query NEEDS libmaliput_sdk.so                               │
│       - libmaliput_sdk.so NEEDS libmaliput_malidrive_road_network.so        │
│       - libmaliput_sdk.so NEEDS libmaliput_geopackage_road_network.so       │
│                                                                             │
│     → ALL .so files are loaded into memory IMMEDIATELY at program start     │
│       (before main() even runs!)                                            │
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
| **When is `libmaliput_sdk.so` loaded?** | At program startup, before `main()` |
| **Are the plugin `.so` files loaded at startup too?** | **Yes!** Because they are `NEEDED` dependencies of `libmaliput_sdk.so` |
| **Are they loaded again via plugin architecture?** | `dlopen()` is called, but Linux returns a handle to the **already-loaded** library (reference count increases) |
| **Are they loaded twice?** | **No.** Linux's dynamic linker ensures each `.so` is only mapped once into memory |

### Why This Design?

The `cc_import` + dependency in `BUILD.bazel`:
```bazel
cc_binary(
    name = "maliput_sdk",
    deps = [
        ...
        ":malidrive_plugin",    # ← This creates the NEEDED entry for malidrive
        ":geopackage_plugin",   # ← This creates the NEEDED entry for geopackage
    ],
)
```

This was added to ensure:
1. **The plugin .so files are built** when building `maliput_sdk`
2. **The RUNPATH is set up correctly** so the linker can find the plugins
3. **Downstream packages** know there's a dependency on the backends

### Trade-offs

This approach means:
- ✅ **Simpler deployment** - no need to worry about `MALIPUT_PLUGIN_PATH` for the libraries to load
- ✅ **Symbols are available** at startup
- ⚠️ **All plugins are always loaded** even if you never create a RoadNetwork or only use one backend
- ⚠️ **Loses some "plugin flexibility"** - in a pure plugin architecture, you'd only load backends you actually use

The plugin architecture via `MALIPUT_PLUGIN_PATH` + `dlopen()` still works, but for the vendored plugins it's effectively a no-op since the libraries are already loaded.

---

## Library Loading Flow Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                      Compile Time (static linking)              │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   maliput-sys.rlib ──────links────► libmaliput_sdk.so          │
│                                      (contains maliput core +   │
│                                       plugin infrastructure)    │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                      Runtime (dynamic loading via dlopen)       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   MaliputPluginManager ───dlopen()──► libmaliput_malidrive_     │
│   (inside libmaliput_sdk.so)          road_network.so           │
│                                       (discovered via           │
│                          │             MALIPUT_PLUGIN_PATH)     │
│                          │                                      │
│                          └─dlopen()──► libmaliput_geopackage_   │
│                                        road_network.so          │
│                                        (discovered via          │
│                                         MALIPUT_PLUGIN_PATH)    │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## ELF Analysis

### libmaliput_sdk.so Dependencies

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

### libmaliput_sdk.so RUNPATH

```
$ readelf -d libmaliput_sdk.so | grep RUNPATH

RUNPATH: $ORIGIN/_solib_k8/_U_S_S_Cmalidrive_Uplugin___Uexternal_Smaliput_Umalidrive~0.17.2_Smaliput_Uplugins:...
```

### maliput_query Binary Dependencies

```
$ readelf -d maliput_query | grep -E "(NEEDED|RUNPATH)"

NEEDED: libmaliput_sdk.so
NEEDED: libstdc++.so.6
NEEDED: libgcc_s.so.1
NEEDED: libpthread.so.0
NEEDED: libdl.so.2
NEEDED: libc.so.6
NEEDED: ld-linux-x86-64.so.2
RUNPATH: .../bazel-bin:.../bazel-bin/external/maliput_malidrive~0.18.0/maliput_plugins:.../bazel-bin/external/maliput_geopackage~0.1.0/maliput_plugins
```

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
