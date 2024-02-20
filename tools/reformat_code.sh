#!/bin/bash

cargo fmt --all
cargo clippy --all-targets --workspace --fix
