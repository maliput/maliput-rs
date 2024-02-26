#!/bin/bash

cargo clippy --all-targets --workspace --fix
cargo fmt --all
