#!/bin/bash

cargo clippy --all-targets --workspace --fix --allow-staged
cargo fmt --all
