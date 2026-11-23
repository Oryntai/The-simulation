#!/bin/sh
# Собирает образ стенда и кладёт его в папку проектов. Временные файлы удаляются.
set -eu
cd "$(dirname "$0")"
build_dir="$(mktemp -d "${TMPDIR:-/tmp}/bench-enable.XXXXXXXX")"
trap 'rm -rf "$build_dir"' EXIT
CARGO_TARGET_DIR="$build_dir" \
RUSTFLAGS="-C link-arg=-Tpico.ld -C strip=symbols -C link-arg=-z -C link-arg=max-page-size=256" \
    cargo build --offline --release --target thumbv6m-none-eabi
cp "$build_dir/thumbv6m-none-eabi/release/bench-enable" ../../projects/elbow-bench.elf
