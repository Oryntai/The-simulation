#!/bin/sh
# Сборка трёх независимых стендов с уже установленной целью RISC-V.
set -eu
cd "$(dirname "$0")"
build_dir="$(mktemp -d)"
trap 'rm -rf "$build_dir"' EXIT
build() {
    rustc ports.rs --edition 2024 --target riscv32imc-unknown-none-elf --cfg "$1" \
        -C panic=abort -C opt-level=s -C "link-arg=-T$2.ld" -C strip=symbols \
        -C link-arg=-z -C link-arg=max-page-size=256 -o "$build_dir/$3.elf"
}
build esp_bus esp esp_bus
build esp_uart esp esp_uart
build vmcu vmcu vmcu_ports
du -sk "$build_dir"
cp "$build_dir/"*.elf ../testdata/
