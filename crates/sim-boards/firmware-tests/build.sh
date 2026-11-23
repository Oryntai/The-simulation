#!/bin/sh
# Сборочные файлы временные, образы обновляются только после успешной сборки.
set -eu
cd "$(dirname "$0")"
build_dir="$(mktemp -d "$PWD/.сборка.XXXXXXXX")"
trap 'rm -rf "$build_dir"' EXIT
export CARGO_TARGET_DIR="$build_dir"
build() {
    target="$1"
    profile="$2"
    RUSTFLAGS="--check-cfg=cfg(pico) --check-cfg=cfg(stm) --cfg $profile -C link-arg=-T$profile.ld -C strip=symbols -C link-arg=-z -C link-arg=max-page-size=256" \
        cargo build --offline --release --target "$target"
    cp "$CARGO_TARGET_DIR/$target/release/boards-firmware-tests" "../testdata/$profile.elf"
}
build thumbv6m-none-eabi pico
build thumbv7m-none-eabi stm
build riscv32imc-unknown-none-elf esp
python3 avr_image.py
build_kib="$(du -sk "$build_dir" | cut -f 1)"
printf 'Временный каталог сборки к удалению: %s КиБ\n' "$build_kib"
