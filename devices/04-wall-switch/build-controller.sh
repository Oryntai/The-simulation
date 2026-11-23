#!/bin/sh
# Сборка образа RV32IMC существующим локальным LLVM, без установки пакетов.
set -eu
cd "$(dirname "$0")"
scratch="$(mktemp -d)"
trap 'rm -rf "$scratch"' EXIT
sysroot="$(rustc --print sysroot)"
host="$(rustc -vV | sed -n 's/^host: //p')"
build() {
variant="$1"
shift
for symbol do
    printf '.set %s, 1\n' "$symbol"
done > "$scratch/firmware.S"
cat stand-controller.S >> "$scratch/firmware.S"
cat > "$scratch/wrapper.rs" <<'WRAPPER'
#![no_std]
core::arch::global_asm!(include_str!("firmware.S"));
WRAPPER
rustc --target riscv32imc-unknown-none-elf --crate-type rlib --emit=obj \
    -o "$scratch/controller.o" "$scratch/wrapper.rs"
"$sysroot/lib/rustlib/$host/bin/rust-lld" -flavor gnu -m elf32lriscv -s -z max-page-size=256 \
    -T ../../crates/sim-boards/firmware-tests/esp.ld -o "$scratch/controller.elf" "$scratch/controller.o"
cp "$scratch/controller.elf" "$variant"
}
build stand-controller.elf
build stand-ignore.elf IGNORE_SENSOR
