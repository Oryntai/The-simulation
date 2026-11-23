#!/bin/sh
# Сборка прошивок VMCU-C3 и копирование ELF в assets/firmware/ (их загружает приложение
# и встраивает sim-mcu). Нужны только rustup-цель riscv32imc-unknown-none-elf и rustc
# (компоновщик rust-lld идёт с rustc); внешних зависимостей нет.
#
#   rustup target add riscv32imc-unknown-none-elf   # один раз
#   firmware/build.sh
set -eu
cd "$(dirname "$0")"
export CARGO_TARGET_DIR="$PWD/target"
cargo build --release -p selftest -p exo-ctrl
out=../assets/firmware
mkdir -p "$out"
cp target/riscv32imc-unknown-none-elf/release/selftest "$out/selftest.elf"
cp target/riscv32imc-unknown-none-elf/release/exo-ctrl "$out/exo-ctrl.elf"
ls -l "$out"/*.elf
