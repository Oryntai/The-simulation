//! Кладёт скрипт компоновки `link.x` в OUT_DIR и добавляет его в путь поиска
//! компоновщика (так же устроены cortex-m-rt/riscv-rt).
use std::{env, fs, path::PathBuf};

fn main() {
    let out = PathBuf::from(env::var("OUT_DIR").expect("OUT_DIR"));
    fs::write(out.join("link.x"), include_bytes!("link.x")).expect("запись link.x");
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=link.x");
    println!("cargo:rerun-if-changed=build.rs");
}
