//! Линкуемся с mujoco.framework из sim/assets/third_party/mujoco (см. PROVENANCE.md)
//! и генерируем Rust-объявления из заголовков MuJoCo.

use std::env;
use std::path::PathBuf;

fn main() {
    let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let fw_dir = manifest
        .join("../../assets/third_party/mujoco")
        .canonicalize()
        .expect("нет sim/assets/third_party/mujoco/mujoco.framework — см. sim/assets/third_party/PROVENANCE.md");
    let header = fw_dir.join("mujoco.framework/Headers/mujoco.h");
    println!("cargo:rerun-if-changed={}", header.display());
    println!("cargo:rerun-if-changed=build.rs");

    println!("cargo:rustc-link-search=framework={}", fw_dir.display());
    println!("cargo:rustc-link-lib=framework=mujoco");
    // rpath для тестов этого crate; зависимым — через DEP_MUJOCO_FRAMEWORK_DIR.
    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", fw_dir.display());
    println!("cargo:framework_dir={}", fw_dir.display());

    let bindings = bindgen::Builder::default()
        .header(header.to_str().unwrap())
        .clang_arg(format!("-F{}", fw_dir.display()))
        .allowlist_function("mj_.*")
        .allowlist_function("mju_.*")
        .allowlist_function("mjs_.*")
        .allowlist_type("mj.*")
        .allowlist_var("mj.*")
        .constified_enum_module("mjt.*")
        .layout_tests(false)
        .generate_comments(false)
        .generate()
        .expect("bindgen не смог разобрать mujoco.h");
    let out = PathBuf::from(env::var("OUT_DIR").unwrap()).join("bindings.rs");
    bindings
        .write_to_file(out)
        .expect("не записать bindings.rs");
}
