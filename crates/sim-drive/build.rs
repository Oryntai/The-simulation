//! rpath поставляемого MuJoCo для тестов на macOS.

fn main() {
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() != Ok("macos") {
        return;
    }
    println!("cargo:rerun-if-changed=build.rs");
    let manifest = std::path::PathBuf::from(std::env::var_os("CARGO_MANIFEST_DIR").unwrap());
    // Cargo передаёт DEP_MUJOCO_FRAMEWORK_DIR только прямой обычной
    // зависимости. С этим движком крейт связан через sim-runtime.
    let directory = manifest
        .join("../../assets/third_party/mujoco")
        .canonicalize()
        .expect("не найден каталог поставляемого MuJoCo");
    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", directory.display());
}
