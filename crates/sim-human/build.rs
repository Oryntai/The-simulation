//! rpath на mujoco.framework для тестов и примеров этого крейта (путь отдаёт sim-mujoco через `links`).

fn main() {
    println!("cargo:rerun-if-env-changed=DEP_MUJOCO_FRAMEWORK_DIR");
    if let Ok(dir) = std::env::var("DEP_MUJOCO_FRAMEWORK_DIR") {
        println!("cargo:rustc-link-arg=-Wl,-rpath,{dir}");
    }
}
