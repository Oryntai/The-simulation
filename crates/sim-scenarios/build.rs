//! Тестам этого crate (и модульным, и из tests/) нужен путь к mujoco.framework при запуске (rpath).
fn main() {
    if let Ok(dir) = std::env::var("DEP_MUJOCO_FRAMEWORK_DIR") {
        println!("cargo:rustc-link-arg=-Wl,-rpath,{dir}");
    }
}
