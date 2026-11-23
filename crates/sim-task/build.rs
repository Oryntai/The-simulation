//! Путь к динамической библиотеке для тестов рантайма.
fn main() {
    if let Ok(dir) = std::env::var("DEP_MUJOCO_FRAMEWORK_DIR") {
        println!("cargo:rustc-link-arg=-Wl,-rpath,{dir}");
    }
}
