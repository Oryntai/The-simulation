//! Путь к поставляемой библиотеке MuJoCo для тестов и примеров.
fn main() {
    if let Ok(dir) = std::env::var("DEP_MUJOCO_FRAMEWORK_DIR") {
        println!("cargo:rustc-link-arg=-Wl,-rpath,{dir}");
    }
}
