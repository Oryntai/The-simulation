//! Программе нужен путь к mujoco.framework во время запуска (rpath): абсолютный — для этой
//! папки, и относительный — если `sim/` перенесут вместе с `target/`.
fn main() {
    if let Ok(dir) = std::env::var("DEP_MUJOCO_FRAMEWORK_DIR") {
        println!("cargo:rustc-link-arg-bins=-Wl,-rpath,{dir}");
    }
    println!(
        "cargo:rustc-link-arg-bins=-Wl,-rpath,@executable_path/../../assets/third_party/mujoco"
    );
}
