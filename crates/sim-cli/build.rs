//! Путь к динамической библиотеке для программы и тестов CLI.
fn main() {
    #[cfg(target_os = "macos")]
    {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/third_party/mujoco");
        println!("cargo:rustc-link-arg=-Wl,-rpath,{}", dir.display());
    }
}
