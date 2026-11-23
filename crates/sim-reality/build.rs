fn main() {
    // Путь к тому же фреймворку, с которым связывается sim-mujoco.
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/third_party/mujoco");
    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", dir.display());
}
