//! Путь к библиотеке MuJoCo для тестов при транзитивной зависимости через sim-world.

fn main() {
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("macos") {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/third_party/mujoco");
        println!("cargo:rustc-link-arg=-Wl,-rpath,{}", dir.display());
    }
}
