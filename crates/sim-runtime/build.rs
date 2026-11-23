//! Путь к динамической библиотеке для тестов и отпечаток сборки для прогонов.

use std::path::Path;

fn mix(hash: &mut u64, bytes: impl IntoIterator<Item = u8>) {
    // FNV-1a, 64-битный вариант из RFC 9923.
    for byte in bytes {
        *hash ^= u64::from(byte);
        *hash = hash.wrapping_mul(0x100000001b3);
    }
}

fn visit(root: &Path, path: &Path, hash: &mut u64) -> std::io::Result<()> {
    if path.is_dir() {
        let mut entries = std::fs::read_dir(path)?.collect::<Result<Vec<_>, _>>()?;
        entries.sort_by_key(|entry| entry.file_name());
        for item in entries {
            visit(root, &item.path(), hash)?;
        }
    } else if path.is_file() {
        let name = path
            .strip_prefix(root)
            .expect("файл внутри сборки")
            .as_os_str()
            .as_encoded_bytes();
        let content = std::fs::read(path)?;
        mix(hash, (name.len() as u64).to_le_bytes());
        mix(hash, name.iter().copied());
        mix(hash, (content.len() as u64).to_le_bytes());
        mix(hash, content);
    }
    Ok(())
}

fn main() -> std::io::Result<()> {
    if let Ok(dir) = std::env::var("DEP_MUJOCO_FRAMEWORK_DIR") {
        println!("cargo:rustc-link-arg=-Wl,-rpath,{dir}");
    }
    let root = Path::new("../..");
    let mut hash = 0xcbf29ce484222325;
    for relative in [
        "Cargo.lock",
        "crates/sim-runtime/src",
        "crates/sim-runtime/build.rs",
        "crates/sim-world/src",
        "crates/sim-mujoco/src",
        "crates/sim-core/src",
        "crates/sim-signal/src",
        "crates/sim-control/src",
        "crates/sim-boards/src",
        "crates/sim-mcu/src",
        "crates/sim-radio/src",
        "crates/sim-drive/src",
        "crates/sim-sense/src",
        "crates/sim-em/src",
    ] {
        let path = root.join(relative);
        println!("cargo:rerun-if-changed={}", path.display());
        visit(root, &path, &mut hash)?;
    }
    println!("cargo:rerun-if-env-changed=DEP_MUJOCO_FRAMEWORK_DIR");
    println!("cargo:rustc-env=SIM_RUNTIME_BUILD={hash:016x}");
    Ok(())
}
