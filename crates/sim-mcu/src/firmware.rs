//! Собранные прошивки, встроенные в библиотеку (чтобы сверки работали без
//! RISC-V-инструментов). Те же файлы лежат в `assets/firmware/` для загрузки
//! приложением во время работы. Пересборка — `firmware/build.sh`.

/// Самопроверка ядра (`firmware/selftest`).
pub fn selftest_elf() -> &'static [u8] {
    include_bytes!("../../../assets/firmware/selftest.elf")
}

/// Контроллер локтевого экзоскелета (`firmware/exo-ctrl`).
pub fn exo_ctrl_elf() -> &'static [u8] {
    include_bytes!("../../../assets/firmware/exo-ctrl.elf")
}

/// Путь к прошивкам относительно корня рабочего пространства.
pub const ASSETS_DIR: &str = "assets/firmware";
