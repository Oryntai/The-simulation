//! Отдельный формат каталога точек потребления МК.

use std::path::Path;

use sim_process::schematic::mcu_energy::McuPowerProfile;
use sim_runtime::control::ControlConfig;

pub fn parse_mcu_power_profile(path: &Path, bytes: &[u8]) -> Result<McuPowerProfile, String> {
    std::str::from_utf8(bytes)
        .map_err(|e| e.to_string())
        .and_then(ControlConfig::parse_mcu_power_profile)
        .map_err(|e| format!("{}: {e}", path.display()))
}

pub fn load_mcu_power_profile(path: &Path) -> Result<McuPowerProfile, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
    parse_mcu_power_profile(path, &bytes)
}
