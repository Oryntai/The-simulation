//! Цифровая граница внутренней шины REGI2C ESP32-C3.

use super::Access;
use crate::{
    BoardFault,
    esp_clock::{PLL_320_HZ, PLL_480_HZ},
    esp_reset_profile::EspResetValues,
};

// esp32c3_rev3_rom.elf, rom_chip_i2c_{readReg_org,writeReg} и
// rom_get_i2c_hostid: два командных слова, выбор хоста по ведомому.
// ABI ESP-IDF esp_rom/include/esp_rom_regi2c.h:17–59 и esp_hal_regi2c/esp32c3/rom.regi2c.ld:6–9.
const MASTER0: u32 = 0x6000_e000;
const MASTER1: u32 = MASTER0 + 4;
const START: u32 = 1 << 26;
const BUSY: u32 = 1 << 25;
const WRITE: u32 = 1 << 24;
const DATA_SHIFT: u32 = 16;
const REGISTER_SHIFT: u32 = 8;
const DATA: u32 = (u8::MAX as u32) << DATA_SHIFT;
const COMMAND: u32 = (START | WRITE) | (DATA | u16::MAX as u32);

// ESP-IDF v6.1.1 soc/esp32c3/include/soc/regi2c_defs.h:12–26.
const ANA_CONF0: u32 = 0x6000_e040;
const ANA_CONFIG: u32 = 0x6000_e044;
const ANA_CONFIG2: u32 = 0x6000_e048;
const STOP_HIGH: u32 = 1 << 2;
const STOP_LOW: u32 = 1 << 3;
// ESP32-C3 PAC, i2c_ana_mst/ana_conf0.rs:27–30, поле BBPLL_CAL_DONE.
// https://github.com/esp-rs/esp-pacs/blob/main/esp32c3/src/i2c_ana_mst/ana_conf0.rs#L27-L30
const CAL_DONE: u32 = 1 << 24;
const BBPLL_STOP_FIELD: &str = "BBPLL_STOP_FORCE_HIGH/LOW";

// Адреса и номера регистров: ESP-IDF soc/esp32c3/include/soc/
// regi2c_{lp_bias,brownout,bbpll,saradc,bias,dig_reg}.h соответственно.
// Выбор ведомого для чтения: esp32c3_rev3_rom.elf, таблица
// rom_get_i2c_read_mask в 0x3ff1a134, индексируется от 0x61.
const ULP_BOD: u8 = 0x61;
const BBPLL: u8 = 0x66;
const SAR: u8 = 0x69;
const BIAS: u8 = 0x6a;
const DIG_REG: u8 = 0x6d;
const BBPLL_STATUS: u8 = 8; // regi2c_bbpll.h:121–139, OR_CAL_* и OR_LOCK.
const BYTE_FIELDS: &[(u8, u8, &str)] = &[
    (BBPLL, 5, "BBPLL.5"),       // regi2c_bbpll.h: OC_DR1/DR3 и EN_USB.
    (BBPLL, 9, "BBPLL.9"),       // regi2c_bbpll.h: OC_VCO_DBIAS и BBADC.
    (SAR, 7, "SARADC.7"),        // regi2c_saradc.h: DTEST, ENT, ENCAL_REF.
    (DIG_REG, 13, "DIG_REG.13"), // regi2c_dig_reg.h: настройки цифрового регулятора.
    (DIG_REG, 4, "DIG_REG.4"),   // regi2c_dig_reg.h: RTC_DREG и ENX.
    (DIG_REG, 5, "DIG_REG.5"),   // regi2c_dig_reg.h: EXT_RTC_DREG_SLEEP.
    (DIG_REG, 6, "DIG_REG.6"),   // regi2c_dig_reg.h: DIG_DREG и ENX.
    (DIG_REG, 7, "DIG_REG.7"),   // regi2c_dig_reg.h: EXT_DIG_DREG_SLEEP.
    (ULP_BOD, 0, "ULP_BOD.0"),   // regi2c_lp_bias.h: ULP_IR_FORCE_XPD_IPH.
    (ULP_BOD, 5, "ULP_BOD.5"),   // regi2c_brownout.h: I2C_BOD_THRESHOLD.
];
const ULP_DONE: &str = "ULP_BOD.3"; // regi2c_lp_bias.h: O_DONE_FLAG и BG_O_DONE_FLAG.

pub(super) fn supports_reset_value(address: u32, field: &str, value: u32) -> bool {
    match (address, field) {
        (ANA_CONFIG, "ANA_CONFIG_REG") | (ANA_CONFIG2, "ANA_CONFIG2_REG") => true,
        (ANA_CONF0, BBPLL_STOP_FIELD) => value & !(STOP_HIGH | STOP_LOW) == 0,
        (MASTER0, "I2C_MST0_COMMAND") => value & !(COMMAND | BUSY) == 0,
        (MASTER0, ULP_DONE) => u8::try_from(value).is_ok(),
        (MASTER0, field) => {
            u8::try_from(value).is_ok() && BYTE_FIELDS.iter().any(|entry| entry.2 == field)
        }
        _ => false,
    }
}

#[derive(Clone, Default)]
pub(super) struct EspRegi2c {
    command: Option<u32>,
    config: Option<u32>,
    config2: Option<u32>,
    conf0: Option<u32>,
    ulp_bod: [Option<u8>; 7], // regi2c_lp_bias.h:21–55, regi2c_brownout.h:20–22.
    bbpll: [Option<u8>; 11],  // regi2c_bbpll.h:21–175, последний регистр 10.
    sar: [Option<u8>; 8],     // regi2c_saradc.h:21–79, последний регистр 7.
    bias: Option<u8>,         // regi2c_bias.h:20–22, регистр 1.
    dig_reg: [Option<u8>; 14], // regi2c_dig_reg.h:18–60, последний регистр 13.
    locked_hz: Option<u32>,
}

impl EspRegi2c {
    pub(super) fn access(
        &mut self,
        pc: u32,
        addr: u32,
        size: u8,
        value: Option<u32>,
        pll_on: bool,
        resets: &mut EspResetValues,
    ) -> Access {
        if !(MASTER0..ANA_CONFIG2 + u32::BITS / u8::BITS).contains(&addr) {
            return Ok(None);
        }
        let fault = |block| BoardFault::UnsupportedRegister {
            pc,
            addr,
            size,
            write: value.is_some(),
            block,
        };
        if usize::from(size) != size_of::<u32>() {
            return Err(fault("RTC REGI2C: нужен словный доступ"));
        }
        let unknown_reset = || fault("RTC REGI2C: состояние холодного сброса не задокументировано");
        let result = match (addr, value) {
            (ANA_CONFIG, Some(word)) => {
                // ROM rom_chip_i2c_readReg_org пишет полную инверсию маски
                // ведомого. Остальные записанные биты хранятся для RMW.
                self.config = Some(word);
                0
            }
            (ANA_CONFIG2, Some(word)) => {
                // ROM rom_get_i2c_hostid сохраняет прочитанные старшие биты.
                self.config2 = Some(word);
                0
            }
            (ANA_CONFIG, None) => match self.config {
                Some(word) => word,
                None => resets
                    .read(addr, "ANA_CONFIG_REG")?
                    .ok_or_else(unknown_reset)?,
            },
            (ANA_CONFIG2, None) => match self.config2 {
                Some(word) => word,
                None => resets
                    .read(addr, "ANA_CONFIG2_REG")?
                    .ok_or_else(unknown_reset)?,
            },
            (ANA_CONF0, Some(word)) if word & !(STOP_HIGH | STOP_LOW | CAL_DONE) == 0 => {
                let stop = word & (STOP_HIGH | STOP_LOW);
                if stop == STOP_HIGH | STOP_LOW {
                    return Err(fault(
                        "RTC REGI2C: одновременно заданы два уровня остановки BBPLL",
                    ));
                }
                self.conf0 = Some(stop);
                if pll_on {
                    self.calibrate();
                }
                0
            }
            (ANA_CONF0, None) => {
                let stop = match self.conf0 {
                    Some(word) => word,
                    None => resets
                        .read(addr, BBPLL_STOP_FIELD)?
                        .ok_or_else(unknown_reset)?,
                };
                stop | if self.locked_hz.is_some() {
                    CAL_DONE
                } else {
                    0
                }
            }
            (MASTER0, Some(word)) if word & !COMMAND == 0 && word & START != 0 => {
                let slave = word as u8;
                let reg = (word >> REGISTER_SHIFT) as u8;
                // Таблица rom_get_i2c_read_mask, ROM rev. 3, 0x3ff1a134.
                let read_mask = match slave {
                    ULP_BOD => 1 << 22,
                    BBPLL => 1 << 17,
                    SAR => 1 << 18,
                    BIAS => 1 << 13,
                    DIG_REG => 1 << 14,
                    _ => return Err(fault("RTC REGI2C: неизвестный ведомый MASTER0")),
                };
                if slave == BBPLL && !pll_on {
                    return Err(fault(
                        "RTC REGI2C: питание BBPLL или её интерфейса отключено",
                    ));
                }
                let reading = word & WRITE == 0;
                // ROM выбирает маршрут перед чтением. Для записи BBPLL
                // SDK также открывает окно настройки через
                // clk_ll_bbpll_calibration_start (clk_tree_ll.h:400–404).
                // rtc_init.c, calibrate_ocode: после чтения ULP CPU
                // возвращается на PLL через это окно, не меняя маску чтения.
                let needs_route = reading || slave == BBPLL && self.conf0 != Some(STOP_LOW);
                if needs_route && self.config.ok_or_else(unknown_reset)? & read_mask != 0 {
                    return Err(fault("RTC REGI2C: ведомый отключён"));
                }
                if reading && (slave, reg) == (ULP_BOD, 3) {
                    // rtc_init.c, calibrate_ocode: SDK читает два флага.
                    // При заданном профиле это наблюдение, без вычисления
                    // аналоговой калибровки или её длительности.
                    let data = resets.read(addr, ULP_DONE)?.ok_or_else(|| {
                        fault("RTC REGI2C: аналоговый результат калибровки неизвестен")
                    })?;
                    self.command = Some((word & !(DATA | BUSY)) | (data << DATA_SHIFT));
                    return Ok(Some(0));
                }
                let cell = match (slave, reg) {
                    // Эти поля возвращают результат аналоговой калибровки.
                    (BBPLL, BBPLL_STATUS) | (ULP_BOD, 3 | 4) | (DIG_REG, 9) => {
                        return Err(fault(
                            "RTC REGI2C: аналоговый результат калибровки неизвестен",
                        ));
                    }
                    (BBPLL, 0..=6 | 9..=10) => &mut self.bbpll[usize::from(reg)],
                    (ULP_BOD, 0 | 5 | 6) => &mut self.ulp_bod[usize::from(reg)],
                    (SAR, 0..=7) => &mut self.sar[usize::from(reg)],
                    (BIAS, 1) => &mut self.bias,
                    (DIG_REG, 4..=7 | 13) => &mut self.dig_reg[usize::from(reg)],
                    _ => return Err(fault("RTC REGI2C: неизвестный регистр ведомого")),
                };
                let data = if reading {
                    let observed = if cell.is_some() {
                        *cell
                    } else if let Some(&(_, _, field)) = BYTE_FIELDS
                        .iter()
                        .find(|entry| (entry.0, entry.1) == (slave, reg))
                    {
                        resets.read(addr, field)?.map(|word| word as u8)
                    } else {
                        None
                    };
                    observed.ok_or_else(|| {
                        fault("RTC REGI2C: байт ведомого не записан, сброс неизвестен")
                    })?
                } else {
                    let data = (word >> DATA_SHIFT) as u8;
                    *cell = Some(data);
                    if slave == BBPLL {
                        self.locked_hz = None;
                        self.calibrate();
                    }
                    data
                };
                // ROM опрашивает BUSY и извлекает DATA из того же слова.
                // На цифровой границе транзакция завершается атомарно.
                self.command = Some((word & !(DATA | BUSY)) | (u32::from(data) << DATA_SHIFT));
                0
            }
            (MASTER0, None) => match self.command {
                Some(word) => word,
                None => resets
                    .read(addr, "I2C_MST0_COMMAND")?
                    .ok_or_else(unknown_reset)?,
            },
            (MASTER1, _) => return Err(fault("RTC REGI2C: MASTER1 не моделируется")),
            _ => return Err(fault("RTC REGI2C: неизвестный регистр или режим")),
        };
        Ok(Some(result))
    }

    pub(super) fn power_down(&mut self, interface_off: bool) {
        self.locked_hz = None;
        if interface_off {
            // При выключении интерфейса удержание его аналоговых байтов не задано.
            self.bbpll.fill(None);
        }
    }

    pub(super) fn check_pll_selection(&self, hz: u32) -> Result<(), BoardFault> {
        if self.locked_hz != Some(hz) {
            return Err(BoardFault::Unsupported(format!(
                "RTC REGI2C: нет подтверждённого цифрового захвата BBPLL на {hz} Гц"
            )));
        }
        Ok(())
    }

    pub(super) fn may_change_pll(addr: u32, word: u32) -> bool {
        addr == ANA_CONF0 || (addr == MASTER0 && word & WRITE != 0 && word as u8 == BBPLL)
    }

    fn calibrate(&mut self) {
        if self.conf0 != Some(STOP_LOW) {
            return;
        }
        // ESP-IDF esp_hal_clock/esp32c3/include/hal/clk_tree_ll.h:319–395.
        // Рецепты BBPLL при XTAL 40 МГц. DR1/DR3 и VCO_DBIAS в SDK
        // меняются маской, остальные биты этих байтов сохраняются.
        // Цифровое завершение означает применение полного рецепта SDK.
        let [
            Some(reference),
            Some(divider),
            Some(mode),
            Some(dr),
            Some(current),
        ] = self.bbpll[2..7]
        else {
            return;
        };
        if reference != 0x50
            || dr & 0x77 != 0
            || current != 0x63
            || self.bbpll[9].map(|byte| byte & 0x03) != Some(2)
        {
            return;
        }
        self.locked_hz = match (mode, divider) {
            (0x69, 4) => Some(PLL_320_HZ),
            (0x6b, 8) => Some(PLL_480_HZ),
            _ => None,
        };
    }
}
