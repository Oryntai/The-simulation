use sim_boards::esp_adc::{
    EspAdcAttenuation as Atten, EspAdcCalibration, EspAdcError, EspAdcPoint,
};
use sim_boards::esp_memory::EspMemory;

// EFUSE_RD_WR_DIS_REG (0x2c)..=EFUSE_RD_SYS_PART2_DATA7_REG (0x178), efuse_reg.h.
const READBACK_WORDS: usize = (0x178 - 0x2c) / size_of::<u32>() + 1;
// EFUSE_RD_SYS_PART1_DATA4_REG..=DATA7_REG (0x6c..=0x78), efuse_reg.h.
const DATA4: usize = (0x6c - 0x2c) / size_of::<u32>();
// Синтетический BLOCK2 из `espefuse v5.7.0 --chip esp32c3 --virt ... dump`.
// calib-info: INIT=[0, 683, 1023, 289], CAL_VOL=[341, 894, 511, 1023].
// Те же четыре слова прочитала прошивка ESP-IDF v6.1.1 в QEMU.
const FACTORY_WORDS: [u32; 4] = [0xc000_0001, 0x5487_ffaa, 0xfdff_df95, 0xf];
// efuse_reg.h: в DATA4 выставлена версия 1, все поля смещения нулевые.
const ZERO_FIELDS: [u32; 4] = [1, 0, 0, 0];

fn readback(block2_tail: [u32; 4]) -> [u32; READBACK_WORDS] {
    let mut words = [0; READBACK_WORDS];
    words[DATA4..DATA4 + block2_tail.len()].copy_from_slice(&block2_tail);
    words
}

#[test]
fn factory_fields_match_idf_and_espefuse() {
    let words = readback(FACTORY_WORDS);
    EspMemory::default().install_efuse_readback(&words).unwrap();
    let adc = EspAdcCalibration::from_efuse_readback(Some(&words)).unwrap();
    // esp_efuse_rtc_calib_get_init_code и esp_efuse_rtc_calib_get_cal_voltage,
    // UART контрольной программы ESP-IDF v6.1.1 в QEMU.
    for (atten, init_code, reference_raw, reference_mv) in [
        (Atten::Db0, 1000, 2341, 400),
        (Atten::Db2_5, 1683, 1618, 550),
        (Atten::Db6, 2023, 2511, 750),
        (Atten::Db12, 1289, 1489, 1370),
    ] {
        assert_eq!(
            adc.point(atten),
            EspAdcPoint {
                init_code,
                reference_raw,
                reference_mv
            }
        );
    }
}

#[test]
fn every_raw_code_matches_idf_curve() {
    // ESP-IDF v6.1.1, adc_cali_raw_to_voltage, qemu-system-riscv32 -M esp32c3.
    // Для каждого ATTEN: выход на заводской точке, выход при raw=4095 и
    // FNV-1a 64 всех 4096 результатов i32 LE в порядке raw.
    for (block, expected) in [
        (
            FACTORY_WORDS,
            [
                (398, 694, 0x9068_4fb1_1d1c_2797),
                (547, 1374, 0xe439_020c_b9b7_2ae3),
                (749, 1213, 0x9fe2_bf5c_93ca_06ee),
                (1367, 3464, 0x85c3_b589_eb3e_70e9),
            ],
        ),
        (
            ZERO_FIELDS,
            [
                (398, 811, 0xe2a4_c09c_db73_7cc2),
                (547, 1115, 0x74f3_d4eb_0e39_3acd),
                (750, 1515, 0x681b_7b07_2219_b16a),
                (1367, 2727, 0x006d_2281_f36a_d37d),
            ],
        ),
    ] {
        let adc = EspAdcCalibration::from_efuse_readback(Some(&readback(block))).unwrap();
        for (atten, (at_reference, at_full_scale, idf_hash)) in
            [Atten::Db0, Atten::Db2_5, Atten::Db6, Atten::Db12]
                .into_iter()
                .zip(expected)
        {
            assert_eq!(adc.raw_to_mv(atten, 0), Ok(0));
            assert_eq!(
                adc.raw_to_mv(atten, u32::from(adc.point(atten).reference_raw)),
                Ok(at_reference)
            );
            assert_eq!(adc.raw_to_mv(atten, 4095), Ok(at_full_scale));
            // Начальное значение и множитель FNV-1a 64 по спецификации FNV.
            let mut hash = 14_695_981_039_346_656_037_u64;
            for raw in 0..=4095 {
                for byte in adc.raw_to_mv(atten, raw).unwrap().to_le_bytes() {
                    hash = (hash ^ u64::from(byte)).wrapping_mul(1_099_511_628_211);
                }
            }
            assert_eq!(hash, idf_hash, "BLOCK2 {block:08x?}, ослабление {atten:?}");
        }
    }
}

#[test]
fn missing_unsupported_and_truncated_profiles_are_rejected() {
    assert_eq!(
        EspAdcCalibration::from_efuse_readback(None),
        Err(EspAdcError::NotAnalyzed {
            block_version: None
        }),
    );
    // В IDF версия 0 и неподдержанная версия BLOCK2 дают ESP_ERR_NOT_SUPPORTED.
    for version in [0, 2, 3] {
        let mut words = readback(FACTORY_WORDS);
        // efuse_reg.h, EFUSE_BLK_VERSION_MAJOR[1:0]. Оставляем остальные поля.
        words[DATA4] = (words[DATA4] & !3) | u32::from(version);
        assert_eq!(
            EspAdcCalibration::from_efuse_readback(Some(&words)),
            Err(EspAdcError::NotAnalyzed {
                block_version: Some(version)
            }),
        );
    }
    let words = readback(FACTORY_WORDS);
    let mut oversized = words.to_vec();
    oversized.push(0);
    for malformed in [&[][..], &words[..READBACK_WORDS - 1], &oversized] {
        assert_eq!(
            EspAdcCalibration::from_efuse_readback(Some(malformed)),
            Err(EspAdcError::InvalidReadbackLength {
                expected: READBACK_WORDS,
                actual: malformed.len()
            }),
        );
    }
}

#[test]
fn version_one_accepts_both_encodings_of_zero_offset() {
    // efuse_reg.h: версия 1 в DATA4 и знак CAL_VOL_ATTEN0_1 в DATA6[5].
    let zero_fields = readback(ZERO_FIELDS);
    let signed_zero = readback([1, 0, 1 << 5, 0]);
    let zero = EspAdcCalibration::from_efuse_readback(Some(&zero_fields)).unwrap();
    let signed = EspAdcCalibration::from_efuse_readback(Some(&signed_zero)).unwrap();
    let expected = EspAdcPoint {
        init_code: 1000,
        reference_raw: 2000,
        reference_mv: 400,
    };
    assert_eq!(zero.point(Atten::Db0), expected);
    assert_eq!(signed, zero);
}

#[test]
fn attenuation_limits_and_total_error_match_datasheet() {
    // ESP32-C3 Series Datasheet v2.5 §5.5, таблица 5-6, все четыре строки.
    for (encoding, atten, max_mv, error_mv) in [
        (0, Atten::Db0, 750, 10),
        (1, Atten::Db2_5, 1050, 10),
        (2, Atten::Db6, 1300, 10),
        (3, Atten::Db12, 2500, 35),
    ] {
        let limits = atten.limits();
        assert_eq!(Atten::try_from(encoding), Ok(atten));
        assert_eq!(limits.min_input_mv, 0);
        assert_eq!(limits.max_input_mv, max_mv);
        assert_eq!(limits.max_abs_error_mv, error_mv);
        assert_eq!(atten.validate_input_mv(0.0), Ok(()));
        assert_eq!(atten.validate_input_mv(f64::from(max_mv)), Ok(()));
        for edge in [0.0_f64.next_down(), f64::from(max_mv).next_up(), f64::MAX] {
            assert_eq!(
                atten.validate_input_mv(edge),
                Err(EspAdcError::OutsideEffectiveRange(atten))
            );
        }
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert_eq!(
                atten.validate_input_mv(invalid),
                Err(EspAdcError::NonFiniteInput)
            );
        }
    }
}

#[test]
fn invalid_attenuation_and_raw_codes_are_rejected() {
    let adc = EspAdcCalibration::from_efuse_readback(Some(&readback(FACTORY_WORDS))).unwrap();
    // adc_types.h: последний код ослабления 3. TRM §34.2.3.2: raw не больше 4095.
    for invalid in [4, u8::MAX] {
        assert_eq!(
            Atten::try_from(invalid),
            Err(EspAdcError::InvalidAttenuation(invalid))
        );
    }
    for raw in [4096, u32::MAX] {
        assert_eq!(
            adc.raw_to_mv(Atten::Db0, raw),
            Err(EspAdcError::InvalidRaw(raw))
        );
    }
}
