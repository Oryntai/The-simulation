//! SHA-256 через регистры ESP32-C3, TRM гл. 21, режим работы без DMA.

use super::Access;
use crate::{
    BoardFault,
    esp_image::{SHA256_INITIAL, sha256_compress},
};

// ESP-IDF soc/esp32c3/{include/soc/hwcrypto_reg.h,register/soc/reg_base.h}.
const BASE: u32 = 0x6003_b000;
const END: u32 = BASE + 0x1000;
const MODE: u32 = BASE;
const START: u32 = BASE + 0x10;
const CONTINUE: u32 = BASE + 0x14;
const BUSY: u32 = BASE + 0x18;
const H: u32 = BASE + 0x40;
const M: u32 = BASE + 0x80;
const SHA256: u32 = 2;
const TRIGGER: u32 = 1;
const WORD: u32 = size_of::<u32>() as u32;
// FIPS 180-4, §§5.2.1, 5.3.3: 16 слов в блоке, 8 слов состояния SHA-256.
const STATE_WORDS: usize = SHA256_INITIAL.len();
const TEXT_WORDS: usize = 16;

#[derive(Clone, Default)]
pub(super) struct EspSha {
    mode: u32, // SHA_MODE сбрасывается в 0, режим SHA-1.
    text: [u32; TEXT_WORDS],
    text_valid: u16,
    state: [u32; STATE_WORDS],
    state_valid: u8,
}

impl EspSha {
    pub(super) fn access(
        &mut self,
        pc: u32,
        addr: u32,
        size: u8,
        value: Option<u32>,
        enabled: bool,
    ) -> Access {
        if !(BASE..END).contains(&addr) {
            return Ok(None);
        }
        let unsupported = || BoardFault::UnsupportedRegister {
            pc,
            addr,
            size,
            write: value.is_some(),
            block: "SHA ESP32-C3",
        };
        if !enabled || size != WORD as u8 {
            return Err(unsupported());
        }
        crate::memory::check_data_alignment(pc, addr, size)?;
        let result = match (addr, value) {
            (MODE, None) => self.mode,
            (MODE, Some(SHA256)) => {
                self.mode = SHA256;
                0
            }
            (BUSY, None) => 0, // Сжатие одного блока атомарно.
            (START | CONTINUE, Some(TRIGGER))
                if self.mode == SHA256 && self.text_valid == u16::MAX =>
            {
                if addr == START {
                    self.state = SHA256_INITIAL;
                } else if self.state_valid != u8::MAX {
                    return Err(unsupported());
                }
                let mut block = [0; TEXT_WORDS * size_of::<u32>()];
                // ESP-IDF esp32c3/include/hal/sha_ll.h: загрузка байтов
                // сообщения словами CPU. Алгоритм FIPS читает слова big endian.
                for (bytes, word) in block.chunks_exact_mut(WORD as usize).zip(self.text) {
                    bytes.copy_from_slice(&word.to_le_bytes());
                }
                sha256_compress(&mut self.state, &block);
                self.state_valid = u8::MAX;
                0
            }
            _ if (M..M + TEXT_WORDS as u32 * WORD).contains(&addr) => {
                let index = ((addr - M) / WORD) as usize;
                if let Some(bits) = value {
                    self.text[index] = bits;
                    self.text_valid |= 1 << index;
                    0
                } else if self.text_valid & (1 << index) != 0 {
                    self.text[index]
                } else {
                    return Err(unsupported());
                }
            }
            _ if (H..H + STATE_WORDS as u32 * WORD).contains(&addr) && self.mode == SHA256 => {
                let index = ((addr - H) / WORD) as usize;
                if let Some(bits) = value {
                    self.state[index] = bits.swap_bytes();
                    self.state_valid |= 1 << index;
                    0
                } else if self.state_valid & (1 << index) != 0 {
                    // SHA_Hn отдаёт байты дайджеста при сохранении LE-слов CPU.
                    self.state[index].swap_bytes()
                } else {
                    return Err(unsupported());
                }
            }
            _ => return Err(unsupported()),
        };
        Ok(Some(result))
    }
}
