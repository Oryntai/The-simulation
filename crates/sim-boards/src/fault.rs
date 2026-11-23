//! Ошибки эмуляции платы. Главное правило: всё, что эмулятор не понял или что на
//! настоящем чипе не заработало бы, — явная ошибка, а не молчаливый ноль.

use std::fmt;

/// Почему прошивка не может считаться работающей.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BoardFault {
    /// Образ не разобран или не подходит этой плате.
    BadImage(String),
    /// Запуск без загруженной прошивки.
    NoImage,
    /// Плата обесточена.
    Unpowered,
    /// Исполнение прошивки завершилось ошибкой.
    Execution(String),
    /// Инструкция, которой нет в архитектуре или которую ядро не реализует.
    UnknownInstruction {
        pc: u32,
        raw: u32,
        isa: &'static str,
    },
    /// Обращение к регистру периферии, которого эмулятор не знает.
    UnsupportedRegister {
        pc: u32,
        addr: u32,
        write: bool,
        size: u8,
        block: &'static str,
    },
    /// Адрес вне карты памяти (на настоящем чипе — исключение шины).
    BusError { pc: u32, addr: u32, write: bool },
    /// Запись в память только для чтения (флеш, ПЗУ).
    WriteToReadOnly { pc: u32, addr: u32 },
    /// Невыровненный доступ там, где архитектура его запрещает.
    Unaligned { pc: u32, addr: u32, size: u8 },
    /// Переход в недопустимое состояние ядра (например, ARM-режим на Cortex-M).
    InvalidState {
        pc: u32,
        target: u32,
        why: &'static str,
    },
    /// Настройка, при которой настоящий чип не заработает или выйдет за паспорт.
    Misconfigured(String),
    /// Возможность чипа существует, но эмулятор её не моделирует.
    Unsupported(String),
    /// Останов по точке останова в прошивке (BKPT, BREAK, EBREAK).
    Breakpoint { pc: u32 },
    /// Ядро встало: ошибка внутри обработчика, неверный возврат из исключения.
    Lockup { pc: u32, why: String },
}

impl fmt::Display for BoardFault {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BoardFault::BadImage(s) => write!(f, "образ прошивки не принят: {s}"),
            BoardFault::Unpowered => write!(f, "плата обесточена"),
            BoardFault::Execution(s) => write!(f, "сбой исполнения прошивки: {s}"),
            BoardFault::NoImage => write!(f, "прошивка не загружена"),
            BoardFault::UnknownInstruction { pc, raw, isa } => write!(
                f,
                "неизвестная инструкция {raw:#010x} по адресу {pc:#010x} (набор {isa})"
            ),
            BoardFault::UnsupportedRegister {
                pc,
                addr,
                write,
                size,
                block,
            } => write!(
                f,
                "{} неподдержанного регистра {block} {addr:#010x} ({size} байт) по адресу {pc:#010x}",
                if *write {
                    "запись"
                } else {
                    "чтение"
                }
            ),
            BoardFault::BusError { pc, addr, write } => write!(
                f,
                "{} вне карты памяти: {addr:#010x} по адресу {pc:#010x}",
                if *write {
                    "запись"
                } else {
                    "чтение"
                }
            ),
            BoardFault::WriteToReadOnly { pc, addr } => {
                write!(
                    f,
                    "запись в память только для чтения {addr:#010x} по адресу {pc:#010x}"
                )
            }
            BoardFault::Unaligned { pc, addr, size } => write!(
                f,
                "невыровненный доступ {size} байт к {addr:#010x} по адресу {pc:#010x}"
            ),
            BoardFault::InvalidState { pc, target, why } => {
                write!(
                    f,
                    "недопустимый переход на {target:#010x} с {pc:#010x}: {why}"
                )
            }
            BoardFault::Misconfigured(s) => write!(f, "ошибка настройки чипа: {s}"),
            BoardFault::Unsupported(s) => write!(f, "не моделируется: {s}"),
            BoardFault::Breakpoint { pc } => write!(f, "точка останова по адресу {pc:#010x}"),
            BoardFault::Lockup { pc, why } => write!(f, "ядро встало на {pc:#010x}: {why}"),
        }
    }
}

impl std::error::Error for BoardFault {}
