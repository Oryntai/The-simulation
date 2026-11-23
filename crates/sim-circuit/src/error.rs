//! Ошибки построения схемы и расчёта (сообщения — по-русски, с подсказкой, что делать).

use std::fmt;

use crate::netlist::ParseError;

#[derive(Clone, Debug, PartialEq)]
pub enum SimError {
    /// Ошибка синтаксиса нетлиста.
    Parse(ParseError),
    /// Ошибка в элементе: неизвестная модель, неверные параметры.
    Element {
        name: String,
        line: usize,
        message: String,
    },
    /// Нет такого узла.
    UnknownNode(String),
    /// Нет такого элемента.
    UnknownElement(String),
    /// Операция не подходит к элементу (например, `set_source` для не-EXT источника).
    WrongElement { name: String, message: String },
    /// Вырожденная матрица: плавающий узел, контур из источников напряжения/индуктивностей.
    Singular(String),
    /// Метод Ньютона не сошёлся даже с дроблением шага / gmin / шагами по источникам.
    NoConvergence { time: f64, message: String },
    /// Неверные аргументы (шаг ≤ 0 и т. п.) или нет нужной директивы анализа.
    Invalid(String),
}

impl fmt::Display for SimError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SimError::Parse(e) => write!(f, "{e}"),
            SimError::Element {
                name,
                line,
                message,
            } => write!(f, "строка {line}, элемент «{name}»: {message}"),
            SimError::UnknownNode(n) => write!(f, "узел «{n}» не найден в схеме"),
            SimError::UnknownElement(n) => write!(f, "элемент «{n}» не найден в схеме"),
            SimError::WrongElement { name, message } => write!(f, "элемент «{name}»: {message}"),
            SimError::Singular(m) => write!(f, "матрица схемы вырождена: {m}"),
            SimError::NoConvergence { time, message } => {
                write!(f, "расчёт не сошёлся при t = {time:.6e} с: {message}")
            }
            SimError::Invalid(m) => write!(f, "{m}"),
        }
    }
}

impl std::error::Error for SimError {}

impl From<ParseError> for SimError {
    fn from(e: ParseError) -> Self {
        SimError::Parse(e)
    }
}
