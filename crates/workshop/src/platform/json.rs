//! Чтение JSON отчёта и входов интерфейса без дополнительных зависимостей.
use std::collections::BTreeMap;
use std::iter::Peekable;
use std::str::Chars;

#[derive(Debug, PartialEq)]
pub enum Value {
    Null,
    Bool(bool),
    Number(String),
    Text(String),
    Array(Vec<Self>),
    Object(BTreeMap<String, Self>),
}

impl Value {
    pub fn get(&self, key: &str) -> Option<&Self> {
        self.object()?.get(key)
    }
    pub fn object(&self) -> Option<&BTreeMap<String, Self>> {
        if let Self::Object(map) = self {
            Some(map)
        } else {
            None
        }
    }
    pub fn array(&self) -> Option<&[Self]> {
        if let Self::Array(list) = self {
            Some(list)
        } else {
            None
        }
    }
    pub fn str(&self) -> Option<&str> {
        if let Self::Text(s) = self {
            Some(s)
        } else {
            None
        }
    }
    pub fn f64(&self) -> Option<f64> {
        if let Self::Number(s) = self {
            s.parse().ok()
        } else {
            None
        }
    }
    pub fn u64(&self) -> Option<u64> {
        if let Self::Number(s) = self {
            s.parse().ok()
        } else {
            None
        }
    }
}

pub fn parse(source: &str) -> Result<Value, String> {
    let mut parser = Parser(source.chars().peekable());
    let value = parser.value(0).ok_or("неверный JSON")?;
    parser.space();
    if parser.0.next().is_some() {
        return Err("лишние данные после JSON".into());
    }
    Ok(value)
}

struct Parser<'a>(Peekable<Chars<'a>>);

impl Parser<'_> {
    fn space(&mut self) {
        while matches!(self.0.peek(), Some(' ' | '\n' | '\r' | '\t')) {
            self.0.next();
        }
    }

    fn value(&mut self, depth: usize) -> Option<Value> {
        // Ограничение глубины защищает стек при чтении постороннего файла.
        if depth >= 128 {
            return None;
        }
        self.space();
        let start = self.0.next()?;
        Some(match start {
            '"' => Value::Text(self.string()?),
            'n' if self.literal("ull") => Value::Null,
            't' if self.literal("rue") => Value::Bool(true),
            'f' if self.literal("alse") => Value::Bool(false),
            '[' => Value::Array(self.sequence(']', |p| p.value(depth + 1))?),
            '{' => {
                let entries = self.sequence('}', |p| {
                    if p.0.next()? != '"' {
                        return None;
                    }
                    let key = p.string()?;
                    p.space();
                    if p.0.next()? != ':' {
                        return None;
                    }
                    Some((key, p.value(depth + 1)?))
                })?;
                let len = entries.len();
                let map: BTreeMap<_, _> = entries.into_iter().collect();
                if map.len() != len {
                    return None;
                }
                Value::Object(map)
            }
            '-' | '0'..='9' => Value::Number(self.number(start)?),
            _ => return None,
        })
    }

    fn sequence<T>(
        &mut self,
        end: char,
        mut item: impl FnMut(&mut Self) -> Option<T>,
    ) -> Option<Vec<T>> {
        let mut items = Vec::new();
        self.space();
        if self.0.next_if_eq(&end).is_some() {
            return Some(items);
        }
        loop {
            items.push(item(self)?);
            self.space();
            match self.0.next()? {
                c if c == end => return Some(items),
                ',' => self.space(),
                _ => return None,
            }
        }
    }

    fn literal(&mut self, suffix: &str) -> bool {
        suffix.chars().all(|c| self.0.next() == Some(c))
    }

    fn string(&mut self) -> Option<String> {
        let mut text = String::new();
        loop {
            text.push(match self.0.next()? {
                '"' => return Some(text),
                '\\' => match self.0.next()? {
                    '"' => '"',
                    '\\' => '\\',
                    '/' => '/',
                    'b' => '\u{8}',
                    'f' => '\u{c}',
                    'n' => '\n',
                    'r' => '\r',
                    't' => '\t',
                    'u' => {
                        let a = self.hex_quad()?;
                        if (0xd800..=0xdbff).contains(&a) {
                            if !self.literal("\\u") {
                                return None;
                            }
                            let b = self.hex_quad()?;
                            if !(0xdc00..=0xdfff).contains(&b) {
                                return None;
                            }
                            char::from_u32(0x10000 + ((a - 0xd800) << 10) + (b - 0xdc00))?
                        } else {
                            char::from_u32(a)?
                        }
                    }
                    _ => return None,
                },
                c if c >= '\u{20}' => c,
                _ => return None,
            });
        }
    }

    fn hex_quad(&mut self) -> Option<u32> {
        (0..4).try_fold(0, |n, _| Some(n * 16 + self.0.next()?.to_digit(16)?))
    }

    fn digits(&mut self, text: &mut String) -> bool {
        let start = text.len();
        while let Some(digit) = self.0.next_if(char::is_ascii_digit) {
            text.push(digit);
        }
        start != text.len()
    }

    fn number(&mut self, start: char) -> Option<String> {
        let mut text = String::from(start);
        let first = if start == '-' {
            let digit = self.0.next_if(char::is_ascii_digit)?;
            text.push(digit);
            digit
        } else {
            start
        };
        if first != '0' {
            self.digits(&mut text);
        }
        if self.0.next_if_eq(&'.').is_some() {
            text.push('.');
            if !self.digits(&mut text) {
                return None;
            }
        }
        if let Some(exp) = self.0.next_if(|c| matches!(c, 'e' | 'E')) {
            text.push(exp);
            if let Some(sign) = self.0.next_if(|c| matches!(c, '+' | '-')) {
                text.push(sign);
            }
            if !self.digits(&mut text) {
                return None;
            }
        }
        text.parse::<f64>().ok()?.is_finite().then_some(text)
    }
}
