//! Числа в стиле SPICE: инженерные суффиксы и хвостовые единицы измерения.
//!
//! Суффиксы (регистр не важен): `f`=1e-15, `p`=1e-12, `n`=1e-9, `u`/`µ`=1e-6, `m`=1e-3 (мИлли!),
//! `k`=1e3, `meg`=1e6, `g`=1e9, `t`=1e12, `mil`=25.4e-6. Всё, что идёт после суффикса, —
//! единица измерения и игнорируется: `10uF`, `5V`, `2.2kOhm`.
//!
//! Классическая ловушка SPICE сохранена намеренно, ради совместимости нетлистов:
//! `1F` — это 1 фемто-, а не 1 фарад; `1M` — 1 милли-, а не мега (мега — `1MEG`).

/// Разобрать число SPICE. `None`, если строка не начинается с числа.
pub fn parse_value(s: &str) -> Option<f64> {
    let s = s.trim();
    let bytes = s.as_bytes();
    let mut i = 0;
    if i < bytes.len() && (bytes[i] == b'+' || bytes[i] == b'-') {
        i += 1;
    }
    let mant_start = i;
    while i < bytes.len() && bytes[i].is_ascii_digit() {
        i += 1;
    }
    if i < bytes.len() && bytes[i] == b'.' {
        i += 1;
        while i < bytes.len() && bytes[i].is_ascii_digit() {
            i += 1;
        }
    }
    // Мантисса должна содержать хотя бы одну цифру.
    if !s[mant_start..i].bytes().any(|b| b.is_ascii_digit()) {
        return None;
    }
    let mant_end = i;
    // Порядок: e/E, затем необязательный знак и обязательная цифра.
    let mut exp: i32 = 0;
    if i < bytes.len() && (bytes[i] == b'e' || bytes[i] == b'E') {
        let mut j = i + 1;
        if j < bytes.len() && (bytes[j] == b'+' || bytes[j] == b'-') {
            j += 1;
        }
        if j < bytes.len() && bytes[j].is_ascii_digit() {
            while j < bytes.len() && bytes[j].is_ascii_digit() {
                j += 1;
            }
            exp = s[i + 1..j].parse().ok()?;
            i = j;
        }
    }
    let rest = s[i..].to_lowercase();
    // Десятичный порядок суффикса складываем с порядком числа и разбираем одной строкой:
    // так «10u» даёт ровно 1e-5 (умножение 10·1e-6 дало бы 9.999…e-6).
    let (suffix_exp, extra) = if rest.starts_with("meg") {
        (6, 1.0)
    } else if rest.starts_with("mil") {
        (-6, 25.4)
    } else {
        match rest.chars().next() {
            Some('f') => (-15, 1.0),
            Some('p') => (-12, 1.0),
            Some('n') => (-9, 1.0),
            Some('u') | Some('µ') | Some('μ') => (-6, 1.0),
            Some('m') => (-3, 1.0),
            Some('k') => (3, 1.0),
            Some('g') => (9, 1.0),
            Some('t') => (12, 1.0),
            _ => (0, 1.0),
        }
    };
    let number: f64 = format!("{}e{}", &s[..mant_end], exp + suffix_exp)
        .parse()
        .ok()?;
    Some(number * extra)
}

#[cfg(test)]
mod tests {
    use super::parse_value;

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() <= 1e-12 * b.abs().max(1e-30)
    }

    #[test]
    fn suffixes_and_units() {
        let cases = [
            ("1k", 1e3),
            ("2.2K", 2.2e3),
            ("10uF", 10e-6),
            ("5V", 5.0),
            ("1meg", 1e6),
            ("1MEG", 1e6),
            ("1m", 1e-3),
            ("3mA", 3e-3),
            ("1e-14", 1e-14),
            ("1.5e3k", 1.5e6),
            ("-4.7n", -4.7e-9),
            ("100p", 100e-12),
            ("2g", 2e9),
            ("1t", 1e12),
            ("10mil", 254e-6),
            (".5", 0.5),
            ("1F", 1e-15),
            ("1e", 1.0),
        ];
        for (s, v) in cases {
            let got = parse_value(s).unwrap_or_else(|| panic!("не разобрано: {s}"));
            assert!(close(got, v), "{s}: {got} != {v}");
        }
    }

    #[test]
    fn rejects_non_numbers() {
        for s in ["", "abc", "-", ".", "e5", "pulse"] {
            assert!(parse_value(s).is_none(), "{s} не число");
        }
    }
}
