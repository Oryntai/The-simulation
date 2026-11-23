//! Результат сверки «симуляция против эталона». Расхождение не прячется: в каждой
//! проверке есть оба числа, ошибка, допуск и вердикт.

use std::fmt;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Tolerance {
    /// Относительная: |sim − ref| ≤ tol·|ref|.
    Relative(f64),
    /// Абсолютная (в единицах величины).
    Absolute(f64),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    Pass,
    Fail,
}

#[derive(Clone, Debug)]
pub struct Check {
    pub id: &'static str,
    pub title: String,
    pub simulated: f64,
    pub reference: f64,
    pub unit: &'static str,
    /// Чем получено число симуляции.
    pub method: String,
    /// Чем получен эталон (формула/источник).
    pub reference_method: String,
    pub tolerance: Tolerance,
    /// Допущения и пояснения.
    pub note: String,
}

impl Check {
    pub fn abs_error(&self) -> f64 {
        (self.simulated - self.reference).abs()
    }

    pub fn rel_error(&self) -> f64 {
        let r = self.reference.abs();
        if r > 0.0 {
            self.abs_error() / r
        } else {
            self.abs_error()
        }
    }

    pub fn passed(&self) -> bool {
        let err = self.abs_error();
        let ok = match self.tolerance {
            Tolerance::Relative(t) => err <= t * self.reference.abs(),
            Tolerance::Absolute(t) => err <= t,
        };
        // Сломанный эталон (∞, NaN) — не «пройдено»: при ∞ допуск тоже становится ∞.
        ok && self.simulated.is_finite() && self.reference.is_finite()
    }

    pub fn verdict(&self) -> Verdict {
        if self.passed() {
            Verdict::Pass
        } else {
            Verdict::Fail
        }
    }

    pub fn tolerance_text(&self) -> String {
        match self.tolerance {
            Tolerance::Relative(t) => format!("≤ {} отн.", fmt_sci(t)),
            Tolerance::Absolute(t) => format!("≤ {} {}", fmt_sci(t), self.unit),
        }
    }

    pub fn error_text(&self) -> String {
        match self.tolerance {
            Tolerance::Relative(_) => format!("{} отн.", fmt_sci(self.rel_error())),
            Tolerance::Absolute(_) => format!("{} {}", fmt_sci(self.abs_error()), self.unit),
        }
    }

    /// Падает с понятным сообщением — для `cargo test`.
    pub fn assert_pass(&self) {
        assert!(self.passed(), "ПРОВЕРКА НЕ ПРОЙДЕНА:\n{self}");
    }
}

pub fn fmt_sci(x: f64) -> String {
    if x == 0.0 {
        "0".into()
    } else if (1e-3..1e4).contains(&x.abs()) {
        format!("{x:.4}")
    } else {
        format!("{x:.2e}")
    }
}

/// Значение с разумным числом значащих цифр.
pub fn fmt_val(x: f64) -> String {
    let a = x.abs();
    if a == 0.0 {
        "0".into()
    } else if a >= 1e4 || a < 1e-3 {
        format!("{x:.6e}")
    } else {
        format!("{x:.8}")
    }
}

impl fmt::Display for Check {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mark = if self.passed() { "OK  " } else { "FAIL" };
        writeln!(f, "[{mark}] {} ({})", self.title, self.id)?;
        writeln!(
            f,
            "       симуляция: {} {}  — {}",
            fmt_val(self.simulated),
            self.unit,
            self.method
        )?;
        writeln!(
            f,
            "       эталон:    {} {}  — {}",
            fmt_val(self.reference),
            self.unit,
            self.reference_method
        )?;
        writeln!(
            f,
            "       ошибка:    {}  (допуск {})",
            self.error_text(),
            self.tolerance_text()
        )?;
        if !self.note.is_empty() {
            writeln!(f, "       допущения: {}", self.note)?;
        }
        Ok(())
    }
}
