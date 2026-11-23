//! Запуск ngspice (внешняя программа, лицензия BSD-3-Clause для основной части) в пакетном
//! режиме для перекрёстной сверки. Код ngspice в наш не переносится: мы передаём нетлист на
//! стандартный вход `ngspice -b` и разбираем напечатанные таблицы. Временных файлов нет.
//!
//! Где искать программу: переменная окружения `NGSPICE`, затем
//! `assets/third_party/ngspice/bin/ngspice` относительно каталога `sim/` (от исходников крейта
//! и от текущего каталога).

use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

/// Найти исполняемый файл ngspice.
pub fn find_ngspice() -> Option<PathBuf> {
    if let Ok(p) = std::env::var("NGSPICE") {
        let p = PathBuf::from(p);
        if p.is_file() {
            return Some(p);
        }
    }
    let rel = Path::new("assets/third_party/ngspice/bin/ngspice");
    let candidates = [
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../..")
            .join(rel),
        rel.to_path_buf(),
        Path::new("..").join(rel),
        Path::new("../..").join(rel),
    ];
    candidates.into_iter().find(|p| p.is_file())
}

/// Таблица результатов ngspice: имена столбцов и значения.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct NgTable {
    pub columns: Vec<String>,
    pub rows: Vec<Vec<f64>>,
}

impl NgTable {
    pub fn column(&self, name: &str) -> Option<Vec<f64>> {
        let name = name.to_lowercase();
        let k = self.columns.iter().position(|c| *c == name)?;
        Some(self.rows.iter().map(|r| r[k]).collect())
    }
}

/// Анализ для запуска.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum NgAnalysis {
    Op,
    /// tstep, tstop, начало вывода, наибольший шаг, uic.
    Tran {
        tstep: f64,
        tstop: f64,
        tstart: f64,
        tmax: f64,
        uic: bool,
    },
}

/// Убрать из нетлиста директивы анализа и `.end`, заменить `EXT v` у источников на `DC v`
/// (ngspice не знает внешних источников; при постоянном значении это одно и то же).
pub fn prepare_netlist(netlist: &str) -> String {
    let mut out = String::new();
    for (k, line) in netlist.lines().enumerate() {
        let l = line.trim().to_lowercase();
        if k > 0 && (l.starts_with(".tran") || l.starts_with(".op") || l.starts_with(".end")) {
            continue;
        }
        if k > 0 && (l.starts_with('v') || l.starts_with('i')) && l.contains(" ext ") {
            out.push_str(
                &line
                    .replacen(" EXT ", " DC ", 1)
                    .replacen(" ext ", " dc ", 1),
            );
        } else {
            out.push_str(line);
        }
        out.push('\n');
    }
    out
}

/// Запустить ngspice и напечатать сигналы `signals`.
pub fn run(
    ngspice: &Path,
    netlist: &str,
    analysis: NgAnalysis,
    signals: &[&str],
) -> Result<NgTable, String> {
    let mut input = prepare_netlist(netlist);
    input.push_str(".control\nset width=10000\nset height=10000000\nset noaskquit\n");
    match analysis {
        NgAnalysis::Op => input.push_str("op\n"),
        NgAnalysis::Tran {
            tstep,
            tstop,
            tstart,
            tmax,
            uic,
        } => input.push_str(&format!(
            "tran {tstep:e} {tstop:e} {tstart:e} {tmax:e}{}\n",
            if uic { " uic" } else { "" }
        )),
    }
    input.push_str(&format!("print {}\n.endc\n.end\n", signals.join(" ")));
    let mut child = Command::new(ngspice)
        .arg("-b")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|e| format!("не удалось запустить ngspice: {e}"))?;
    child
        .stdin
        .take()
        .ok_or("нет stdin у ngspice")?
        .write_all(input.as_bytes())
        .map_err(|e| format!("ошибка записи в ngspice: {e}"))?;
    let out = child
        .wait_with_output()
        .map_err(|e| format!("ngspice: {e}"))?;
    let text = String::from_utf8_lossy(&out.stdout);
    match analysis {
        NgAnalysis::Op => parse_op(&text, signals),
        NgAnalysis::Tran { .. } => parse_table(&text),
    }
    .map_err(|e| {
        let err = String::from_utf8_lossy(&out.stderr);
        format!(
            "{e}; stderr ngspice: {}",
            err.lines().take(5).collect::<Vec<_>>().join(" | ")
        )
    })
}

fn parse_op(text: &str, signals: &[&str]) -> Result<NgTable, String> {
    let mut row = Vec::new();
    for s in signals {
        let s = s.to_lowercase();
        let v = text
            .lines()
            .find_map(|l| {
                let (name, val) = l.split_once('=')?;
                (name.trim().to_lowercase() == s)
                    .then(|| val.trim().parse::<f64>().ok())
                    .flatten()
            })
            .ok_or_else(|| format!("ngspice не напечатал {s}"))?;
        row.push(v);
    }
    Ok(NgTable {
        columns: signals.iter().map(|s| s.to_lowercase()).collect(),
        rows: vec![row],
    })
}

fn parse_table(text: &str) -> Result<NgTable, String> {
    let mut columns: Vec<String> = Vec::new();
    let mut rows = Vec::new();
    for line in text.lines() {
        let toks: Vec<&str> = line.split_whitespace().collect();
        if toks.first() == Some(&"Index") {
            columns = toks[1..].iter().map(|s| s.to_lowercase()).collect();
            continue;
        }
        if columns.is_empty()
            || toks.len() != columns.len() + 1
            || toks[0].parse::<usize>().is_err()
        {
            continue;
        }
        let vals: Option<Vec<f64>> = toks[1..].iter().map(|t| t.parse().ok()).collect();
        if let Some(v) = vals {
            rows.push(v);
        }
    }
    if rows.is_empty() {
        return Err("в выводе ngspice нет таблицы".into());
    }
    Ok(NgTable { columns, rows })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prepare_replaces_ext_and_drops_analyses() {
        let n = prepare_netlist("t\nVb a 0 EXT 4\nR1 a 0 1\n.tran 1u 1m\n.end\n");
        assert!(n.contains("Vb a 0 DC 4"));
        assert!(!n.contains(".tran") && !n.contains(".end"));
    }

    #[test]
    fn parses_tables() {
        let t = parse_table(
            "junk\nIndex   time            v(out)\n-----\n0\t0.000000e+00\t1.0e-3\t\n1\t1.0e-4\t2.0\n",
        )
        .unwrap();
        assert_eq!(t.columns, vec!["time", "v(out)"]);
        assert_eq!(t.column("v(out)").unwrap(), vec![1.0e-3, 2.0]);
        let o = parse_op("v(a) = 6.5e-01\ni(v1) = -4.3e-03\n", &["v(a)", "i(v1)"]).unwrap();
        assert_eq!(o.rows[0], vec![0.65, -4.3e-3]);
    }
}
