//! Правило платформы: в её коде нет имён конкретных устройств (docs/PLATFORM_SPEC.md, правило 1).
//!
//! * Любой крейт, которого нет в `LEGACY`, обязан быть чистым. Новые крейты чисты по умолчанию.
//! * В старых крейтах счётчик по каждому файлу может только падать. Сниженный счётчик надо
//!   записать: `UPDATE_PLATFORM_BASELINE=1 cargo test -p sim-verify --test platform_boundary`.
//!   Запись отказывается поднимать число и добавлять файл.
//! * Анатомия не устройство: в `sim-human` слова вроде «локоть» допустимы, названия устройств нет.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// Крейты, написанные до платформы. Список может только сокращаться.
const LEGACY: &[&str] = &[
    "sim-actuators",
    "sim-device",
    "sim-em",
    "sim-env",
    "sim-human",
    "sim-mcu",
    "sim-mujoco",
    "sim-scenarios",
    "workshop",
];

/// Код устройств и окна: платформа от них не зависит.
const DEVICE_LAYER: &[&str] = &["sim-scenarios", "workshop"];

/// Основы слов-устройств: совпадает начало слова.
const DEVICE_STEMS: &[&str] = &[
    "экзоскелет", "exoskelet", "ортез", "orthosis", "orthotic", "gantry", "портал", "тележк",
    "segway", "fingerbot",
];
/// Слова целиком: их основы слишком короткие или совпадают с обычными словами.
const DEVICE_WORDS: &[&str] = &[
    "экзо", "exo", "очки", "очков", "очках", "очками", "очкам", "glasses",
];
/// Устойчивые сочетания, слова разделены одним пробелом.
const DEVICE_PHRASES: &[&str] = &[
    "третья рука", "третьей руки", "третьей руке", "третью руку", "third arm",
    "палец кнопка", "пальца кнопки", "finger button", "поворотная камера", "поворотной камеры",
    "pan tilt", "balance bot", "balancing cart", "self balancing",
];
const ANATOMY_STEMS: &[&str] = &["локот", "локт", "elbow"];

fn root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn baseline_file() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/platform_baseline.txt")
}

/// Слова в нижнем регистре: граница — не буква и не цифра, а также переход `aB` в CamelCase.
fn words(text: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut cur = String::new();
    let mut prev_lower = false;
    for ch in text.chars() {
        if !ch.is_alphanumeric() {
            if !cur.is_empty() {
                out.push(std::mem::take(&mut cur));
            }
            prev_lower = false;
            continue;
        }
        if ch.is_uppercase() && prev_lower {
            out.push(std::mem::take(&mut cur));
        }
        prev_lower = ch.is_lowercase();
        cur.extend(ch.to_lowercase());
    }
    if !cur.is_empty() {
        out.push(cur);
    }
    out
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Hits {
    device: usize,
    anatomy: usize,
}

fn scan(text: &str) -> Hits {
    let w = words(text);
    let mut hits = Hits::default();
    for word in &w {
        if DEVICE_WORDS.contains(&word.as_str())
            || DEVICE_STEMS.iter().any(|s| word.starts_with(s))
        {
            hits.device += 1;
        }
        if ANATOMY_STEMS.iter().any(|s| word.starts_with(s)) {
            hits.anatomy += 1;
        }
    }
    for pair in w.windows(2) {
        let phrase = format!("{} {}", pair[0], pair[1]);
        if DEVICE_PHRASES.contains(&phrase.as_str()) {
            hits.device += 1;
        }
    }
    hits
}

fn rust_files(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return;
    };
    for entry in entries.flatten() {
        let path = entry.path();
        if path.is_dir() {
            rust_files(&path, out);
        } else if path.extension().is_some_and(|e| e == "rs") {
            out.push(path);
        }
    }
}

fn crates() -> Vec<String> {
    let mut names: Vec<String> = std::fs::read_dir(root().join("crates"))
        .expect("каталог крейтов")
        .flatten()
        .filter(|e| e.path().join("Cargo.toml").is_file())
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();
    names
}

/// Счётчик по файлам `src` одного крейта. В `sim-human` анатомия не считается.
fn crate_hits(name: &str) -> BTreeMap<String, usize> {
    let base = root().join("crates").join(name);
    let mut files = Vec::new();
    rust_files(&base.join("src"), &mut files);
    let mut out = BTreeMap::new();
    for file in files {
        let text = std::fs::read_to_string(&file).expect("исходник читается");
        let hits = scan(&text);
        let count = hits.device + if name == "sim-human" { 0 } else { hits.anatomy };
        if count > 0 {
            let relative = file.strip_prefix(root().join("crates")).unwrap();
            out.insert(relative.to_string_lossy().replace('\\', "/"), count);
        }
    }
    out
}

fn read_baseline() -> BTreeMap<String, usize> {
    let text = std::fs::read_to_string(baseline_file()).unwrap_or_default();
    text.lines()
        .filter(|l| !l.trim().is_empty() && !l.starts_with('#'))
        .map(|l| {
            let (count, file) = l.split_once(' ').expect("строка: число и файл");
            (file.to_string(), count.parse().expect("число"))
        })
        .collect()
}

fn write_baseline(map: &BTreeMap<String, usize>) {
    let mut text =
        String::from("# Упоминания устройств в старых крейтах. Числа могут только падать.\n");
    for (file, count) in map {
        text.push_str(&format!("{count} {file}\n"));
    }
    std::fs::write(baseline_file(), text).expect("база записывается");
}

#[test]
fn scanner_catches_device_names_and_spares_ordinary_words() {
    for text in [
        "struct ExoLoop;",
        "fn exo_step()",
        "экзоскелет локтя",
        "умные очки",
        "кнопка на Третья рука",
        "третья_рука",
        "PanTilt",
        "pan-tilt",
        "gantry_xyz",
        "балансирующая тележка",
    ] {
        assert!(scan(text).device > 0, "не поймано: {text}");
    }
    assert!(scan("ElbowPlant").anatomy > 0);
    for text in [
        "точка очередь строчка",
        "execute exotic exponent",
        "балансировка ячеек",
        "cartesian portable",
        "телефон телега",
    ] {
        assert_eq!(scan(text), Hits::default(), "ложное срабатывание: {text}");
    }
}

#[test]
fn platform_crates_have_no_device_names() {
    let mut bad = Vec::new();
    for name in crates().iter().filter(|c| !LEGACY.contains(&c.as_str())) {
        for (file, count) in crate_hits(name) {
            bad.push(format!("{file}: {count}"));
        }
    }
    assert!(bad.is_empty(), "имена устройств в платформе:\n{}", bad.join("\n"));
}

#[test]
fn legacy_crates_only_shrink() {
    let old = read_baseline();
    let mut now = BTreeMap::new();
    for name in LEGACY {
        assert!(
            root().join("crates").join(name).is_dir(),
            "{name} удалён: уберите его из LEGACY"
        );
        now.extend(crate_hits(name));
    }
    let grown: Vec<String> = now
        .iter()
        .filter(|(file, count)| old.get(*file).is_none_or(|was| *count > was))
        .map(|(file, count)| format!("{file}: было {:?}, стало {count}", old.get(file)))
        .collect();
    assert!(grown.is_empty(), "код под устройства вырос:\n{}", grown.join("\n"));
    if std::env::var_os("UPDATE_PLATFORM_BASELINE").is_some() {
        write_baseline(&now);
        return;
    }
    assert_eq!(
        now, old,
        "счётчик упал: запишите новую базу (UPDATE_PLATFORM_BASELINE=1)"
    );
}

#[test]
fn platform_does_not_depend_on_device_layer() {
    let mut bad = Vec::new();
    for name in crates().iter().filter(|c| !DEVICE_LAYER.contains(&c.as_str())) {
        let manifest = std::fs::read_to_string(root().join("crates").join(name).join("Cargo.toml"))
            .expect("манифест");
        for dep in DEVICE_LAYER {
            if manifest.lines().any(|l| l.trim_start().starts_with(&format!("{dep}."))
                || l.trim_start().starts_with(&format!("{dep} ")))
            {
                bad.push(format!("{name} → {dep}"));
            }
        }
    }
    assert!(bad.is_empty(), "{}", bad.join("\n"));
}
