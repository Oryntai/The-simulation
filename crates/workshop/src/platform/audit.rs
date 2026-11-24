//! Реакции из диагностической секции SIMLOG v2. Кодировка этой секции сейчас Debug.
//! Обрабатываем известные записи, изменение формата оставляет данные недоступными.
use sim_signal::Time;
use std::collections::BTreeMap;

use super::{
    PointSample,
    json::{self, Value},
};

#[derive(Clone, Debug)]
pub struct Wrench {
    pub pair: [String; 2],
    pub point_m: [f64; 3],
    pub force_n: [f64; 3],
}

#[derive(Default)]
pub(crate) struct Audit {
    pub wrenches: BTreeMap<Time, Vec<Wrench>>,
    pub forces: BTreeMap<String, Vec<PointSample>>,
}

impl Audit {
    pub fn read(entries: &[(Time, String)]) -> Result<Self, String> {
        let mut audit = Self::default();
        for (at, text) in entries
            .iter()
            .filter(|(_, t)| t.starts_with("ProcessTick { "))
        {
            let mut wrenches = Vec::new();
            for raw in records(text, "ContactWrench { ") {
                let after_pair = raw
                    .strip_prefix("geometries: [")
                    .and_then(|r| {
                        let (_, r) = string_token(r)?;
                        let r = r.strip_prefix(", ")?;
                        let (_, r) = string_token(r)?;
                        r.strip_prefix(']')
                    })
                    .ok_or("неверная запись пары контакта")?;
                let pair = &raw["geometries: ".len()..raw.len() - after_pair.len()];
                let rest = after_pair
                    .strip_prefix(", point_m: ")
                    .ok_or("отсутствует точка контакта")?;
                let pair = json::parse(pair)?;
                let [a, b] = pair.array().ok_or("отсутствует пара контакта")?
                else {
                    return Err("ожидалось две геометрии контакта".into());
                };
                let pair = [
                    a.str().ok_or("неверное имя геометрии")?.into(),
                    b.str().ok_or("неверное имя геометрии")?.into(),
                ];
                let (point, rest) = rest
                    .split_once(", frame: ")
                    .ok_or("отсутствует точка контакта")?;
                let (_, rest) = rest
                    .split_once(", force_n: ")
                    .ok_or("отсутствует сила контакта")?;
                let (force, _) = rest
                    .split_once(", torque_nm: ")
                    .ok_or("незавершённая запись силы контакта")?;
                wrenches.push(Wrench {
                    pair,
                    point_m: vector(point)?,
                    force_n: vector(force)?,
                });
            }
            if !wrenches.is_empty() {
                // ContactStep сохраняет мировую точку и силу до интегрирования.
                // Запись приходит на правой границе, а стрелка относится к левой позе.
                let (start, end) = text
                    .split_once("completed: Some(PhysicsFrame { interval: Interval { start: Time(")
                    .and_then(|(_, body)| body.split_once("), end: Time("))
                    .and_then(|(start, rest)| {
                        Some((
                            start.parse::<u64>().ok()?,
                            rest.split_once(") }")?.0.parse::<u64>().ok()?,
                        ))
                    })
                    .filter(|&(start, end)| start < end && end == at.0)
                    .ok_or("неверный интервал пространственной реакции")?;
                let _ = end;
                audit
                    .wrenches
                    .entry(Time(start))
                    .or_default()
                    .extend(wrenches);
            }
            for body in records(text, "ProcessMeasurement { ") {
                let (time, body) = body
                    .strip_prefix("at: Time(")
                    .and_then(|r| r.split_once("), signal: SignalSpec { name: "))
                    .ok_or("неверное время измерения")?;
                let time = Time(time.parse::<u64>().map_err(|e| e.to_string())?);
                if time != *at {
                    return Err("время измерения отличается от времени строки".into());
                }
                let (raw_name, rest) = string_token(body).ok_or("неверное имя измерения")?;
                let Some(result) = rest.strip_prefix(", unit: \"N\" }, result: ") else {
                    continue;
                };
                let name = json::parse(raw_name)?;
                let name = name.str().ok_or("неверное имя силы")?;
                let value = if let Some((raw_value, _)) = result
                    .strip_prefix("Analyzed { value: ")
                    .and_then(|r| r.split_once(", scope: "))
                {
                    Some(
                        json::parse(raw_value)?
                            .f64()
                            .ok_or("неверное значение силы")?,
                    )
                } else if result.starts_with("NotAnalyzed { reason: ") {
                    None
                } else {
                    return Err("неверный статус измерения силы".into());
                };
                audit
                    .forces
                    .entry(name.into())
                    .or_default()
                    .push(PointSample {
                        at: *at,
                        value: value.unwrap_or_default(),
                        valid: value.is_some(),
                    });
            }
        }
        Ok(audit)
    }
}

fn vector(raw: &str) -> Result<[f64; 3], String> {
    let v = json::parse(raw)?;
    let [a, b, c] = v.array().ok_or("ожидался вектор")? else {
        return Err("ожидались три компоненты вектора".into());
    };
    let number = |n: &Value| {
        n.f64()
            .ok_or_else(|| "неконечная компонента вектора".to_owned())
    };
    Ok([number(a)?, number(b)?, number(c)?])
}

fn records<'a>(text: &'a str, marker: &'a str) -> impl Iterator<Item = &'a str> {
    // Пропускаем экранированные пользовательские имена и сообщения. Только
    // лексема вне строки может начинать запись диагностической структуры.
    let mut remaining = text;
    std::iter::from_fn(move || {
        loop {
            let at = remaining.find(['"', marker.chars().next()?])?;
            remaining = &remaining[at..];
            if let Some(body) = remaining.strip_prefix(marker) {
                remaining = body;
                return Some(body);
            }
            if remaining.starts_with('"') {
                remaining = string_token(remaining)?.1;
            } else {
                remaining = &remaining[remaining.chars().next()?.len_utf8()..];
            }
        }
    })
}

fn string_token(text: &str) -> Option<(&str, &str)> {
    let body = text.strip_prefix('"')?;
    let mut escaped = false;
    let end = body.char_indices().find_map(|(i, c)| {
        if escaped {
            escaped = false;
        } else if c == '\\' {
            escaped = true;
        } else if c == '"' {
            return Some(i + 2);
        }
        None
    })?;
    Some(text.split_at(end))
}
