//! Чтение формата 3 и автоматическое определение формата подключённых моделей.

use std::{collections::BTreeMap, fs, path::Path};

use serde::de::DeserializeOwned;
use toml::de::{DeTable, DeValue, Deserializer};

use crate::compile::{base, version};
use crate::{
    CompiledWorld, ContactBuildOptions, Device, DeviceInput, LegacyCatalog, World,
    compile_world_with_contact_options, import_mjcf, import_urdf, migrate_v2,
};

fn read(path: &Path) -> Result<String, String> {
    fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))
}

fn field_at(table: &DeTable<'_>, index: usize, prefix: &str) -> Option<String> {
    table.iter().find_map(|(k, v)| {
        let key: &str = k.get_ref();
        let path = if prefix.is_empty() {
            key.to_owned()
        } else {
            format!("{prefix}.{key}")
        };
        if k.span().contains(&index) {
            Some(path)
        } else {
            value_at(v.get_ref(), v.span().contains(&index), index, &path)
        }
    })
}

fn value_at(value: &DeValue<'_>, spans: bool, index: usize, path: &str) -> Option<String> {
    let child = match value {
        DeValue::Table(table) => field_at(table, index, path),
        DeValue::Array(array) => array.iter().enumerate().find_map(|(j, v)| {
            value_at(
                v.get_ref(),
                v.span().contains(&index),
                index,
                &format!("{path}[{j}]"),
            )
        }),
        _ => None,
    };
    child.or_else(|| spans.then(|| path.into()))
}

fn finite_toml(table: &DeTable<'_>, prefix: &str) -> Result<(), String> {
    for (k, v) in table {
        let key: &str = k.get_ref();
        let path = if prefix.is_empty() {
            key.to_owned()
        } else {
            format!("{prefix}.{key}")
        };
        finite_toml_value(v.get_ref(), &path)?;
    }
    Ok(())
}

fn finite_toml_value(value: &DeValue<'_>, path: &str) -> Result<(), String> {
    match value {
        DeValue::Float(v) if v.as_str().parse::<f64>().is_ok_and(|f| !f.is_finite()) => {
            return Err(format!("{path}: допустимы только конечные числа"));
        }
        DeValue::Table(t) => finite_toml(t, path)?,
        DeValue::Array(a) => {
            for (i, v) in a.iter().enumerate() {
                finite_toml_value(v.get_ref(), &format!("{path}[{i}]"))?;
            }
        }
        _ => {}
    }
    Ok(())
}

fn parse<T: DeserializeOwned>(path: &Path, text: &str) -> Result<T, String> {
    let parsed = DeTable::parse(text).map_err(|e| format!("{}: {e}", path.display()))?;
    finite_toml(parsed.get_ref(), "").map_err(|e| format!("{}: {e}", path.display()))?;
    T::deserialize(Deserializer::from(parsed.clone())).map_err(|mut e| {
        let field = e
            .span()
            .and_then(|span| field_at(parsed.get_ref(), span.start, ""));
        // Включаем строку и столбец исходника, если десериализатор их предоставил.
        e.set_input(Some(text));
        format!(
            "{}: {}: {e}",
            path.display(),
            field.as_deref().unwrap_or("корень документа")
        )
    })
}

pub fn parse_world(path: &Path, text: &str) -> Result<World, String> {
    let w: World = parse(path, text)?;
    version(w.format).map_err(|e| format!("{}: {e}", path.display()))?;
    Ok(w)
}

pub fn parse_device(path: &Path, text: &str) -> Result<Device, String> {
    let d: Device = parse(path, text)?;
    version(d.format).map_err(|e| format!("{}: {e}", path.display()))?;
    Ok(d)
}

pub fn load_device(path: &Path, catalog: &dyn sim_device::Catalog) -> Result<Device, String> {
    let text = read(path)?;
    match path
        .extension()
        .and_then(|e| e.to_str())
        .map(str::to_ascii_lowercase)
        .as_deref()
    {
        Some("urdf") => import_urdf(path, &text),
        Some("xml" | "mjcf") => import_mjcf(path, &text),
        Some("device")
            if !text.lines().any(|line| {
                line.trim_start_matches('\u{feff}')
                    .split_once('=')
                    .is_some_and(|(key, _)| key.trim() == "format")
            }) && text.lines().any(|line| line.trim() == "[device]") =>
        {
            migrate_v2(path, &text, catalog)
        }
        _ => parse_device(path, &text),
    }
}

/// Читает мир и все подключённые модели. Для прежнего каталога с приводами
/// предназначен `load_and_compile_with_catalog`.
pub fn load_and_compile(path: &Path) -> Result<CompiledWorld, String> {
    load_and_compile_with_catalog(path, &LegacyCatalog)
}

pub fn load_and_compile_with_catalog(
    path: &Path,
    catalog: &dyn sim_device::Catalog,
) -> Result<CompiledWorld, String> {
    load_and_compile_with_catalog_and_contact_options(
        path,
        catalog,
        &ContactBuildOptions::default(),
    )
}

pub fn load_and_compile_with_contact_options(
    path: &Path,
    options: &ContactBuildOptions,
) -> Result<CompiledWorld, String> {
    load_and_compile_with_catalog_and_contact_options(path, &LegacyCatalog, options)
}

pub fn load_and_compile_with_catalog_and_contact_options(
    path: &Path,
    catalog: &dyn sim_device::Catalog,
    options: &ContactBuildOptions,
) -> Result<CompiledWorld, String> {
    let world = parse_world(path, &read(path)?)?;
    let mut devices = BTreeMap::new();
    for (i, d) in world.devices.iter().enumerate() {
        if devices.contains_key(&d.name) {
            return Err(format!(
                "{}: devices[{i}].name: имя «{}» задано дважды",
                path.display(),
                d.name
            ));
        }
        let device_path = base(path).join(&d.file);
        let device = load_device(&device_path, catalog)
            .map_err(|e| format!("{}: devices[{i}].file: {e}", path.display()))?;
        devices.insert(
            d.name.clone(),
            DeviceInput {
                path: device_path,
                device,
            },
        );
    }
    compile_world_with_contact_options(path, &world, &devices, options)
}
