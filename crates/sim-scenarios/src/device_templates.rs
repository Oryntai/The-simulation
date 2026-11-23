//! Каталог файлов конструктора. Названия берутся из поля `name` файла `.device`.

use std::collections::BTreeSet;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

use sim_device::file::from_text;
use sim_device::{BodyRegion, Device};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DeviceTemplate {
    pub id: String,
    pub title: String,
}

/// Список шаблонов в порядке показа. При ошибке чтения каталога список пуст.
pub fn templates() -> Vec<DeviceTemplate> {
    TemplateCatalog::open(&template_directory()).map_or_else(|_| Vec::new(), |c| c.entries)
}

/// Загружает шаблон или возвращает пустое устройство, которое не проходит валидацию.
pub fn template(index: usize) -> Device {
    try_template(index).unwrap_or_else(|_| empty_device())
}

/// Загружает шаблон с диагностикой ошибки каталога, индекса или файла.
pub fn try_template(index: usize) -> Result<Device, String> {
    TemplateCatalog::open(&template_directory())?.load(index)
}

pub(crate) fn template_directory() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../devices/templates")
}

pub(crate) fn empty_device() -> Device {
    Device {
        name: String::new(),
        template: String::new(),
        mount: BodyRegion::None,
        links: Vec::new(),
        electronics: Default::default(),
        usage: Default::default(),
        settings: Default::default(),
        measurements: Vec::new(),
    }
}

pub(crate) struct TemplateCatalog {
    dir: PathBuf,
    pub(crate) entries: Vec<DeviceTemplate>,
    pub(crate) default_index: Option<usize>,
}

impl TemplateCatalog {
    pub(crate) fn open(dir: &Path) -> Result<Self, String> {
        let index = CatalogIndex::read(&dir.join("index.toml"))?;
        let dir_error = |error| format!("Каталог шаблонов «{}»: {error}", dir.display());
        let mut discovered = BTreeSet::new();
        for entry in fs::read_dir(dir).map_err(dir_error)? {
            let path = entry.map_err(dir_error)?.path();
            if path.extension().is_some_and(|ext| ext == "device") {
                let id = path
                    .file_stem()
                    .and_then(|stem| stem.to_str())
                    .filter(|stem| valid_stem(stem))
                    .ok_or_else(|| format!("Неверное имя файла шаблона «{}»", path.display()))?;
                discovered.insert(id.to_owned());
            }
        }
        // Запись из индекса остаётся доступна даже после потери её файла.
        discovered.extend(index.default.iter().cloned());
        let mut ids = index.order;
        for id in &ids {
            discovered.remove(id);
        }
        ids.extend(discovered);
        let default_index = index
            .default
            .and_then(|id| ids.iter().position(|entry| *entry == id));
        let entries = ids
            .into_iter()
            .map(|id| {
                let title = read_device(dir, &id)
                    .map_or_else(|_| format!("{id} (ошибка загрузки)"), |device| device.name);
                DeviceTemplate { id, title }
            })
            .collect();
        Ok(Self {
            dir: dir.to_owned(),
            entries,
            default_index,
        })
    }

    pub(crate) fn load(&self, index: usize) -> Result<Device, String> {
        let entry = self
            .entries
            .get(index)
            .ok_or_else(|| format!("Нет шаблона с индексом {index}"))?;
        // Проверяем файл заново: он мог измениться после показа списка.
        read_device(&self.dir, &entry.id)
    }
}

fn read_device(dir: &Path, id: &str) -> Result<Device, String> {
    let text = read_regular_text(&dir.join(format!("{id}.device")))
        .map_err(|error| format!("Чтение шаблона «{id}»: {error}"))?;
    from_text(&text).map_err(|error| format!("Шаблон «{id}»: {error}"))
}

fn read_regular_text(path: &Path) -> io::Result<String> {
    if !path.symlink_metadata()?.is_file() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "ожидался обычный файл",
        ));
    }
    fs::read_to_string(path)
}

pub(crate) fn valid_stem(stem: &str) -> bool {
    !stem.is_empty()
        && stem
            .chars()
            .all(|c| c.is_alphanumeric() || c == '-' || c == '_')
}

#[derive(Default)]
struct CatalogIndex {
    default: Option<String>,
    order: Vec<String>,
}

impl CatalogIndex {
    fn read(path: &Path) -> Result<Self, String> {
        match read_regular_text(path) {
            Ok(text) => Self::parse(&text).map_err(|error| format!("Индекс шаблонов: {error}")),
            Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(Self::default()),
            Err(error) => Err(format!("Чтение индекса шаблонов: {error}")),
        }
    }

    // Подмножество TOML: две необязательные однострочные записи без экранирования id.
    fn parse(text: &str) -> Result<Self, String> {
        let mut default = None;
        let mut order = None;
        for (line, raw) in text.lines().enumerate() {
            let record = raw.split('#').next().unwrap_or_default().trim();
            if record.is_empty() {
                continue;
            }
            let error = |detail| format!("строка {}: {detail}", line + 1);
            let (key, value) = record
                .split_once('=')
                .ok_or_else(|| error("ожидалась запись «ключ = значение»"))?;
            let value = value.trim();
            match key.trim() {
                "default" if default.is_none() => default = Some(quoted_id(value).map_err(error)?),
                "order" if order.is_none() => {
                    let list = value
                        .strip_prefix('[')
                        .and_then(|s| s.strip_suffix(']'))
                        .ok_or_else(|| error("в «order» ожидается массив идентификаторов"))?;
                    let ids = list
                        .trim()
                        .split_terminator(',')
                        .map(|s| quoted_id(s.trim()))
                        .collect::<Result<Vec<_>, _>>()
                        .map_err(error)?;
                    if ids.iter().collect::<BTreeSet<_>>().len() != ids.len() {
                        return Err(error("идентификатор в «order» указан дважды"));
                    }
                    order = Some(ids);
                }
                "default" | "order" => return Err(error("поле указано дважды")),
                _ => return Err(error("неизвестное поле индекса")),
            }
        }
        Ok(Self {
            default,
            order: order.unwrap_or_default(),
        })
    }
}

fn quoted_id(text: &str) -> Result<String, &'static str> {
    text.strip_prefix('"')
        .and_then(|s| s.strip_suffix('"'))
        .filter(|s| valid_stem(s))
        .map(str::to_owned)
        .ok_or("ожидался идентификатор файла в двойных кавычках")
}
