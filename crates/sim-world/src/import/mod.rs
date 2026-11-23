//! Поддерживаемое подмножество XML явно задано в конвертерах. Любой
//! неизвестный элемент или атрибут возвращает имя и путь в исходном XML.

mod mjcf;
mod urdf;

pub use mjcf::{import_mjcf, import_mjcf_world};
pub use urdf::import_urdf;

use std::collections::BTreeMap;

use quick_xml::{
    Reader,
    events::{BytesStart, Event},
};

pub(super) struct Node {
    tag: String,
    path: String,
    attrs: BTreeMap<String, String>,
    children: Vec<Node>,
}

impl Node {
    fn check(&self, attrs: &[&str], children: &[&str]) -> Result<(), String> {
        for key in self.attrs.keys() {
            if !attrs.contains(&key.as_str()) {
                return Err(self.err(format!("неподдерживаемый атрибут «{key}»")));
            }
        }
        for child in &self.children {
            if !children.contains(&child.tag.as_str()) {
                return Err(child.err(format!("неподдерживаемый элемент <{}>", child.tag)));
            }
        }
        Ok(())
    }

    fn err(&self, reason: impl std::fmt::Display) -> String {
        format!("{}: {reason}", self.path)
    }

    fn get(&self, key: &str) -> Option<&str> {
        self.attrs.get(key).map(String::as_str)
    }

    fn need(&self, key: &str) -> Result<&str, String> {
        self.get(key)
            .ok_or_else(|| self.err(format!("требуется атрибут «{key}»")))
    }

    fn opt(&self, tag: &str) -> Result<Option<&Node>, String> {
        let mut nodes = self.children.iter().filter(|n| n.tag == tag);
        let first = nodes.next();
        if nodes.next().is_some() {
            Err(self.err(format!("элемент <{tag}> задан дважды")))
        } else {
            Ok(first)
        }
    }

    fn one(&self, tag: &str) -> Result<&Node, String> {
        self.opt(tag)?
            .ok_or_else(|| self.err(format!("требуется элемент <{tag}>")))
    }

    fn parse<T: std::str::FromStr>(&self, key: &str, v: &str) -> Result<T, String> {
        v.parse()
            .map_err(|_| self.err(format!("«{key}»: неверное значение «{v}»")))
    }

    fn scalar(&self, key: &str) -> Result<f64, String> {
        self.float(key, self.need(key)?)
    }

    fn optional(&self, key: &str) -> Result<Option<f64>, String> {
        self.get(key).map(|v| self.float(key, v)).transpose()
    }

    fn float(&self, key: &str, value: &str) -> Result<f64, String> {
        let v: f64 = self.parse(key, value)?;
        if !v.is_finite() {
            return Err(self.err(format!("«{key}»: требуется конечное число")));
        }
        Ok(v)
    }

    fn floats(&self, key: &str) -> Result<Vec<f64>, String> {
        self.need(key)?
            .split_whitespace()
            .map(|v| self.float(key, v))
            .collect()
    }

    fn array<const N: usize>(&self, key: &str) -> Result<[f64; N], String> {
        self.floats(key)?
            .try_into()
            .map_err(|_| self.err(format!("«{key}»: требуется {N} чисел")))
    }

    fn array_or<const N: usize>(&self, key: &str, default: [f64; N]) -> Result<[f64; N], String> {
        if self.get(key).is_some() {
            self.array(key)
        } else {
            Ok(default)
        }
    }

    fn flag(&self, key: &str, default: bool) -> Result<bool, String> {
        self.get(key)
            .map(|v| self.parse(key, v))
            .transpose()
            .map(|v| v.unwrap_or(default))
    }
}

fn node(event: BytesStart<'_>, reader: &Reader<&[u8]>, parents: &[Node]) -> Result<Node, String> {
    let tag = std::str::from_utf8(event.name().as_ref())
        .map_err(|e| e.to_string())?
        .to_owned();
    let path = format!(
        "{}/{tag}",
        parents.last().map(|n| n.path.as_str()).unwrap_or("")
    );
    let mut attrs = BTreeMap::new();
    for attr in event.attributes() {
        let attr = attr.map_err(|e| format!("{path}: {e}"))?;
        let key = std::str::from_utf8(attr.key.as_ref()).map_err(|e| format!("{path}: {e}"))?;
        let value = attr
            .decoded_and_normalized_value(quick_xml::XmlVersion::Implicit1_0, reader.decoder())
            .map_err(|e| format!("{path}.{key}: {e}"))?;
        attrs.insert(key.to_owned(), value.into_owned());
    }
    Ok(Node {
        tag,
        path,
        attrs,
        children: Vec::new(),
    })
}

fn parse_xml(text: &str, root: &str) -> Result<Node, String> {
    fn add(n: Node, parents: &mut [Node], doc: &mut Option<Node>) -> Result<(), String> {
        if let Some(p) = parents.last_mut() {
            p.children.push(n);
        } else if doc.is_none() {
            *doc = Some(n);
        } else {
            return Err(n.err("в документе два корневых элемента"));
        }
        Ok(())
    }

    let mut reader = Reader::from_str(text);
    let mut parents = Vec::new();
    let mut doc = None;
    // Такой же предел вложенности, как у включённой защиты рекурсии TOML.
    const MAX_XML_DEPTH: usize = 80;
    loop {
        let event = reader
            .read_event()
            .map_err(|e| format!("байт {}: {e}", reader.error_position()))?;
        match event {
            Event::Start(e) => {
                let n = node(e, &reader, &parents)?;
                if parents.len() >= MAX_XML_DEPTH {
                    return Err(n.err("слишком глубокая вложенность"));
                }
                parents.push(n);
            }
            Event::Empty(e) => {
                let n = node(e, &reader, &parents)?;
                add(n, &mut parents, &mut doc)?;
            }
            Event::End(_) => {
                let n = parents.pop().ok_or("неожиданный закрывающий элемент")?;
                add(n, &mut parents, &mut doc)?;
            }
            Event::Text(t) if t.decode().is_ok_and(|s| s.trim().is_empty()) => {}
            Event::Comment(_) => {}
            Event::Decl(_) if doc.is_none() && parents.is_empty() => {}
            Event::Eof => break,
            event => {
                return Err(format!(
                    "байт {}: неподдерживаемая запись XML {event:?}",
                    reader.buffer_position()
                ));
            }
        }
    }
    if let Some(unclosed) = parents.last() {
        return Err(unclosed.err("элемент не закрыт"));
    }
    let doc = doc.ok_or("нет корневого элемента XML")?;
    if doc.tag != root {
        return Err(doc.err(format!(
            "неподдерживаемый элемент <{}>, нужен <{root}>",
            doc.tag
        )));
    }
    Ok(doc)
}

fn input_path(n: &Node, key: &str) -> Result<std::path::PathBuf, String> {
    let path = n.need(key)?;
    if path.is_empty() || path.contains("://") {
        Err(n.err(format!(
            "«{key}»: требуется локальный путь к STL/OBJ, получено «{path}»"
        )))
    } else {
        Ok(path.into())
    }
}
