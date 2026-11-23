//! Файл проекта `.device`: обычный текст, секции и строки «имя = значение». Его можно
//! править руками, поэтому разбор строгий: опечатка в имени поля или числе — это ошибка с
//! номером строки, а не молча подставленное значение.

use std::collections::BTreeMap;
use std::fmt::Write as _;

use crate::Wiring;
use crate::measure::{Measurement, Quantity};
use crate::model::*;
use crate::settings::*;

const ESCAPED_FORMAT: &str = "# Формат sim-device 2";

/// Текст проекта.
pub fn to_text(d: &Device) -> String {
    let mut s = format!("# Проект мастерской устройств\n{ESCAPED_FORMAT}\n\n[device]\n");
    let _ = writeln!(s, "name = {}", encode(&d.name));
    let _ = writeln!(s, "template = {}", encode(&d.template));
    let _ = writeln!(s, "mount = {}", d.mount.key());

    let u = &d.usage;
    s += "\n[use]\n";
    let _ = writeln!(s, "ambient_c = {}", u.ambient_c);
    let _ = writeln!(s, "cycles = {}", u.cycles);
    let _ = writeln!(s, "long_wear = {}", u.long_wear);
    let _ = writeln!(s, "hours_per_day = {}", u.hours_per_day);

    let e = &d.electronics;
    s += "\n[electronics]\n";
    let _ = writeln!(s, "board = {}", encode(&e.board));
    let _ = writeln!(s, "driver = {}", encode(&e.driver));
    let _ = writeln!(s, "regulator = {}", encode(&e.regulator));
    let _ = writeln!(s, "cell = {}", encode(&e.cell));
    let _ = writeln!(s, "series = {}", e.series);
    let _ = writeln!(s, "parallel = {}", e.parallel);
    let _ = writeln!(s, "fuse_a = {}", e.fuse_a);
    let _ = writeln!(s, "wire_mm2 = {}", e.wire_mm2);
    let _ = writeln!(s, "wire_m = {}", e.wire_m);
    let _ = writeln!(s, "bms = {}", e.bms);
    let _ = writeln!(s, "firmware = {}", encode(&e.firmware));

    write_settings(&mut s, ElectricalSettings::FIELDS, |key| {
        e.settings.get(key)
    });
    if DeviceSettings::FIELDS
        .iter()
        .any(|f| d.settings.get(f.key).is_some())
    {
        s += "\n[design]\n";
        write_settings(&mut s, DeviceSettings::FIELDS, |key| d.settings.get(key));
    }
    for c in &e.connections {
        let _ = writeln!(s, "\n[connection]\nspec = {}", c.to_line());
    }
    for sensor in &e.sensors {
        s += "\n[sensor]\n";
        write_settings(&mut s, SensorSettings::FIELDS, |key| sensor.get(key));
    }
    for case in &e.firmware_cases {
        s += "\n[firmware_case]\n";
        write_settings(&mut s, FirmwareSettings::FIELDS, |key| case.get(key));
    }
    for l in &d.links {
        s += "\n[link]\n";
        let _ = writeln!(s, "name = {}", encode(&l.name));
        let parent = l.parent.map_or("none".to_string(), |p| (p + 1).to_string());
        let _ = writeln!(s, "parent = {parent}");
        let _ = writeln!(
            s,
            "offset_mm = {} {} {}",
            l.offset_mm[0], l.offset_mm[1], l.offset_mm[2]
        );
        let _ = writeln!(s, "direction = {}", l.direction.key());
        let _ = writeln!(s, "length_mm = {}", l.length_mm);
        let section = match l.section {
            Section::Rect { b_mm, h_mm } => format!("rect {b_mm} {h_mm}"),
            Section::Round { d_mm } => format!("round {d_mm}"),
            Section::Tube { od_mm, id_mm } => format!("tube {od_mm} {id_mm}"),
        };
        let _ = writeln!(s, "section = {section}");
        let _ = writeln!(s, "material = {}", encode(&l.material));
        let print = match l.print {
            None => "none".to_string(),
            Some(p) => format!(
                "{} {} {} {}",
                p.layer_mm,
                p.walls,
                p.infill_pct,
                if p.across_layers { "across" } else { "along" }
            ),
        };
        let _ = writeln!(s, "print = {print}");
        let _ = writeln!(s, "hole_mm = {}", l.hole_mm);
        let _ = writeln!(s, "end_mass_kg = {}", l.end_mass_kg);
        let _ = writeln!(s, "near_body = {}", l.near_body.key());
        let _ = writeln!(s, "contact_lever_mm = {}", l.contact_lever_mm);
        let j = &l.joint;
        let kind = if j.kind == JointKind::Hinge {
            "hinge"
        } else {
            "fixed"
        };
        let _ = writeln!(s, "joint = {kind}");
        let _ = writeln!(s, "axis = {}", j.axis.key());
        let _ = writeln!(s, "min_deg = {}", j.min_deg);
        let _ = writeln!(s, "max_deg = {}", j.max_deg);
        let _ = writeln!(s, "actuator = {}", encode(&j.actuator));
        let _ = writeln!(s, "speed_limit_dps = {}", j.speed_limit_dps);
        let _ = writeln!(s, "torque_limit_nm = {}", j.torque_limit_nm);
        let _ = writeln!(s, "pinch_gap_mm = {}", j.pinch_gap_mm);
        let _ = writeln!(s, "guarded = {}", j.guarded);
        write_settings(&mut s, LinkSettings::FIELDS, |key| l.settings.get(key));
    }
    for m in &d.measurements {
        s += "\n[measurement]\n";
        let _ = writeln!(s, "quantity = {}", m.quantity.key());
        let link = m.link.map_or("none".to_string(), |l| (l + 1).to_string());
        let _ = writeln!(s, "link = {link}");
        let _ = writeln!(s, "value = {}", m.value);
        let _ = writeln!(s, "note = {}", encode(&m.note));
        if let Some(design) = m.design {
            let _ = writeln!(s, "design = {design:016x}");
        }
    }
    s
}

fn encode(value: &str) -> String {
    if value.trim() == value && !value.contains(['\n', '\r']) && !value.starts_with("@:") {
        return value.to_owned();
    }
    let mut out = String::from("@:");
    for byte in value.as_bytes() {
        let _ = write!(out, "{byte:02x}");
    }
    out
}

fn decode(value: String, line: usize) -> Result<String, String> {
    let Some(encoded) = value.strip_prefix("@:") else {
        return Ok(value);
    };
    let error = || format!("строка {line}: повреждённое текстовое поле");
    if encoded.len() % 2 != 0 || !encoded.is_ascii() {
        return Err(error());
    }
    let bytes: Result<Vec<_>, _> = (0..encoded.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&encoded[i..i + 2], 16))
        .collect();
    String::from_utf8(bytes.map_err(|_| error())?).map_err(|_| error())
}

fn write_settings(s: &mut String, fields: &[Field], get: impl Fn(&str) -> Option<String>) {
    for f in fields {
        if let Some(value) = get(f.key) {
            let _ = writeln!(s, "{} = {}", f.key, encode(&value));
        }
    }
}

fn read_settings(
    b: &mut Block,
    fields: &[Field],
    mut set: impl FnMut(&str, &str) -> Result<(), String>,
) -> Result<(), String> {
    for f in fields {
        if let Some((value, line)) = b.values.remove(f.key) {
            let decoded = if b.escaped {
                decode(value, line)?
            } else {
                value
            };
            if f.numeric && !decoded.trim().is_empty() {
                parse_num(decoded.trim(), line, f.key)?;
            }
            set(f.key, &decoded).map_err(|e| format!("строка {line}: {e}"))?;
        }
    }
    Ok(())
}

/// Одна секция файла: значения и номера строк, чтобы ошибка указывала на место.
struct Block {
    title: String,
    line: usize,
    values: BTreeMap<String, (String, usize)>,
    escaped: bool,
}

impl Block {
    fn take(&mut self, key: &str) -> Result<(String, usize), String> {
        self.values.remove(key).ok_or_else(|| {
            format!(
                "секция [{}] (строка {}): нет поля «{key}»",
                self.title, self.line
            )
        })
    }

    fn has(&self, key: &str) -> bool {
        self.values.contains_key(key)
    }

    fn text(&mut self, key: &str) -> Result<String, String> {
        let (v, line) = self.take(key)?;
        if self.escaped { decode(v, line) } else { Ok(v) }
    }

    fn num(&mut self, key: &str) -> Result<f64, String> {
        let (v, n) = self.take(key)?;
        parse_num(&v, n, key)
    }

    fn uint(&mut self, key: &str) -> Result<u32, String> {
        let (v, n) = self.take(key)?;
        v.parse()
            .map_err(|_| format!("строка {n}: «{key}» должно быть целым, а не «{v}»"))
    }

    fn flag(&mut self, key: &str) -> Result<bool, String> {
        let (v, n) = self.take(key)?;
        match v.as_str() {
            "true" => Ok(true),
            "false" => Ok(false),
            _ => Err(format!(
                "строка {n}: «{key}» должно быть true или false, а не «{v}»"
            )),
        }
    }

    fn finish(self) -> Result<(), String> {
        match self.values.iter().next() {
            Some((k, (_, n))) => Err(format!(
                "строка {n}: неизвестное поле «{k}» в секции [{}]",
                self.title
            )),
            None => Ok(()),
        }
    }
}

fn parse_num(v: &str, line: usize, key: &str) -> Result<f64, String> {
    match v.parse::<f64>() {
        Ok(x) if x.is_finite() => Ok(x),
        _ => Err(format!(
            "строка {line}: «{key}» должно быть числом, а не «{v}»"
        )),
    }
}

fn parse_blocks(text: &str) -> Result<Vec<Block>, String> {
    let mut blocks: Vec<Block> = Vec::new();
    let escaped = text
        .lines()
        .map(str::trim)
        .take_while(|line| line.starts_with('#') || line.is_empty())
        .any(|line| line == ESCAPED_FORMAT);
    for (idx, raw) in text.lines().enumerate() {
        let n = idx + 1;
        let line = raw.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if let Some(title) = line.strip_prefix('[').and_then(|x| x.strip_suffix(']')) {
            blocks.push(Block {
                title: title.trim().to_string(),
                line: n,
                values: BTreeMap::new(),
                escaped,
            });
            continue;
        }
        let (k, v) = line
            .split_once('=')
            .ok_or_else(|| format!("строка {n}: нужно «имя = значение»"))?;
        let block = blocks
            .last_mut()
            .ok_or_else(|| format!("строка {n}: значение вне секции"))?;
        let key = k.trim().to_string();
        if block
            .values
            .insert(key.clone(), (v.trim().to_string(), n))
            .is_some()
        {
            return Err(format!("строка {n}: поле «{key}» задано дважды"));
        }
    }
    Ok(blocks)
}

fn parse_link(mut b: Block, index: usize) -> Result<LinkSpec, String> {
    let name = b.text("name")?;
    let (parent_text, n) = b.take("parent")?;
    let parent = if parent_text == "none" {
        None
    } else {
        let p: usize = parent_text
            .parse()
            .map_err(|_| format!("строка {n}: «parent» — номер звена или none"))?;
        if p == 0 || p > index {
            return Err(format!(
                "строка {n}: родителем может быть только звено выше по файлу"
            ));
        }
        Some(p - 1)
    };
    let (off, n) = b.take("offset_mm")?;
    let parts: Vec<&str> = off.split_whitespace().collect();
    if parts.len() != 3 {
        return Err(format!("строка {n}: «offset_mm» — три числа"));
    }
    let offset_mm = [
        parse_num(parts[0], n, "offset_mm")?,
        parse_num(parts[1], n, "offset_mm")?,
        parse_num(parts[2], n, "offset_mm")?,
    ];
    let (dir, n) = b.take("direction")?;
    let direction = Direction::from_key(&dir)
        .ok_or_else(|| format!("строка {n}: неизвестное направление «{dir}»"))?;
    let length_mm = b.num("length_mm")?;
    let (sec, n) = b.take("section")?;
    let p: Vec<&str> = sec.split_whitespace().collect();
    let section = match p.as_slice() {
        ["rect", x, y] => Section::Rect {
            b_mm: parse_num(x, n, "section")?,
            h_mm: parse_num(y, n, "section")?,
        },
        ["round", x] => Section::Round {
            d_mm: parse_num(x, n, "section")?,
        },
        ["tube", x, y] => Section::Tube {
            od_mm: parse_num(x, n, "section")?,
            id_mm: parse_num(y, n, "section")?,
        },
        _ => {
            return Err(format!(
                "строка {n}: сечение — rect Ш В, round Д или tube Д_нар Д_вн"
            ));
        }
    };
    let material = b.text("material")?;
    let (pr, n) = b.take("print")?;
    let print = if pr == "none" {
        None
    } else {
        let q: Vec<&str> = pr.split_whitespace().collect();
        let [layer, walls, infill, dir] = q.as_slice() else {
            return Err(format!(
                "строка {n}: печать — «слой стенки заполнение along|across» или none"
            ));
        };
        let across_layers = match *dir {
            "across" => true,
            "along" => false,
            _ => {
                return Err(format!(
                    "строка {n}: направление нагрузки — along или across"
                ));
            }
        };
        Some(PrintSpec {
            layer_mm: parse_num(layer, n, "print")?,
            walls: walls
                .parse()
                .map_err(|_| format!("строка {n}: число стенок — целое"))?,
            infill_pct: parse_num(infill, n, "print")?,
            across_layers,
        })
    };
    let hole_mm = b.num("hole_mm")?;
    let end_mass_kg = b.num("end_mass_kg")?;
    let (nb, n) = b.take("near_body")?;
    let near_body = BodyRegion::from_key(&nb)
        .ok_or_else(|| format!("строка {n}: неизвестный участок тела «{nb}»"))?;
    let contact_lever_mm = b.num("contact_lever_mm")?;
    let (jk, n) = b.take("joint")?;
    let kind = match jk.as_str() {
        "hinge" => JointKind::Hinge,
        "fixed" => JointKind::Fixed,
        _ => return Err(format!("строка {n}: шарнир — hinge или fixed")),
    };
    let (ax, n) = b.take("axis")?;
    let axis = Axis::from_key(&ax).ok_or_else(|| format!("строка {n}: ось — x, y или z"))?;
    let joint = JointSpec {
        kind,
        axis,
        min_deg: b.num("min_deg")?,
        max_deg: b.num("max_deg")?,
        actuator: b.text("actuator")?,
        speed_limit_dps: b.num("speed_limit_dps")?,
        torque_limit_nm: b.num("torque_limit_nm")?,
        pinch_gap_mm: b.num("pinch_gap_mm")?,
        guarded: b.flag("guarded")?,
    };
    let mut settings = LinkSettings::default();
    read_settings(&mut b, LinkSettings::FIELDS, |k, v| settings.set(k, v))?;
    b.finish()?;
    Ok(LinkSpec {
        name,
        parent,
        offset_mm,
        direction,
        length_mm,
        section,
        material,
        print,
        hole_mm,
        end_mass_kg,
        near_body,
        contact_lever_mm,
        joint,
        settings,
    })
}

/// Разобрать текст проекта. Результат дополнительно проходит [`Device::validate`].
pub fn from_text(text: &str) -> Result<Device, String> {
    let mut device = None;
    let mut usage = None;
    let mut electronics = None;
    let mut links = Vec::new();
    let mut measurements = Vec::new();
    let mut settings = None;
    let mut connections = Vec::new();
    let mut sensors = Vec::new();
    let mut firmware_cases = Vec::new();
    for mut b in parse_blocks(text)? {
        let duplicate = match b.title.as_str() {
            "device" => device.is_some(),
            "use" => usage.is_some(),
            "electronics" => electronics.is_some(),
            "design" => settings.is_some(),
            _ => false,
        };
        if duplicate {
            return Err(format!(
                "строка {}: секция [{}] задана дважды",
                b.line, b.title
            ));
        }
        match b.title.as_str() {
            "device" => {
                let (name, template, (m, n)) =
                    (b.text("name")?, b.text("template")?, b.take("mount")?);
                let mount = BodyRegion::from_key(&m)
                    .ok_or_else(|| format!("строка {n}: неизвестное крепление «{m}»"))?;
                b.finish()?;
                device = Some((name, template, mount));
            }
            "use" => {
                let u = UseProfile {
                    ambient_c: b.num("ambient_c")?,
                    cycles: b.num("cycles")?,
                    long_wear: b.flag("long_wear")?,
                    hours_per_day: b.num("hours_per_day")?,
                };
                b.finish()?;
                usage = Some(u);
            }
            "electronics" => {
                let mut e = Electronics {
                    board: b.text("board")?,
                    driver: b.text("driver")?,
                    regulator: b.text("regulator")?,
                    cell: b.text("cell")?,
                    series: b.uint("series")?,
                    parallel: b.uint("parallel")?,
                    fuse_a: b.num("fuse_a")?,
                    wire_mm2: b.num("wire_mm2")?,
                    wire_m: b.num("wire_m")?,
                    bms: b.flag("bms")?,
                    firmware: b.text("firmware")?,
                    ..Electronics::default()
                };
                read_settings(&mut b, ElectricalSettings::FIELDS, |k, v| {
                    e.settings.set(k, v)
                })?;
                b.finish()?;
                electronics = Some(e);
            }
            "design" => {
                let mut profile = DeviceSettings::default();
                read_settings(&mut b, DeviceSettings::FIELDS, |k, v| profile.set(k, v))?;
                b.finish()?;
                settings = Some(profile);
            }
            "connection" => {
                let (spec, line) = b.take("spec")?;
                connections
                    .push(Wiring::from_line(&spec).map_err(|e| format!("строка {line}: {e}"))?);
                b.finish()?;
            }
            "sensor" => {
                let mut sensor = SensorSettings::default();
                read_settings(&mut b, SensorSettings::FIELDS, |k, v| sensor.set(k, v))?;
                b.finish()?;
                sensors.push(sensor);
            }
            "firmware_case" => {
                let mut case = FirmwareSettings::default();
                read_settings(&mut b, FirmwareSettings::FIELDS, |k, v| case.set(k, v))?;
                b.finish()?;
                firmware_cases.push(case);
            }
            "link" => links.push(parse_link(b, links.len())?),
            "measurement" => {
                let (qk, n) = b.take("quantity")?;
                let quantity = Quantity::from_key(&qk)
                    .ok_or_else(|| format!("строка {n}: неизвестная величина «{qk}»"))?;
                let (lk, n) = b.take("link")?;
                let link = if lk == "none" {
                    None
                } else {
                    let v: usize = lk
                        .parse()
                        .map_err(|_| format!("строка {n}: «link» — номер звена или none"))?;
                    if v == 0 {
                        return Err(format!("строка {n}: звенья нумеруются с 1"));
                    }
                    Some(v - 1)
                };
                let value = b.num("value")?;
                let note = b.text("note")?;
                // Старые файлы без отпечатка открываются, но такой замер ничего не подтверждает.
                let design = if b.has("design") {
                    let (text, n) = b.take("design")?;
                    Some(u64::from_str_radix(&text, 16).map_err(|_| {
                        format!("строка {n}: «design» — шестнадцатеричный отпечаток")
                    })?)
                } else {
                    None
                };
                b.finish()?;
                measurements.push(Measurement {
                    quantity,
                    link,
                    value,
                    note,
                    design,
                });
            }
            other => return Err(format!("строка {}: неизвестная секция [{other}]", b.line)),
        }
    }
    let (name, template, mount) = device.ok_or("нет секции [device]")?;
    let mut electronics = electronics.ok_or("нет секции [electronics]")?;
    electronics.connections = connections;
    electronics.sensors = sensors;
    electronics.firmware_cases = firmware_cases;
    let d = Device {
        name,
        template,
        mount,
        links,
        electronics,
        settings: settings.unwrap_or_default(),
        usage: usage.ok_or("нет секции [use]")?,
        measurements,
    };
    let errors = d.validate();
    if errors.is_empty() {
        Ok(d)
    } else {
        Err(errors.join("; "))
    }
}
