//! Чтение треугольных сеток. Файлы используются только при загрузке формата 3.

use std::{collections::BTreeMap, fs, path::Path};

use nalgebra::Vector3;

#[derive(Clone, Debug, PartialEq)]
pub struct Mesh {
    pub vertices_m: Vec<[f64; 3]>,
    pub faces: Vec<[usize; 3]>,
}

pub fn load_mesh(path: &Path, scale: [f64; 3]) -> Result<Mesh, String> {
    let load = || {
        if !scale.iter().all(|x| x.is_finite() && *x > 0.0) {
            return Err("scale: нужны три конечных положительных множителя".into());
        }
        let bytes = fs::read(path).map_err(|e| e.to_string())?;
        let mut mesh = match path
            .extension()
            .and_then(|s| s.to_str())
            .map(str::to_ascii_lowercase)
            .as_deref()
        {
            Some("obj") => obj(std::str::from_utf8(&bytes).map_err(|e| e.to_string())?)?,
            Some("stl") => stl(&bytes)?,
            _ => return Err("поддерживаются только STL и OBJ".into()),
        };
        for v in &mut mesh.vertices_m {
            for (x, scale) in v.iter_mut().zip(scale) {
                *x *= scale;
            }
        }
        mesh.validate()?;
        Ok(mesh)
    };
    load().map_err(|e: String| format!("{}: {e}", path.display()))
}

fn numbers<'a, const N: usize>(
    mut words: impl Iterator<Item = &'a str>,
) -> Result<[f64; N], String> {
    let mut out = [0.0; N];
    for value in &mut out {
        *value = words
            .next()
            .ok_or("не хватает координат")?
            .parse::<f64>()
            .map_err(|e| e.to_string())?;
        if !value.is_finite() {
            return Err("неконечная координата".into());
        }
    }
    if words.next().is_some() {
        return Err("лишние координаты".into());
    }
    Ok(out)
}

fn index(raw: &str, count: usize) -> Result<usize, String> {
    let i: i64 = raw.parse::<i64>().map_err(|e| e.to_string())?;
    // OBJ: положительный индекс начинается с 1, отрицательный отсчитывается с конца.
    let i = if i < 0 { count as i64 + i } else { i - 1 };
    usize::try_from(i)
        .ok()
        .filter(|i| *i < count)
        .ok_or_else(|| format!("индекс {raw} вне списка"))
}

fn obj(text: &str) -> Result<Mesh, String> {
    let mut mesh = Mesh {
        vertices_m: Vec::new(),
        faces: Vec::new(),
    };
    let mut tex = 0;
    let mut normals = 0;
    for (line, raw) in text.lines().enumerate() {
        let mut words = raw.split('#').next().unwrap_or("").split_whitespace();
        let Some(key) = words.next() else { continue };
        let parse = || -> Result<(), String> {
            match key {
                "v" => mesh.vertices_m.push(numbers(words)?),
                "vn" => {
                    numbers::<3>(words)?;
                    normals += 1;
                }
                "vt" => {
                    let values: Vec<_> = words.collect();
                    match values.len() {
                        1 => {
                            numbers::<1>(values.into_iter())?;
                        }
                        2 => {
                            numbers::<2>(values.into_iter())?;
                        }
                        3 => {
                            numbers::<3>(values.into_iter())?;
                        }
                        _ => return Err("vt: нужна 1–3 координаты".into()),
                    }
                    tex += 1;
                }
                "f" => {
                    let face = words
                        .map(|word| {
                            let mut fields = word.split('/');
                            let v = index(fields.next().unwrap_or(""), mesh.vertices_m.len())?;
                            let t = fields.next();
                            let n = fields.next();
                            if let Some(t) = t.filter(|s| !s.is_empty()) {
                                index(t, tex)?;
                            }
                            if let Some(n) = n {
                                index(n, normals)?;
                            }
                            if fields.next().is_some() || t == Some("") && n.is_none() {
                                return Err(format!("неверная вершина грани «{word}»"));
                            }
                            Ok(v)
                        })
                        .collect::<Result<Vec<_>, String>>()?;
                    mesh.add_face(&face)?;
                }
                // Группы, сглаживание и материалы OBJ не задают физическую геометрию.
                "o" | "g" | "s" | "usemtl" | "mtllib" => {}
                _ => return Err(format!("неподдерживаемая запись OBJ «{key}»")),
            }
            Ok(())
        };
        parse().map_err(|e| format!("строка {}: {e}", line + 1))?;
    }
    Ok(mesh)
}

fn stl(bytes: &[u8]) -> Result<Mesh, String> {
    // STL: 80 байт заголовка, 4 байта числа граней, 50 байт на грань.
    let binary = bytes.get(80..84).is_some_and(|count| {
        let count = u32::from_le_bytes(count.try_into().unwrap()) as usize;
        count.checked_mul(50).and_then(|n| n.checked_add(84)) == Some(bytes.len())
    });
    let mut mesh = Mesh {
        vertices_m: Vec::new(),
        faces: Vec::new(),
    };
    let mut vertices = BTreeMap::new();
    let mut add = |triangle: [[f64; 3]; 3]| {
        let face = triangle.map(|v| {
            let key = v.map(|x| if x == 0.0 { 0 } else { x.to_bits() });
            *vertices.entry(key).or_insert_with(|| {
                let i = mesh.vertices_m.len();
                mesh.vertices_m.push(v);
                i
            })
        });
        mesh.faces.push(face);
    };
    if binary {
        for (i, face) in bytes[84..].as_chunks::<50>().0.iter().enumerate() {
            // Двоичный STL хранит нормаль и три вершины в 12 float32 little-endian.
            let data: Vec<_> = face[..48]
                .as_chunks::<4>()
                .0
                .iter()
                .map(|b| f64::from(f32::from_le_bytes(*b)))
                .collect();
            if data.iter().any(|x| !x.is_finite()) {
                return Err(format!(
                    "грань {}: неконечная координата или нормаль",
                    i + 1
                ));
            }
            add([
                data[3..6].try_into().unwrap(),
                data[6..9].try_into().unwrap(),
                data[9..12].try_into().unwrap(),
            ]);
        }
    } else {
        let text = std::str::from_utf8(bytes)
            .map_err(|_| "повреждённый двоичный STL или неверная кодировка ASCII STL")?;
        let mut lines = text
            .lines()
            .enumerate()
            .filter(|(_, s)| !s.trim().is_empty());
        let first = lines.next().ok_or("пустой STL")?;
        if first.1.split_whitespace().next() != Some("solid") {
            return Err("ASCII STL должен начинаться с solid".into());
        }
        loop {
            let (line, raw) = lines.next().ok_or("нет endsolid")?;
            let mut words = raw.split_whitespace();
            if words.next() == Some("endsolid") {
                if lines.next().is_some() {
                    return Err("лишние данные после endsolid".into());
                }
                break;
            }
            let mut words = raw.split_whitespace();
            if words.next() != Some("facet") || words.next() != Some("normal") {
                return Err(format!("строка {}: ожидалось facet normal", line + 1));
            }
            numbers::<3>(words).map_err(|e| format!("строка {}: {e}", line + 1))?;
            let expect = |next: Option<(usize, &str)>, value| match next {
                Some((_, raw)) if raw.split_whitespace().eq(value) => Ok(()),
                Some((i, _)) => Err(format!("строка {}: неверная структура ASCII STL", i + 1)),
                None => Err("незавершённая грань ASCII STL".into()),
            };
            expect(lines.next(), ["outer", "loop"].as_slice().iter().copied())?;
            let mut triangle = [[0.0; 3]; 3];
            for vertex in &mut triangle {
                let (i, raw) = lines.next().ok_or("незавершённая грань ASCII STL")?;
                let mut words = raw.split_whitespace();
                if words.next() != Some("vertex") {
                    return Err(format!("строка {}: ожидалось vertex", i + 1));
                }
                *vertex = numbers(words).map_err(|e| format!("строка {}: {e}", i + 1))?;
            }
            expect(lines.next(), ["endloop"].as_slice().iter().copied())?;
            expect(lines.next(), ["endfacet"].as_slice().iter().copied())?;
            add(triangle);
        }
    }
    Ok(mesh)
}

impl Mesh {
    pub(crate) fn validate(&self) -> Result<(), String> {
        if self.vertices_m.len() < 3 || self.faces.is_empty() {
            return Err("сетка пуста".into());
        }
        for (i, v) in self.vertices_m.iter().enumerate() {
            // MuJoCo хранит координаты сетки в float32.
            if !v.iter().all(|x| (*x as f32).is_finite()) {
                return Err(format!("vertices_m[{i}]: координата вне диапазона float32"));
            }
        }
        for (i, f) in self.faces.iter().enumerate() {
            if f.iter().any(|j| *j >= self.vertices_m.len()) {
                return Err(format!("faces[{i}]: индекс вне массива вершин"));
            }
            let [a, b, c] =
                f.map(|j| Vector3::from(self.vertices_m[j].map(|x| f64::from(x as f32))));
            let area = (b - a).cross(&(c - a)).norm();
            if !(area.is_finite() && area > 0.0) {
                return Err(format!("faces[{i}]: вырожденная грань"));
            }
        }
        Ok(())
    }

    fn add_face(&mut self, face: &[usize]) -> Result<(), String> {
        if face.len() < 3 {
            return Err("грань имеет меньше трёх вершин".into());
        }
        if face.len() > 3 {
            // Допуск округления для проверки планарности относительно масштаба грани.
            const PLANAR_ULPS: f64 = 64.0 * f64::EPSILON;
            let origin = Vector3::from(self.vertices_m[face[0]]);
            let points: Vec<_> = face
                .iter()
                .map(|i| Vector3::from(self.vertices_m[*i]) - origin)
                .collect();
            let normal = points[1]
                .cross(&points[2])
                .try_normalize(0.0)
                .ok_or("вырожденный многоугольник")?;
            let scale = points.iter().map(Vector3::norm).fold(0.0, f64::max);
            if !scale.is_finite()
                || points
                    .iter()
                    .any(|p| normal.dot(p).abs() > PLANAR_ULPS * scale)
            {
                return Err("неплоская грань, задайте треугольники".into());
            }
            // У выпуклой грани каждая ориентированная сторона видит остальные вершины слева.
            for (&a, &b) in points.iter().zip(points.iter().cycle().skip(1)) {
                if points
                    .iter()
                    .any(|p| (b - a).cross(&(p - a)).dot(&normal) < 0.0)
                {
                    return Err(
                        "невыпуклая или самопересекающаяся грань, задайте треугольники".into(),
                    );
                }
            }
        }
        self.faces
            .extend(face[1..].windows(2).map(|w| [face[0], w[0], w[1]]));
        Ok(())
    }

    pub(crate) fn require_closed(&self) -> Result<(), String> {
        let mut edges = BTreeMap::new();
        for &[a, b, c] in &self.faces {
            for (from, to) in [(a, b), (b, c), (c, a)] {
                let counts = edges
                    .entry((from.min(to), from.max(to)))
                    .or_insert([0_u32; 2]);
                counts[usize::from(from > to)] += 1;
            }
        }
        for ((a, b), counts) in edges {
            if counts != [1, 1] {
                return Err(format!(
                    "ребро {a}–{b}: для объёма нужны две противоположно ориентированные грани"
                ));
            }
        }
        Ok(())
    }
}
