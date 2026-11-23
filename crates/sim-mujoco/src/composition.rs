//! Композиция локальных MJCF-моделей с сохранением имён и проверкой перезагрузки.

use std::collections::{BTreeMap, BTreeSet};
use std::ffi::{CStr, CString, c_char};
use std::path::{Path, PathBuf};
use std::ptr::null;

use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};

use crate::ffi::{self, mjtObj as Obj};
use crate::{Sim, Spec, cstr_or_empty};

#[derive(Clone, Copy, Debug)]
pub struct MjcfFragment<'a> {
    pub prefix: &'a str,
    /// Имя исходного файла (может быть виртуальным), родитель задаёт каталог ресурсов.
    pub source: &'a Path,
    /// Полный MJCF с уже развёрнутыми включениями.
    pub xml: &'a str,
    pub position_m: [f64; 3],
    pub quat_wxyz: [f64; 4],
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ComposedMjcf {
    pub xml: String,
    /// Абсолютные пути из итогового MJCF и отпечатки байтов ресурсов.
    pub resources: BTreeMap<PathBuf, u64>,
}

// FNV-1a, 64 бита: https://www.ietf.org/archive/id/draft-eastlake-fnv-25.html
pub fn content_fingerprint(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf29ce484222325, |hash, byte| {
        (hash ^ u64::from(*byte)).wrapping_mul(0x100000001b3)
    })
}

// Нативный читатель MJCF допускает NaN как маркер незаданных значений.
// Проверяем явно написанные атрибуты до передачи в движок, который проверяет XML целиком.
fn validate_input(mut text: &str, fragment: bool) -> Result<(), String> {
    while let Some((_, markup)) = text.split_once('<') {
        if let Some(comment) = markup.strip_prefix("!--") {
            text = comment
                .split_once("-->")
                .ok_or("незакрытый комментарий MJCF")?
                .1;
            continue;
        }
        if let Some(cdata) = markup.strip_prefix("![CDATA[") {
            text = cdata.split_once("]]>").ok_or("незакрытый раздел CDATA")?.1;
            continue;
        }
        if let Some(pi) = markup.strip_prefix("?xml ") {
            text = pi.split_once("?>").ok_or("незакрытая декларация XML")?.1;
            continue;
        }
        let end = markup
            .find(|c: char| c.is_ascii_whitespace() || matches!(c, '>' | '/'))
            .ok_or("незакрытый тег MJCF")?;
        if end == 0 && markup.starts_with('/') {
            text = markup.split_once('>').ok_or("незакрытый тег MJCF")?.1;
            continue;
        }
        let tag = &markup[..end];
        if tag.starts_with(['!', '?'])
            || matches!(tag, "include" | "attach" | "model" | "plugin" | "extension")
            || (fragment
                && matches!(
                    tag,
                    "size"
                        | "visual"
                        | "statistic"
                        | "keyframe"
                        | "deformable"
                        | "composite"
                        | "flexcomp"
                ))
        {
            return Err(format!("неподдерживаемая секция MJCF «{tag}»"));
        }
        let mut attributes = &markup[end..];
        loop {
            attributes = attributes.trim_start();
            if let Some(tail) = attributes
                .strip_prefix('>')
                .or_else(|| attributes.strip_prefix("/>"))
            {
                text = tail;
                break;
            }
            let (key, value) = attributes
                .split_once('=')
                .ok_or("некорректный атрибут MJCF")?;
            let key = key.trim();
            let value = value.trim_start();
            let delimiter = value
                .chars()
                .next()
                .filter(|c| matches!(c, '\'' | '"'))
                .ok_or("атрибут MJCF должен быть в кавычках")?;
            let (value, tail) = value[1..]
                .split_once(delimiter)
                .ok_or("незакрытый атрибут MJCF")?;
            let decoded = decode_numeric_entities(value)?;
            if !matches!(
                key,
                "name"
                    | "model"
                    | "class"
                    | "childclass"
                    | "file"
                    | "meshdir"
                    | "texturedir"
                    | "assetdir"
            ) && !(tag == "text" && key == "data")
                && decoded
                    .split_whitespace()
                    .any(|token| token.parse::<f64>().is_ok_and(|v| !v.is_finite()))
            {
                return Err(format!("неконечное значение в {tag}.{key}"));
            }
            attributes = tail;
        }
    }
    Ok(())
}

fn decode_numeric_entities(value: &str) -> Result<std::borrow::Cow<'_, str>, String> {
    if !value.contains("&#") {
        return Ok(value.into());
    }
    let mut decoded = String::new();
    let mut rest = value;
    while let Some((text, reference)) = rest.split_once("&#") {
        let (code, tail) = reference.split_once(';').ok_or("незакрытая ссылка XML")?;
        let code = match code.strip_prefix('x') {
            Some(hex) => u32::from_str_radix(hex, 16),
            None => code.parse(),
        }
        .ok()
        .and_then(char::from_u32)
        .ok_or("некорректная ссылка XML")?;
        decoded.push_str(text);
        decoded.push(code);
        rest = tail;
    }
    decoded.push_str(rest);
    Ok(decoded.into())
}

fn parse(xml: &str) -> Result<Spec, String> {
    let xml = CString::new(xml).map_err(|e| e.to_string())?;
    // Буфер диагностики того же размера, что при обычной загрузке Sim.
    let mut error = [0 as c_char; 2048];
    // SAFETY: строки и буфер живут до возврата из MuJoCo.
    let ptr = unsafe {
        ffi::mj_parseXMLString(xml.as_ptr(), null(), error.as_mut_ptr(), error.len() as i32)
    };
    if ptr.is_null() {
        return Err(cstr_or_empty(error.as_ptr()));
    }
    Ok(Spec(ptr))
}

fn compile(spec: &Spec) -> Result<Sim, String> {
    // SAFETY: Spec владеет живым mjSpec. Скомпилированная модель передаётся Sim.
    let model = unsafe { ffi::mj_compile(spec.0, null()) };
    if model.is_null() {
        return Err(spec.error());
    }
    let sim = Sim::from_model(model)?;
    if !sim.numerically_ok() {
        return Err("численная авария в начальной конфигурации".into());
    }
    Ok(sim)
}

fn serialize(spec: &Spec) -> Result<String, String> {
    let mut error = [0 as c_char; 2048];
    let write = |out: &mut [c_char], error: &mut [c_char]| {
        // SAFETY: MuJoCo получает действительные буферы с указанными размерами.
        unsafe {
            ffi::mj_saveXMLString(
                spec.0,
                out.as_mut_ptr(),
                out.len() as i32,
                error.as_mut_ptr(),
                error.len() as i32,
            )
        }
    };
    let mut probe = [0];
    let size = write(&mut probe, &mut error);
    if size <= 0 {
        return Err(cstr_or_empty(error.as_ptr()));
    }
    let capacity = size.checked_add(1).ok_or("слишком большой MJCF")?;
    let mut out = vec![0; capacity as usize];
    if write(&mut out, &mut error) != 0 {
        return Err(cstr_or_empty(error.as_ptr()));
    }
    // SAFETY: успешный mj_saveXMLString записывает строку с завершающим нулём.
    unsafe { CStr::from_ptr(out.as_ptr()) }
        .to_str()
        .map(str::to_owned)
        .map_err(|e| e.to_string())
}

fn elements(spec: &Spec, kind: Obj::Type) -> impl Iterator<Item = *mut ffi::mjsElement> + '_ {
    // SAFETY: обход только читает список, Spec живёт дольше итератора.
    let first = unsafe { ffi::mjs_firstElement(spec.0, kind) };
    std::iter::successors((!first.is_null()).then_some(first), move |&element| {
        let next = unsafe { ffi::mjs_nextElement(spec.0, element) };
        (!next.is_null()).then_some(next)
    })
}

fn resolve_assets(
    spec: &mut Spec,
    source: &Path,
    resources: &mut BTreeMap<PathBuf, u64>,
) -> Result<(), String> {
    // SAFETY: строки принадлежат живому Spec, полученные указатели используются только здесь.
    unsafe {
        let compiler = &(*spec.0).compiler;
        let mesh_dir = cstr_or_empty(ffi::mjs_getString(compiler.meshdir));
        let texture_dir = cstr_or_empty(ffi::mjs_getString(compiler.texturedir));
        let strip = (*spec.0).strippath;
        let root = source.parent().unwrap_or(Path::new(""));
        let mut resolve = |dir: &str, file: &str| -> Result<Option<CString>, String> {
            if file.is_empty() {
                return Ok(None);
            }
            let path = Path::new(file);
            let path = if strip {
                Path::new(path.file_name().ok_or("неверное имя ресурса")?)
            } else {
                path
            };
            let path = root.join(dir).join(path);
            let absolute = std::path::absolute(&path).map_err(|e| e.to_string())?;
            let bytes = std::fs::read(&absolute)
                .map_err(|e| format!("ресурс {}: {e}", absolute.display()))?;
            let stamp = content_fingerprint(&bytes);
            if resources
                .insert(absolute.clone(), stamp)
                .is_some_and(|old| old != stamp)
            {
                return Err(format!(
                    "ресурс {} изменился при сборке",
                    absolute.display()
                ));
            }
            // Имя по исходному расширению необходимо для выбора декодера, в том числе у ссылки.
            let text = absolute.to_str().ok_or("не UTF-8 путь ресурса")?;
            Ok(Some(CString::new(text).map_err(|e| e.to_string())?))
        };
        let mut resolve_file = |dir: &str, file: *mut ffi::mjString| -> Result<(), String> {
            if let Some(path) = resolve(dir, &cstr_or_empty(ffi::mjs_getString(file)))? {
                ffi::mjs_setString(file, path.as_ptr());
            }
            Ok(())
        };
        for element in elements(spec, Obj::mjOBJ_MESH) {
            resolve_file(&mesh_dir, (*ffi::mjs_asMesh(element)).file)?;
        }
        for element in elements(spec, Obj::mjOBJ_HFIELD) {
            resolve_file(&mesh_dir, (*ffi::mjs_asHField(element)).file)?;
        }
        for element in elements(spec, Obj::mjOBJ_TEXTURE) {
            let texture = &*ffi::mjs_asTexture(element);
            let cube = ffi::mjs_getInStringVec(texture.cubefiles, 0);
            if !cube.is_null()
                && (0..6).any(|i| {
                    !cstr_or_empty(ffi::mjs_getInStringVec(texture.cubefiles, i)).is_empty()
                })
            {
                return Err("раздельные файлы граней текстуры не поддержаны в композиции".into());
            }
            resolve_file(&texture_dir, texture.file)?;
        }
        // Все пути теперь абсолютные. Иначе дочерний каталог может попасть в общий MJCF.
        (*spec.0).strippath = false;
        ffi::mjs_setString(compiler.meshdir, c"".as_ptr());
        ffi::mjs_setString(compiler.texturedir, c"".as_ptr());
    }
    Ok(())
}

fn transform(pos: [f64; 3], quat: [f64; 4]) -> Result<Isometry3<f64>, String> {
    if !pos.into_iter().chain(quat).all(f64::is_finite) {
        return Err("поза: требуется конечное число".into());
    }
    let scale = quat.into_iter().map(f64::abs).fold(0.0, f64::max);
    if scale == 0.0 {
        return Err("поза: нулевой кватернион".into());
    }
    let [w, x, y, z] = quat.map(|v| v / scale);
    Ok(Isometry3::from_parts(
        Translation3::from(pos),
        UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z)),
    ))
}

fn attach(
    parent: &mut Spec,
    child: &Spec,
    prefix: &str,
    pose: &Isometry3<f64>,
) -> Result<(), String> {
    let prefix = CString::new(format!("{prefix}/")).map_err(|e| e.to_string())?;
    // SAFETY: элементы принадлежат своим живым Spec. Deep copy оставляет исходную модель у child.
    unsafe {
        let world = ffi::mjs_findBody(parent.0, c"world".as_ptr());
        if world.is_null() {
            return Err("нет корня MJCF".into());
        }
        let frame = ffi::mjs_addFrame(world, std::ptr::null_mut());
        if frame.is_null() {
            return Err(parent.error());
        }
        (*frame).pos = pose.translation.vector.into();
        let q = pose.rotation.quaternion();
        (*frame).quat = [q.w, q.i, q.j, q.k];
        if ffi::mjs_setDeepCopy(parent.0, 1) != 0
            || ffi::mjs_attach(
                (*frame).element,
                (*child.0).element,
                prefix.as_ptr(),
                c"".as_ptr(),
            )
            .is_null()
        {
            return Err(parent.error());
        }
    }
    Ok(())
}

fn check_supported(spec: &Spec) -> Result<(), String> {
    for kind in [
        Obj::mjOBJ_FLEX,
        Obj::mjOBJ_SKIN,
        Obj::mjOBJ_KEY,
        Obj::mjOBJ_PLUGIN,
        Obj::mjOBJ_PAIR,
    ] {
        if elements(spec, kind).next().is_some() {
            // SAFETY: типы перечислены в заголовке MuJoCo.
            let kind = cstr_or_empty(unsafe { ffi::mju_type2Str(kind as i32) });
            return Err(format!("композиция не поддерживает секцию «{kind}»"));
        }
    }
    Ok(())
}

// Настройки массы у элемента локальны, но у MJCF только один compiler. Если
// настройки различаются, записываем вычисленные локальные инерции явно и
// проверяем общий результат по исходной скомпилированной модели.
fn carry_inertia(parent: &Spec, child: &mut Spec, sim: &Sim) -> Result<(), String> {
    // SAFETY: всё принадлежит живым спецификациям или скомпилированному Sim.
    unsafe {
        let a = &(*parent.0).compiler;
        let b = &(*child.0).compiler;
        if a.boundmass == b.boundmass
            && a.boundinertia == b.boundinertia
            && a.inertiafromgeom == b.inertiafromgeom
            && a.inertiagrouprange == b.inertiagrouprange
            && a.balanceinertia == b.balanceinertia
        {
            return Ok(());
        }
        if a.inertiafromgeom == ffi::mjtInertiaFromGeom::mjINERTIAFROMGEOM_TRUE {
            return Err("compiler мира требует перезаписи явных инерций геометрией".into());
        }
        for element in elements(child, Obj::mjOBJ_BODY) {
            let name = cstr_or_empty(ffi::mjs_getString(ffi::mjs_getName(element)));
            let id = sim
                .id(Obj::mjOBJ_BODY, &name)
                .ok_or_else(|| format!("нет тела «{name}»"))?;
            if id == 0 {
                continue;
            }
            let body = &mut *ffi::mjs_asBody(element);
            body.mass = sim.body_mass(id);
            body.ipos = *sim.model().body_ipos.add(3 * id).cast();
            body.iquat = *sim.model().body_iquat.add(4 * id).cast();
            body.inertia = sim.body_inertia(id);
            body.ialt.type_ = ffi::mjtOrientation::mjORIENTATION_QUAT;
            // Не заданный fullinertia хранится в MjSpec как NaN. В XML попадает diaginertia.
            body.fullinertia = [f64::NAN; 6];
            body.explicitinertial = true;
        }
        let b = &mut (*child.0).compiler;
        b.boundmass = a.boundmass;
        b.boundinertia = a.boundinertia;
        b.inertiafromgeom = a.inertiafromgeom;
        b.inertiagrouprange = a.inertiagrouprange;
        b.balanceinertia = a.balanceinertia;
    }
    Ok(())
}

fn check_compiler(a: &ffi::mjsCompiler, dest: &Spec) -> Result<(), String> {
    // SAFETY: Spec жив, поля здесь только читаются.
    let b = unsafe { &(*dest.0).compiler };
    if a.autolimits != b.autolimits
        || a.boundinertia != b.boundinertia
        || a.boundmass != b.boundmass
        || a.balanceinertia != b.balanceinertia
        || a.inertiafromgeom != b.inertiafromgeom
        || a.inertiagrouprange != b.inertiagrouprange
        || a.fusestatic != b.fusestatic
        || a.discardvisual != b.discardvisual
        || a.alignfree != b.alignfree
        || a.usethread != b.usethread
    {
        return Err("перезагрузка изменила настройки compiler мира".into());
    }
    Ok(())
}

fn check_options(a: &ffi::mjOption, b: &ffi::mjOption) -> Result<(), String> {
    macro_rules! fields {
        ($($field:ident),+) => {
            $(if a.$field != b.$field {
                return Err(format!("композиция изменила настройку мира «{}»", stringify!($field)));
            })+
        };
    }
    fields!(
        timestep,
        impratio,
        tolerance,
        ls_tolerance,
        noslip_tolerance,
        ccd_tolerance,
        sleep_tolerance,
        gravity,
        wind,
        magnetic,
        density,
        viscosity,
        o_margin,
        o_solref,
        o_solimp,
        o_friction,
        integrator,
        cone,
        jacobian,
        solver,
        iterations,
        ls_iterations,
        noslip_iterations,
        ccd_iterations,
        disableflags,
        enableflags,
        disableactuator,
        sdf_initpoints,
        sdf_iterations
    );
    Ok(())
}

/// Присоединяет каждую модель целиком в собственном пространстве имён.
/// Сериализованный MJCF повторно компилируется в том же режиме, что при загрузке из рантайма.
pub fn compose_mjcf(base: &str, fragments: &[MjcfFragment<'_>]) -> Result<ComposedMjcf, String> {
    validate_input(base, false)?;
    let mut parent = parse(base)?;
    // SAFETY: параметры компиляции принадлежат ещё не скомпилированному Spec.
    unsafe { (*parent.0).compiler.usethread = false };
    // SAFETY: Spec жив. Буферы путей не используются при сравнении этих значений.
    let world_compiler = unsafe { (*parent.0).compiler };
    let before = compile(&parent)?;
    let mut resources = BTreeMap::new();
    let mut prefixes = BTreeSet::new();
    let mut expected = Vec::new();
    for fragment in fragments {
        let prefix = fragment.prefix;
        if prefix.is_empty()
            || prefix.contains('/')
            || prefix.chars().any(|c| c.is_whitespace() || c.is_control())
        {
            return Err(format!("недопустимый префикс «{prefix}»"));
        }
        if !prefixes.insert(prefix) || has_prefix(&before, prefix) {
            return Err(format!("конфликт префикса «{prefix}»"));
        }
        let mut add = || -> Result<_, String> {
            let pose = transform(fragment.position_m, fragment.quat_wxyz)?;
            validate_input(fragment.xml, true)?;
            let mut child = parse(fragment.xml)?;
            check_supported(&child)?;
            resolve_assets(&mut child, fragment.source, &mut resources)?;
            // SAFETY: каждый дочерний Spec принадлежит этой итерации и ещё не присоединён.
            unsafe {
                (*child.0).compiler.usethread = false;
                if (*child.0).compiler.fusestatic || (*child.0).compiler.discardvisual {
                    return Err(
                        "локальная модель должна сохранять тела и визуальную геометрию".into(),
                    );
                }
                // Опции решателя и среды относятся ко всему миру.
                (*child.0).option = (*parent.0).option;
                (*child.0).authored.option = (*parent.0).authored.option;
                (*child.0).authored.enableflags = (*parent.0).authored.enableflags;
                (*child.0).authored.disableflags = (*parent.0).authored.disableflags;
                (*child.0).authored.disableactuator = (*parent.0).authored.disableactuator;
            }
            let standalone = compile(&child)?;
            check_named(&standalone)?;
            carry_inertia(&parent, &mut child, &standalone)?;
            attach(&mut parent, &child, prefix, &pose)?;
            Ok((prefix, pose, standalone, child))
        };
        expected.push(add().map_err(|e| format!("{} ({prefix}): {e}", fragment.source.display()))?);
    }
    let joined = compile(&parent)?;
    check_options(&before.model().opt, &joined.model().opt)?;
    check_compiler(&world_compiler, &parent)?;
    check_inventory(&before, expected.iter().map(|(_, _, sim, _)| sim), &joined)?;
    check_copy(&before, &joined, "", &Isometry3::identity())?;
    for (prefix, pose, standalone, _) in &expected {
        check_copy(standalone, &joined, &format!("{prefix}/"), pose)?;
    }
    // SAFETY: флаги записи не меняют скомпилированную физическую модель.
    unsafe {
        (*parent.0).compiler.savecanonical = true;
        (*parent.0).compiler.savecompiled = false;
    }
    let xml = serialize(&parent)?;
    let reloaded_spec = parse(&xml)?;
    check_compiler(&world_compiler, &reloaded_spec)?;
    let reloaded = compile(&reloaded_spec)?;
    check_inventory(&joined, std::iter::empty(), &reloaded)?;
    check_options(&before.model().opt, &reloaded.model().opt)?;
    check_copy(&joined, &reloaded, "", &Isometry3::identity())?;
    for (path, &stamp) in &resources {
        let bytes = std::fs::read(path).map_err(|e| format!("ресурс {}: {e}", path.display()))?;
        if content_fingerprint(&bytes) != stamp {
            return Err(format!("ресурс {} изменился при сборке", path.display()));
        }
    }
    Ok(ComposedMjcf { xml, resources })
}

fn objects(sim: &Sim) -> [(Obj::Type, usize); 17] {
    let m = sim.model();
    [
        (Obj::mjOBJ_BODY, sim.nbody()),
        (Obj::mjOBJ_JOINT, sim.njnt()),
        (Obj::mjOBJ_GEOM, sim.ngeom()),
        (Obj::mjOBJ_SITE, m.nsite as usize),
        (Obj::mjOBJ_CAMERA, m.ncam as usize),
        (Obj::mjOBJ_LIGHT, m.nlight as usize),
        (Obj::mjOBJ_TENDON, sim.ntendon()),
        (Obj::mjOBJ_EQUALITY, m.neq as usize),
        (Obj::mjOBJ_SENSOR, m.nsensor as usize),
        (Obj::mjOBJ_ACTUATOR, sim.nactuator()),
        (Obj::mjOBJ_MESH, sim.nmesh()),
        (Obj::mjOBJ_HFIELD, m.nhfield as usize),
        (Obj::mjOBJ_MATERIAL, m.nmat as usize),
        (Obj::mjOBJ_TEXTURE, m.ntex as usize),
        (Obj::mjOBJ_NUMERIC, m.nnumeric as usize),
        (Obj::mjOBJ_TEXT, m.ntext as usize),
        (Obj::mjOBJ_TUPLE, m.ntuple as usize),
    ]
}

fn check_inventory<'a>(
    source: &Sim,
    fragments: impl Iterator<Item = &'a Sim>,
    dest: &Sim,
) -> Result<(), String> {
    let mut expected = objects(source);
    let mut dofs = source.nv();
    let mut excludes = source.model().nexclude;
    for child in fragments {
        for ((kind, sum), (_, n)) in expected.iter_mut().zip(objects(child)) {
            *sum += n - usize::from(*kind == Obj::mjOBJ_BODY);
        }
        dofs += child.nv();
        excludes += child.model().nexclude;
    }
    if expected != objects(dest) || dofs != dest.nv() || excludes != dest.model().nexclude {
        return Err("композиция изменила инвентарь модели".into());
    }
    Ok(())
}

fn has_prefix(sim: &Sim, prefix: &str) -> bool {
    objects(sim).into_iter().any(|(kind, count)| {
        (0..count).any(|i| sim.name(kind, i).split('/').next() == Some(prefix))
    })
}

fn check_named(sim: &Sim) -> Result<(), String> {
    if sim.nbody() <= 1 {
        return Err("локальная модель не содержит тел".into());
    }
    for (kind, count) in objects(sim) {
        let start = usize::from(kind == Obj::mjOBJ_BODY);
        for i in start..count {
            if sim.name(kind, i).is_empty() {
                // SAFETY: kind входит в перечень типов mjtObj.
                let kind = cstr_or_empty(unsafe { ffi::mju_type2Str(kind as i32) });
                return Err(format!("локальному объекту «{kind}» №{i} требуется имя"));
            }
        }
    }
    Ok(())
}

// Запас для смены базиса, нормализации кватернионов и вычисления опорной кинематики.
const ROUNDING: f64 = 4096.0 * f64::EPSILON;

fn close(a: &[f64], b: &[f64], scale: f64) -> bool {
    a.len() == b.len()
        && a.iter().zip(b).all(|(&a, &b)| {
            a.is_finite()
                && b.is_finite()
                && (a - b).abs() <= ROUNDING * scale.max(a.abs()).max(b.abs())
        })
}

fn check_copy(source: &Sim, dest: &Sim, prefix: &str, pose: &Isometry3<f64>) -> Result<(), String> {
    for (kind, count) in objects(source) {
        let start = usize::from(kind == Obj::mjOBJ_BODY);
        for i in start..count {
            let local = source.name(kind, i);
            if local.is_empty() {
                continue;
            }
            let name = format!("{prefix}{local}");
            let j = dest
                .id(kind, &name)
                .ok_or_else(|| format!("потеряно имя «{name}»"))?;
            match kind {
                Obj::mjOBJ_BODY => {
                    let (p, q) = source.body_pose(i);
                    let (actual_p, actual_q) = dest.body_pose(j);
                    let expected = pose * transform(p, q)?;
                    let actual = transform(actual_p, actual_q)?;
                    if !close(expected.translation.vector.as_slice(), &actual_p, 1.0)
                        || !close(
                            expected.rotation.to_rotation_matrix().matrix().as_slice(),
                            actual.rotation.to_rotation_matrix().matrix().as_slice(),
                            1.0,
                        )
                        || !close(&[source.body_mass(i)], &[dest.body_mass(j)], 0.0)
                        || !close(&source.body_inertia(i), &dest.body_inertia(j), 0.0)
                    {
                        return Err(format!("изменились поза, масса или инерция «{name}»"));
                    }
                }
                Obj::mjOBJ_TENDON
                    if !close(&[source.tendon_length(i)], &[dest.tendon_length(j)], 1.0) =>
                {
                    return Err(format!("изменилась длина связи «{name}»"));
                }
                _ => {}
            }
            if !same_parameters(source, dest, kind, i, j, prefix, pose)? {
                return Err(format!("изменились параметры или ссылки «{name}»"));
            }
        }
    }
    // В MuJoCo исключения хранятся по паре тел, имя записи необязательно.
    let exclusions =
        |sim: &Sim| Sim::arr(sim.model().exclude_signature, sim.model().nexclude as usize);
    for signature in exclusions(source) {
        let signature = *signature as u32;
        let body1 = renamed_id(
            source,
            dest,
            Obj::mjOBJ_BODY,
            (signature >> 16) as i32,
            prefix,
        )?;
        let body2 = renamed_id(
            source,
            dest,
            Obj::mjOBJ_BODY,
            (signature & u32::from(u16::MAX)) as i32,
            prefix,
        )?;
        let expected = (body1.min(body2) << 16) | body1.max(body2);
        if !exclusions(dest).contains(&expected) {
            return Err("потеряно исключение контактной пары".into());
        }
    }
    Ok(())
}

fn renamed_id(
    source: &Sim,
    dest: &Sim,
    kind: Obj::Type,
    id: i32,
    prefix: &str,
) -> Result<i32, String> {
    let kind = if kind == Obj::mjOBJ_XBODY {
        Obj::mjOBJ_BODY
    } else {
        kind
    };
    if id < 0 || kind == Obj::mjOBJ_UNKNOWN || (kind == Obj::mjOBJ_BODY && id == 0) {
        return Ok(id);
    }
    let source_name = source.name(kind, id as usize);
    if source_name.is_empty() && prefix.is_empty() {
        return Ok(id);
    }
    let name = format!("{prefix}{source_name}");
    dest.id(kind, &name)
        .map(|i| i as i32)
        .ok_or_else(|| format!("потеряна ссылка «{name}»"))
}

fn same_parameters(
    source: &Sim,
    dest: &Sim,
    kind: Obj::Type,
    i: usize,
    j: usize,
    prefix: &str,
    pose: &Isometry3<f64>,
) -> Result<bool, String> {
    let a = source.model();
    let b = dest.model();
    let map = |kind, id| renamed_id(source, dest, kind, id, prefix);
    // SAFETY: i и j получены из моделей для kind. Ширины строк соответствуют mjModel.
    unsafe {
        macro_rules! same {
            ($($field:ident),+ $(,)?) => {
                $(if *a.$field.add(i) != *b.$field.add(j) { return Ok(false); })+
            };
        }
        macro_rules! numbers {
            ($($field:ident : $width:expr),+ $(,)?) => {
                $(if !close(
                    Sim::arr(a.$field.add(i * $width), $width),
                    Sim::arr(b.$field.add(j * $width), $width),
                    0.0
                ) { return Ok(false); })+
            };
        }
        macro_rules! reference {
            ($field:ident, $kind:expr) => {
                if map($kind, *a.$field.add(i))? != *b.$field.add(j) {
                    return Ok(false);
                }
            };
        }
        // xmat хранит матрицы в порядке строк. Это также проверяет позу
        // объектов, непосредственно закреплённых в мировом теле.
        let world_pose = |pa: *const f64, ra: *const f64, pb: *const f64, rb: *const f64| {
            let pa = nalgebra::Point3::from(*pa.add(3 * i).cast::<[f64; 3]>());
            let ra = nalgebra::Matrix3::from_row_slice(Sim::arr(ra.add(9 * i), 9));
            let pb = Sim::arr(pb.add(3 * j), 3);
            let rb = nalgebra::Matrix3::from_row_slice(Sim::arr(rb.add(9 * j), 9));
            let expected = pose.rotation.to_rotation_matrix().matrix() * ra;
            close((pose * pa).coords.as_slice(), pb, 1.0)
                && close(expected.as_slice(), rb.as_slice(), 1.0)
        };
        use ffi::{mjNIMP, mjNPOLY, mjNREF, mjtWrap};
        match kind {
            Obj::mjOBJ_BODY => {
                reference!(body_parentid, Obj::mjOBJ_BODY);
                numbers!(body_ipos: 3, body_gravcomp: 1);
                // Главные оси могут отличаться знаками. Сравниваем тензор в кадре тела.
                let tensor = |m: &ffi::mjModel, k| -> Result<_, String> {
                    let q = *m.body_iquat.add(k * 4).cast::<[f64; 4]>();
                    let rotation = transform([0.0; 3], q)?.rotation.to_rotation_matrix();
                    let diag = nalgebra::Matrix3::from_diagonal(
                        &(*m.body_inertia.add(k * 3).cast::<[f64; 3]>()).into(),
                    );
                    Ok(rotation.matrix() * diag * rotation.matrix().transpose())
                };
                let a = tensor(a, i)?;
                let b = tensor(b, j)?;
                if !close(a.as_slice(), b.as_slice(), a.amax().max(b.amax())) {
                    return Ok(false);
                }
            }
            Obj::mjOBJ_JOINT => {
                reference!(jnt_bodyid, Obj::mjOBJ_BODY);
                same!(jnt_type, jnt_limited, jnt_actfrclimited, jnt_actgravcomp);
                numbers!(jnt_range: 2, jnt_stiffness: 1, jnt_stiffnesspoly: mjNPOLY as usize,
                    jnt_actfrcrange: 2, jnt_solref: mjNREF as usize, jnt_solimp: mjNIMP as usize,
                    jnt_margin: 1);
                let dims = match *a.jnt_type.add(i) as u32 {
                    ffi::mjtJoint::mjJNT_FREE => 6,
                    ffi::mjtJoint::mjJNT_BALL => 3,
                    _ => 1,
                };
                let da = *a.jnt_dofadr.add(i) as usize;
                let db = *b.jnt_dofadr.add(j) as usize;
                for (a, b, n) in [
                    (a.dof_armature.add(da), b.dof_armature.add(db), dims),
                    (a.dof_damping.add(da), b.dof_damping.add(db), dims),
                    (a.dof_frictionloss.add(da), b.dof_frictionloss.add(db), dims),
                    (
                        a.dof_dampingpoly.add(da * mjNPOLY as usize),
                        b.dof_dampingpoly.add(db * mjNPOLY as usize),
                        dims * mjNPOLY as usize,
                    ),
                ] {
                    if !close(Sim::arr(a, n), Sim::arr(b, n), 0.0) {
                        return Ok(false);
                    }
                }
                if dims == 1 {
                    numbers!(jnt_pos: 3, jnt_axis: 3);
                    let axis = pose.rotation * nalgebra::Vector3::from(source.joint_axis(i));
                    if !close(axis.as_slice(), &dest.joint_axis(j), 1.0) {
                        return Ok(false);
                    }
                    if !close(
                        &[source.qpos0()[*a.jnt_qposadr.add(i) as usize]],
                        &[dest.qpos0()[*b.jnt_qposadr.add(j) as usize]],
                        0.0,
                    ) {
                        return Ok(false);
                    }
                }
            }
            Obj::mjOBJ_GEOM => {
                reference!(geom_bodyid, Obj::mjOBJ_BODY);
                reference!(geom_matid, Obj::mjOBJ_MATERIAL);
                same!(
                    geom_type,
                    geom_contype,
                    geom_conaffinity,
                    geom_condim,
                    geom_group,
                    geom_priority
                );
                if *a.geom_bodyid.add(i) != 0 || prefix.is_empty() {
                    numbers!(geom_pos: 3, geom_quat: 4);
                }
                numbers!(geom_size: 3, geom_friction: 3, geom_solmix: 1, geom_solref: mjNREF as usize,
                    geom_solimp: mjNIMP as usize, geom_margin: 1, geom_gap: 1, geom_adhesion: 1,
                    geom_surfacevel: 6);
                if !world_pose(
                    source.data().geom_xpos,
                    source.data().geom_xmat,
                    dest.data().geom_xpos,
                    dest.data().geom_xmat,
                ) {
                    return Ok(false);
                }
                let asset = match *a.geom_type.add(i) as u32 {
                    ffi::mjtGeom::mjGEOM_MESH => Obj::mjOBJ_MESH,
                    ffi::mjtGeom::mjGEOM_HFIELD => Obj::mjOBJ_HFIELD,
                    _ => Obj::mjOBJ_UNKNOWN,
                };
                reference!(geom_dataid, asset);
            }
            Obj::mjOBJ_SITE => {
                reference!(site_bodyid, Obj::mjOBJ_BODY);
                same!(site_type, site_group);
                if *a.site_bodyid.add(i) != 0 || prefix.is_empty() {
                    numbers!(site_pos: 3, site_quat: 4);
                }
                numbers!(site_size: 3);
                if !world_pose(
                    source.data().site_xpos,
                    source.data().site_xmat,
                    dest.data().site_xpos,
                    dest.data().site_xmat,
                ) {
                    return Ok(false);
                }
            }
            Obj::mjOBJ_EQUALITY => {
                same!(eq_type, eq_objtype, eq_active0);
                let object = *a.eq_objtype.add(i) as u32;
                let kind = if object != Obj::mjOBJ_UNKNOWN {
                    object
                } else {
                    match *a.eq_type.add(i) as u32 {
                        ffi::mjtEq::mjEQ_JOINT => Obj::mjOBJ_JOINT,
                        ffi::mjtEq::mjEQ_TENDON => Obj::mjOBJ_TENDON,
                        _ => Obj::mjOBJ_BODY,
                    }
                };
                reference!(eq_obj1id, kind);
                reference!(eq_obj2id, kind);
                numbers!(eq_solref: mjNREF as usize, eq_solimp: mjNIMP as usize);
                // Привязка к world может менять мировой якорь при размещении.
                if kind != Obj::mjOBJ_BODY || *a.eq_obj2id.add(i) > 0 || prefix.is_empty() {
                    numbers!(eq_data: ffi::mjNEQDATA as usize);
                }
            }
            Obj::mjOBJ_TENDON => {
                same!(
                    tendon_type,
                    tendon_limited,
                    tendon_actfrclimited,
                    tendon_num
                );
                numbers!(tendon_range: 2, tendon_solref_lim: mjNREF as usize, tendon_solimp_lim: mjNIMP as usize,
                    tendon_solref_fri: mjNREF as usize, tendon_solimp_fri: mjNIMP as usize,
                    tendon_lengthspring: 2, tendon_margin: 1, tendon_stiffness: 1, tendon_stiffnesspoly: mjNPOLY as usize,
                    tendon_damping: 1, tendon_dampingpoly: mjNPOLY as usize, tendon_armature: 1, tendon_frictionloss: 1);
                let start_a = *a.tendon_adr.add(i) as usize;
                let start_b = *b.tendon_adr.add(j) as usize;
                for offset in 0..*a.tendon_num.add(i) as usize {
                    let ia = start_a + offset;
                    let ib = start_b + offset;
                    let ty = *a.wrap_type.add(ia);
                    if ty != *b.wrap_type.add(ib) {
                        return Ok(false);
                    }
                    let pa = *a.wrap_prm.add(ia);
                    let pb = *b.wrap_prm.add(ib);
                    let object = match ty as u32 {
                        mjtWrap::mjWRAP_SITE => Obj::mjOBJ_SITE,
                        mjtWrap::mjWRAP_JOINT => Obj::mjOBJ_JOINT,
                        mjtWrap::mjWRAP_SPHERE | mjtWrap::mjWRAP_CYLINDER => {
                            if map(Obj::mjOBJ_SITE, pa as i32)? != pb as i32 {
                                return Ok(false);
                            }
                            Obj::mjOBJ_GEOM
                        }
                        _ => Obj::mjOBJ_UNKNOWN,
                    };
                    if object != Obj::mjOBJ_GEOM && !close(&[pa], &[pb], 0.0) {
                        return Ok(false);
                    }
                    if map(object, *a.wrap_objid.add(ia))? != *b.wrap_objid.add(ib) {
                        return Ok(false);
                    }
                }
            }
            Obj::mjOBJ_SENSOR => {
                same!(
                    sensor_type,
                    sensor_datatype,
                    sensor_needstage,
                    sensor_objtype,
                    sensor_reftype,
                    sensor_dim
                );
                reference!(sensor_objid, *a.sensor_objtype.add(i) as u32);
                reference!(sensor_refid, *a.sensor_reftype.add(i) as u32);
                numbers!(sensor_cutoff: 1, sensor_noise: 1, sensor_delay: 1, sensor_interval: 2);
            }
            Obj::mjOBJ_NUMERIC => {
                same!(numeric_size);
                let n = *a.numeric_size.add(i) as usize;
                let a = Sim::arr(a.numeric_data.add(*a.numeric_adr.add(i) as usize), n);
                let b = Sim::arr(b.numeric_data.add(*b.numeric_adr.add(j) as usize), n);
                if !close(a, b, 0.0) {
                    return Ok(false);
                }
            }
            _ => {}
        }
    }
    Ok(true)
}
