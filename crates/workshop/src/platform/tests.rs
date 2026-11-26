use super::studio::SlotFrame;
use super::*;

#[test]
fn report_does_not_turn_incomplete_evidence_into_approval() {
    let json = r#"{"version":1,"registry_version":3,"status":"completed","assemblies":{
        "блок":{"verdict":"Rejected","worst_run_key":"блок/удар","runs":{"блок/удар":{"findings":[{"reason":"нарушен предел"}]} }},
        "стержень":{"verdict":"NotProven","worst_run_key":null},
        "каретка":{"verdict":"PassedSimulation","readiness":"NeedsWork",
          "coverage":{"required":1,"executed":1,"omitted":0,"open":0},
          "runs":{"каретка/ход":{"status":"completed","outcome":"pass","required":1,"executed":1}}}}}"#;
    let report = Report::parse(json.into()).unwrap();
    assert_eq!(report.verdict(Some("блок")), Verdict::Rejected);
    assert_eq!(report.worst(Some("блок")), Some("блок/удар"));
    assert_eq!(report.reasons(Some("блок")), ["нарушен предел"]);
    assert_eq!(report.verdict(Some("стержень")), Verdict::NotProven);
    assert_eq!(report.verdict(Some("каретка")), Verdict::ProvenInSimulation);
    assert_eq!(report.verdict(None), Verdict::NotAnalyzed);
    assert!(Report::parse(json.replace("Rejected", "Ready")).is_err());
    for verdict in ["Rejected", "NotProven"] {
        assert!(
            Report::parse(json.replace(
                &format!("\"verdict\":\"{verdict}\""),
                &format!("\"verdict\":\"{verdict}\",\"readiness\":\"Passed\""),
            ))
            .is_err()
        );
    }
    assert!(Report::parse(json.replacen("completed", "failed", 1)).is_err());
    for field in ["open", "omitted"] {
        assert!(Report::parse(json.replace(&format!("\"{field}\":0"), &format!("\"{field}\":1"))).is_err());
    }
    assert!(Report::parse(json.replacen("\"executed\":1", "\"executed\":0", 1)).is_err());
    assert!(Report::parse(json.replace("\"executed\":1}}", "\"executed\":0}}")).is_err());
    assert!(Report::parse(json.replace("\"outcome\":\"pass\"", "\"outcome\":\"fail\"")).is_err());
    let task = |status, outcome| {
        Report::parse(format!(r#"{{"status":"{status}","outcome":"{outcome}"}}"#))
    };
    assert_eq!(
        task("failed", "fail").unwrap().verdict(None),
        Verdict::Rejected
    );
    assert_eq!(
        task("incomplete", "not_analyzed").unwrap().verdict(None),
        Verdict::NotAnalyzed
    );
    assert_eq!(
        task("completed", "pass").unwrap().verdict(None),
        Verdict::NotAnalyzed
    );
    assert!(task("completed", "PASS").is_err());
    for invalid in [
        "[NaN]",
        "[1e309]",
        "[1,]",
        "{\"a\":1,\"a\":2}",
        "\"\\uD800\"",
    ] {
        assert!(json::parse(invalid).is_err(), "{invalid}");
    }
}

fn root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
}

#[test]
fn campaign_keeps_pipeline_verdicts_and_repeats_reported_row() {
    let manifest = root().join("devices/stage8/decision-campaign.toml");
    let execute = |repeat| {
        run(&Input::Campaign {
            manifest: manifest.clone(),
            repeat,
            measurements: None,
        })
        .unwrap()
    };
    let output = execute(None);
    let raw = json::parse(&output.report.raw).unwrap();
    let assemblies = raw.get("assemblies").unwrap();
    for (id, expected, verdict) in [
        ("exceeded", "Rejected", Verdict::Rejected),
        ("estimated", "NotProven", Verdict::NotProven),
        ("bounded", "NotProven", Verdict::NotProven),
    ] {
        assert_eq!(
            report::text(assemblies.get(id).unwrap(), "verdict"),
            Some(expected)
        );
        assert_eq!(output.report.verdict(Some(id)), verdict);
    }
    assert_eq!(output.report.readiness("bounded"), Some("NeedsWork"));
    let bounded = assemblies.get("bounded").unwrap().get("runs").unwrap().object().unwrap();
    assert_eq!(bounded.len(), 2); // Две посадки проставки из acceptance/bounded.toml.
    for row in bounded.values() {
        assert!(row.get("findings").unwrap().array().unwrap().iter().any(|finding| {
            report::text(finding.get("key").unwrap(), "class") == Some("static_strength")
                && report::text(finding, "outcome") == Some("not_analyzed")
                && report::text(finding, "reason").is_some_and(|reason| {
                    reason.contains("расчётная прочность не закрывает опасность без стендового замера")
                })
        }));
    }
    let key = output.report.worst(Some("exceeded")).unwrap();
    assert_eq!(output.report.worst(None), Some(key));
    assert_eq!(output.report.worst_time(Some("exceeded")), Some(4_000_000));
    let repeat = execute(Some(key.into()));
    assert_eq!(repeat.report.repeat_key(), Some(key));
    assert_eq!(repeat.report.worst(None), Some(key));
    assert_eq!(repeat.report.verdict(Some("exceeded")), Verdict::Rejected);
    assert!(output.binary.is_none());
    let log = Log::read_binary(repeat.binary.as_deref().expect("журнал выбранной строки")).unwrap();
    let row = |raw: &json::Value| {
        raw.get("assemblies")
            .unwrap()
            .get("exceeded")
            .unwrap()
            .get("runs")
            .unwrap()
            .get(key)
            .unwrap()
            .get("input")
            .unwrap()
            .u64()
            .unwrap()
    };
    assert_eq!(row(&raw), row(&json::parse(&repeat.report.raw).unwrap()));
    let original = output.report.playback(key).unwrap();
    let repeated = repeat.report.playback(key).unwrap();
    assert_eq!(original.input, repeated.input);
    assert_eq!(original.playback, repeated.playback);
    assert_eq!(original.coverage.executed, original.coverage.required);
    assert_eq!(repeated.coverage.executed, 1);
    let playback = &output.report.playback(key).unwrap().playback;
    let stamp = playback.journal;
    let event = playback.event.as_ref().map(|event| event.at_ns);
    assert_eq!(log.fingerprint(), stamp);
    let id = log.header().truth.iter().position(|s| s.name == "qpos[0]").unwrap();
    // Предел оснастки из over-travel-install.toml. Момент сверяется по исходному ряду.
    let first = log.entries().iter().find(|e| {
        e.kind == LogKind::Truth && e.id as usize == id && e.valid && e.value > 8e-6
    }).unwrap();
    assert_eq!(event, Some(first.at.0));
    let (_, replay) = repeat.into_replay().unwrap();
    let replay = replay.unwrap();
    let mut scene = Scene::from_recorded(replay.recorded_scene.as_ref().unwrap()).unwrap();
    assert_eq!(scene.seek(&replay, first.at).unwrap(), first.at);
    assert!((scene.sim.qpos()[0] - first.value).abs() < 1e-14);
    let camera = scene.camera(View::OVERVIEW, [480, 360]).unwrap();
    let rendered = scene.render(&camera, None, &[], 0.0).unwrap();
    assert!(rendered.ids.iter().any(Option::is_some));
    println!(
        "Кампания без замеров · Rejected, NotProven, bounded NotProven/NeedsWork. Повтор {key}, нарушение {} нс, журнал {stamp:016x}",
        first.at.0
    );
}

#[test]
fn replay_is_identical_to_cli_and_has_analytic_pose() {
    let path = root().join("devices/stage8/block-world.toml");
    // 100 шагов по 1 мс дают перемещение по точной сумме накопленных скоростей.
    let output = run(&Input::World {
        world: path.clone(),
        duration: Time(100_000_000),
        catalog: None,
    })
    .unwrap();
    let bytes = output.binary.unwrap();
    let temp = Temp::new().unwrap();
    let cli_log = temp.0.join("cli.bin");
    let args = vec![
        path.clone().into_os_string(),
        "--duration".into(),
        "0.1".into(),
        "--log".into(),
        cli_log.clone().into_os_string(),
        "--json".into(),
    ];
    let mut stdout = Vec::new();
    let mut stderr = Vec::new();
    let status = sim_cli::run(args, Cursor::new([]), &mut stdout, &mut stderr);
    assert_eq!(status, 0, "{}", String::from_utf8_lossy(&stdout));
    assert_eq!(fs::read(cli_log).unwrap(), bytes);
    assert_eq!(stdout, output.report.raw.as_bytes());
    println!(
        "Побайтовая сверка блока · {} байт SIMLOG и {} байт JSON",
        bytes.len(),
        stdout.len()
    );

    let mut scene = Scene::open(&path, None, &BTreeMap::new()).unwrap();
    let mut replay = Replay::read(&bytes).unwrap();
    replay.frames = output.frames;
    assert!(!replay.frames["кадр"].is_empty());
    assert!(replay.image("кадр", Time::ZERO).is_none());
    let frame = replay.image("кадр", replay.end).expect("доставленный кадр");
    // Из каталога: сетка 96×72 и задержка 8 мс после экспозиции 16 мс.
    assert_eq!([frame.width, frame.height], [96, 72]);
    assert_eq!(frame.at.0 - frame.captured.0, 24_000_000);
    assert!(frame.data.chunks_exact(3).any(|p| p != &frame.data[..3]));
    let hash = replay
        .series
        .iter()
        .find(|s| s.kind == SignalKind::Observation && s.name == "кадр/image")
        .unwrap();
    assert_eq!(hash.at(frame.at).unwrap().value, frame.fingerprint());
    assert!(scene.same_world(&replay).unwrap());
    assert_eq!(replay.end, Time(100_000_000));
    let pose = |s: &Scene| {
        s.geometry_pose(s.geoms().iter().find(|g| g.rgba[3] > 0.0).unwrap().id)
            .unwrap()
            .0
    };
    scene.seek(&replay, Time::ZERO).unwrap();
    let initial = pose(&scene)[2];
    scene.seek(&replay, replay.end).unwrap();
    let analytic = initial - 10.0 * (0.001_f64).powi(2) * (100.0 * 101.0) / 2.0;
    assert!((pose(&scene)[2] - analytic).abs() < 1e-13); // Погрешность округления f64 за 100 операций.
    assert!(scene.seek(&replay, Time(replay.end.0 + 1)).is_err());
    assert!(Replay::read(&bytes[..bytes.len() - 1]).is_err());
    let camera = scene.camera(View::OVERVIEW, [320, 240]).unwrap();
    let w = Wrench {
        pair: ["стержень".into(), "блок".into()],
        point_m: pose(&scene),
        force_n: [1.0, 2.0, 3.0],
    };
    let scale = 1.0_f64.hypot(2.0).hypot(3.0);
    let force = Force {
        name: "нагрузка".into(),
        value_n: -2.0,
        scale_n: 8.0,
    };
    let frame = scene
        .render(&camera, Some(&force), std::slice::from_ref(&w), scale)
        .unwrap();
    assert_eq!(frame.force_pixels, 80);
    assert_eq!(
        frame
            .rgb
            .iter()
            .filter(|&&p| p == render::FORCE_COLOR)
            .count(),
        240
    );
    let endpoint = Point::from(w.point_m) + Vec3::from(w.force_n) * (scene.radius / 4.0 / scale);
    assert_eq!(frame.projected_reactions.len(), 1);
    for (actual, expected) in frame.projected_reactions[0].iter().zip([
        independent_projection(&camera, Point::from(w.point_m)),
        independent_projection(&camera, endpoint),
    ]) {
        for (a, b) in actual.iter().zip(expected) {
            assert!((a - b).abs() < 1e-10);
        }
    }
    assert!(frame.rgb.contains(&render::WRENCH_COLOR));
    let bad = Force {
        value_n: f64::NAN,
        ..force
    };
    assert!(scene.render(&camera, Some(&bad), &[], 0.0).is_err());
    assert!(!frame.ids.iter().all(Option::is_none));
}

fn independent_projection(camera: &Camera, p: Point) -> [f64; 2] {
    let d = p - camera.eye;
    let dot = |axis| (0..3).map(|i| camera.axes[(i, axis)] * d[i]).sum::<f64>();
    [
        camera.lens.fx * dot(0) / dot(2) + camera.lens.cx,
        camera.lens.fy * dot(1) / dot(2) + camera.lens.cy,
    ]
}

#[test]
fn звенья_без_поверхностей_видны_по_кинематике() {
    // Два точечных тела на расстоянии 1 м, жёстко связанные. Ось вращения Z.
    let mjcf = r#"<mujoco><worldbody>
      <body name="стержень">
        <joint type="hinge" axis="0 0 1"/>
        <inertial pos="0.5 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
        <body name="блок" pos="1 0 0">
          <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
        </body>
      </body>
    </worldbody></mujoco>"#;
    let mut scene = Scene::build("стержень".into(), 0, Vec::new(), mjcf).unwrap();
    assert!(scene.uses_kinematics());
    assert_eq!(scene.sim.ngeom(), 0);
    scene.sim.qpos_mut()[0] = std::f64::consts::FRAC_PI_2;
    scene.sim.forward();
    let camera = scene.camera(View::OVERVIEW, [480, 360]).unwrap();
    let frame = scene.render(&camera, None, &[], 0.0).unwrap();
    assert!(frame.ids.iter().all(Option::is_none));
    assert!(frame.rgb.contains(&render::KINEMATICS_COLOR));
    // Поворот на 90° переносит родительский отрезок с +X на +Y.
    let [a, b] = [Point::origin(), Point::new(0.0, 1.0, 0.0)]
        .map(|point| independent_projection(&camera, point));
    assert!(frame.projected_kinematics.iter().any(|[from, to]| {
        [from[0] - a[0], from[1] - a[1], to[0] - b[0], to[1] - b[1]]
            .iter().all(|v| v.abs() < 1e-10)
    }));
}

#[test]
fn audit_reacts_to_measured_values_and_ignores_text_that_looks_like_records() {
    let valid = "ProcessTick { evidence: \"ProcessMeasurement { привет\", measurements: [ProcessMeasurement { at: Time(3), signal: SignalSpec { name: \"опора\", unit: \"N\" }, result: Analyzed { value: 2.5, scope: \"модель\" } }] }";
    let audit = audit::Audit::read(&[(Time(3), valid.into())]).unwrap();
    assert_eq!(audit.measurements[&("опора".into(), "N".into())][0].value, 2.5);
    let voltage = valid.replace("unit: \"N\"", "unit: \"V\"");
    let audit = audit::Audit::read(&[(Time(3), voltage)]).unwrap();
    assert_eq!(audit.measurements[&("опора".into(), "V".into())][0].value, 2.5);
    assert!(audit::Audit::read(&[(Time(2), valid.into())]).is_err());
    assert!(audit::Audit::read(&[(Time(3), valid.replace("2.5", "NaN"))]).is_err());
    let contact = r#"ProcessTick { at: Time(3), completed: Some(PhysicsFrame { interval: Interval { start: Time(1), end: Time(3) }, engine_step: ContactStep { contacts: [ContactWrench { geometries: ["блок", "стержень"], point_m: [0, 1, 0], frame: [], force_n: [0, 0, 2.5], torque_nm: [0, 0, 0] }] } }) }"#;
    let audit = audit::Audit::read(&[(Time(3), contact.into())]).unwrap();
    assert_eq!(audit.wrenches[&Time(1)][0].force_n, [0.0, 0.0, 2.5]);
    assert!(!audit.wrenches.contains_key(&Time(3)));
    assert!(audit::Audit::read(&[(Time(4), contact.into())]).is_err());
}

#[test]
fn control_worlds_render_from_task_journals() {
    let suite = input::Suite::load(&root().join("devices/stage8/suite.json")).unwrap();
    let fit = json::parse(&read_text(&root().join("devices/stage8/fit.json")).unwrap()).unwrap();
    let fits = fit.get("entries").unwrap().object().unwrap();
    // Количество записей соответствует §5. Новая строка не требует изменений алгоритма.
    assert_eq!(suite.entries.len(), 7);
    assert_eq!(
        fits.len(),
        suite
            .entries
            .iter()
            .filter(|e| e.participant.is_some())
            .count()
    );
    let storage = Temp::new().unwrap();
    let mut errors = Vec::new();
    for entry in &suite.entries {
        match check_entry(entry, fits.get(&entry.id), &storage.0) {
            Ok(summary) => println!("{} · {summary}", entry.id),
            Err(error) => {
                println!("{} · {error}", entry.id);
                errors.push(format!("{} · {error}", entry.id));
            }
        }
    }
    let bytes = fs::read_dir(&storage.0)
        .unwrap()
        .map(|entry| entry.unwrap().metadata().unwrap().len())
        .sum::<u64>();
    println!("Сохранено во временный каталог кадров на {bytes} байт");
    assert!(errors.is_empty(), "{}", errors.join("\n"));
}

fn check_entry(
    entry: &input::Entry,
    fit: Option<&json::Value>,
    output_directory: &Path,
) -> Result<String, String> {
    let out = run_with_peak(&entry.input(), entry.peak_signal())?;
    let binary = out.binary.as_deref().ok_or("конвейер не выгрузил журнал")?;
    let log = Log::read_binary(binary).map_err(|e| e.to_string())?;
    let temp_log = output_directory.join(format!("{}-headless.bin", entry.id));
    let args = [
        entry.world.as_os_str().to_owned(), "--task".into(), entry.task.as_os_str().to_owned(),
        "--case".into(), entry.case.clone().into(), "--log".into(),
        temp_log.as_os_str().to_owned(), "--json".into(),
    ];
    let mut stdout = Vec::new();
    let mut stderr = Vec::new();
    let code = sim_cli::run(args, Cursor::new([]), &mut stdout, &mut stderr);
    if code > 1 || fs::read(&temp_log).map_err(|e| e.to_string())? != binary
        || stdout != out.report.raw.as_bytes()
    {
        return Err(format!("запуски окна и CLI разошлись: код {code}, {}", String::from_utf8_lossy(&stderr)));
    }
    fs::remove_file(&temp_log).map_err(|e| e.to_string())?;
    let (report, replay) = out.into_replay()?;
    let replay = replay.ok_or("конвейер не выгрузил журнал")?;
    let mut scene = Scene::from_recorded(replay.recorded_scene.as_ref().ok_or("нет точной сцены")?)?;
    let exact_world = scene.same_world(&replay)?;
    if !exact_world {
        return Err("пакет не содержит точный мир журнала".into());
    }
    scene.seek(&replay, Time::ZERO)?;
    if replay.end == Time::ZERO || replay.status != sim_runtime::Status::Running {
        return Err("журнал не содержит завершённой траектории".into());
    }
    if let Some(participant) = &entry.participant {
        let prefix = format!("{}/", participant.name);
        if !scene
            .sim
            .joints()
            .iter()
            .filter(|j| j.name.starts_with(&prefix))
            .any(|joint| {
                replay.series[replay.qpos[joint.qpos_adr]]
                    .points
                    .windows(2)
                    .any(|p| p[0].value != p[1].value)
            })
        {
            return Err("траектория участника осталась неподвижной".into());
        }
    }
    let body = scene
        .sim
        .id(mjtObj::mjOBJ_BODY, &entry.inspect_body)
        .ok_or("нет тела для сверки")?;
    let geoms: Vec<_> = scene
        .geoms()
        .into_iter()
        .filter(|g| g.body == body)
        .collect();
    let peak = replay.peak.as_ref().ok_or("нет отметки худшего события")?;
    // Независимый отсчёт пика из исходного SIMLOG, включая старый текстовый аудит.
    let expected = if let Some(name) = entry.peak_signal() {
        let points = if let Some(id) = log.header().truth.iter().position(|s| s.name == name) {
            log.entries().iter().filter(|e| e.kind == LogKind::Truth && e.id as usize == id && e.valid)
                .map(|e| (e.at, e.value.abs())).collect::<Vec<_>>()
        } else {
            audit::Audit::read(log.process_entries())?.measurements.remove(&(name.into(), "N".into()))
                .ok_or("не найден исходный ряд экстремума")?
                .iter().filter(|p| p.valid).map(|p| (p.at, p.value.abs())).collect()
        };
        first_maximum(points)
    } else {
        let audit = audit::Audit::read(log.process_entries())?;
        first_maximum(audit.wrenches.iter().flat_map(|(&at, values)| {
            values.iter().map(move |w| (at, Vec3::from(w.force_n).norm()))
        }).collect())
    };
    if !expected.is_some_and(|(at, value)| at.0 == peak.at_ns
        && (value - peak.value.abs()).abs() <= value.abs() * 8.0 * f64::EPSILON)
    {
        return Err(format!("пик {:?} не совпал с исходным рядом {expected:?}", peak));
    }
    let end = replay.end;
    let peak_time = Time(peak.at_ns);
    let (mut slot, cursor) = super::studio::open_at_peak(replay)?;
    if cursor != peak_time {
        return Err("курсор окна не перешёл в точный момент события".into());
    }
    let moments = [
        ("start", Time::ZERO),
        ("middle", Time(end.0 / 2)),
        ("worst", cursor),
    ];
    let mut pixels = Vec::new();
    let mut arrows = 0;
    let mut bars = 0;
    let mut distinct = std::collections::BTreeSet::new();
    let mut checked_times = Vec::new();
    for (label, at) in moments {
        let Some(SlotFrame::Rendered(frame)) = slot.frame(
            at,
            View::OVERVIEW,
            [480, 360],
            entry.force.as_deref().unwrap_or_default(),
        )?
        else {
            return Err("окно не подготовило обзорную текстуру".into());
        };
        let scene = slot.scene.as_ref().unwrap();
        let replay = slot.replay.as_ref().unwrap();
        let pose_at = replay
            .qpos
            .first()
            .and_then(|&i| replay.series[i].at(at))
            .map_or(at, |sample| sample.at);
        checked_times.push(pose_at.0);
        distinct.insert(sim_signal::fnv1a(frame.rgb.as_flattened().iter().copied()));
        let mut seen = vec![false; scene.sim.nq()];
        let first = log.entries().partition_point(|e| e.at < pose_at);
        for e in log.entries()[first..]
            .iter()
            .take_while(|e| e.at == pose_at)
            .filter(|e| e.kind == LogKind::Truth)
        {
            let spec = &log.header().truth[e.id as usize];
            if let Some(i) = spec
                .name
                .strip_prefix("qpos[")
                .and_then(|x| x.strip_suffix(']'))
                .and_then(|x| x.parse::<usize>().ok())
            {
                if scene.sim.qpos()[i].to_bits() != e.value.to_bits() {
                    return Err("поза движка отличается от исходного журнала".into());
                }
                seen[i] = true;
            }
        }
        if !seen.into_iter().all(|s| s) {
            return Err("непроверенная координата позы".into());
        }
        let camera = scene.camera(View::OVERVIEW, [480, 360])?;
        let overlay = entry.force.as_ref().and_then(|name| {
            replay
                .series
                .iter()
                .find(|s| &s.name == name && s.unit == "N")?
                .force(at)
        });
        let reactions = replay
            .wrenches
            .get(&pose_at)
            .map(Vec::as_slice)
            .unwrap_or_default();
        let object: Vec<_> = frame
            .ids
            .iter()
            .enumerate()
            .filter(|(_, id)| id.is_some_and(|id| geoms.iter().any(|g| g.id == id)))
            .collect();
        if object.is_empty() {
            return Err(format!(
                "сцена открыта, журнал до {} нс, объект не виден на {label}",
                replay.end.0
            ));
        }
        if frame.rgb.iter().all(|p| p == &render::BACKGROUND) {
            return Err("пустой кадр".into());
        }
        // Независимое обратное преобразование выборки пикселей. Для каждого
        // видимого геома центр и внешний радиус берутся из MuJoCo, минуя растеризацию.
        for &(i, id) in &object {
            let id = id.ok_or("нет номера видимой геометрии")?;
            let x = (i % frame.size[0] as usize) as f64 + 0.5;
            let y = (i / frame.size[0] as usize) as f64 + 0.5;
            let z = frame.depth[i];
            let world = camera.eye
                + camera.axes
                    * Vec3::new(
                        (x - camera.lens.cx) * z / camera.lens.fx,
                        (y - camera.lens.cy) * z / camera.lens.fy,
                        z,
                    );
            let (center, _) = scene.geometry_pose(id).ok_or("нет центра цели")?;
            // SAFETY: id взят из буфера геометрий этой модели, rbound содержит ngeom значений.
            let radius = unsafe { *scene.sim.model().geom_rbound.add(id) };
            // Микрометр покрывает двоичное округление координат сетки f32 в масштабе стендов.
            if z <= 0.0 || !z.is_finite() || (world - Point::from(center)).norm() > radius + 1e-6 {
                return Err(format!(
                    "пиксель {i} не согласуется с позой цели на {label}"
                ));
            }
        }
        if let Some(overlay) = overlay {
            let width = frame.size[0] as usize;
            let expected = if overlay.scale_n == 0.0 {
                0
            } else {
                (width as f64 * overlay.value_n.abs() / overlay.scale_n).round() as usize
            };
            if frame.rgb[frame.rgb.len() - width..]
                .iter()
                .filter(|&&p| p == render::FORCE_COLOR)
                .count()
                != expected
            {
                return Err("пиксели шкалы силы расходятся с отсчётом".into());
            }
            bars += 1;
        }
        let expected_wrenches: Vec<_> = reactions
            .iter()
            .filter_map(|w| {
                let f = Vec3::from(w.force_n);
                if f.norm() == 0.0 {
                    return None;
                }
                let start = Point::from(w.point_m);
                let end = start + f * (scene.radius / 4.0 / replay.wrench_scale);
                (camera.project(start).is_some() && camera.project(end).is_some()).then(|| {
                    [
                        independent_projection(&camera, start),
                        independent_projection(&camera, end),
                    ]
                })
            })
            .collect();
        if frame.projected_reactions.len() != expected_wrenches.len() {
            return Err("пропущен вектор силы".into());
        }
        for (actual, expected) in frame.projected_reactions.iter().zip(expected_wrenches) {
            for (a, b) in actual.iter().zip(expected) {
                if (a[0] - b[0]).abs() > 1e-10 || (a[1] - b[1]).abs() > 1e-10 {
                    return Err("вектор силы расходится с журналом".into());
                }
            }
            arrows += 1;
        }
        let file = output_directory.join(format!("{}-{label}.ppm", entry.id));
        frame.save_ppm(&file).map_err(|e| e.to_string())?;
        pixels.push(object.len());
    }
    if distinct.len() < 2 {
        return Err("все три обзорных кадра совпали".into());
    }
    let sensors: Vec<_> = slot.replay.as_ref().unwrap().frames.iter().filter_map(|(name, frames)| {
        let first = frames.iter().find(|f| f.valid)?;
        Some((name.clone(), first.clone(), frames.iter().filter(|f| f.valid).count()))
    }).collect();
    for (id, spec) in log.header().observations.iter().enumerate() {
        if spec.name.ends_with("/image") || spec.name.ends_with("/depth") {
            let expected = log.entries().iter().filter(|e| {
                e.kind == LogKind::Observation && e.id as usize == id && e.valid
            }).count();
            let actual = slot.replay.as_ref().unwrap().frames.values().flatten()
                .filter(|f| f.signal.0 as usize == id && f.valid).count();
            if actual != expected {
                return Err(format!("«{}»: выгружено {actual} действительных кадров из {expected}", spec.name));
            }
        }
    }
    for (name, expected, _) in &sensors {
        let Some(SlotFrame::Recorded(frame, overlay)) =
            slot.frame(expected.at, View::Sensor(name), [480, 360], "")?
        else {
            return Err(format!("не открыт доставленный кадр «{name}»"));
        };
        if frame != expected || overlay.is_some() {
            return Err("окно изменило пиксели доставленного кадра".into());
        }
        let image = super::studio::display_image(frame, None)?;
        if image.size != [frame.width as usize, frame.height as usize] {
            return Err("размер текстуры изменился при загрузке кадра".into());
        }
        if frame.format == sim_signal::PixelFormat::Rgb8
            && !image.pixels.iter().zip(frame.data.chunks_exact(3)).all(|(actual, expected)| {
                actual.to_array()[..3] == *expected
            })
        {
            return Err("цвет текстуры расходится с доставленными пикселями".into());
        }
    }
    let replay = slot.replay.as_ref().unwrap();
    let mut scalar_views = BTreeMap::new();
    for name in replay.sensor_names() {
        for series in replay.sensor_series(name).filter(|s| s.kind == SignalKind::Observation) {
            if scalar_views.insert((&series.name, &series.unit), series).is_some() {
                return Err(format!("канал «{}» повторяется в видах датчиков", series.name));
            }
        }
    }
    let scalar_sources: Vec<_> = log.header().observations.iter().map(|spec| {
        if spec.name.ends_with("/image") && spec.unit == "RGB8"
            || spec.name.ends_with("/depth") && spec.unit == "mm"
        {
            return Ok(None);
        }
        scalar_views.remove(&(&spec.name, &spec.unit)).map(Some)
            .ok_or_else(|| format!("у канала «{}» нет вида с единицей {}", spec.name, spec.unit))
    }).collect::<Result<_, _>>()?;
    if !scalar_views.is_empty() {
        return Err("видами показаны отсутствующие в SIMLOG каналы наблюдения".into());
    }
    let mut positions = vec![0; scalar_sources.len()];
    for entry in log.entries().iter().filter(|e| e.kind == LogKind::Observation) {
        let id = entry.id as usize;
        if let Some(series) = scalar_sources[id] {
            let point = series.points.get(positions[id]).ok_or_else(|| {
                format!("во виде «{}» пропущен отсчёт", series.name)
            })?;
            if point.at != entry.at || point.valid != entry.valid
                || point.value.to_bits() != entry.value.to_bits()
            {
                return Err(format!("вид «{}» расходится с отсчётом SIMLOG", series.name));
            }
            positions[id] += 1;
        }
    }
    let scalar_channels = scalar_sources.iter().flatten().count();
    if scalar_sources.iter().zip(positions).any(|(series, count)| {
        series.is_some_and(|series| series.points.len() != count)
    }) {
        return Err("вид датчика содержит лишние отсчёты".into());
    }
    let fit = match (fit, &entry.participant) {
        (Some(fit), Some(_)) => check_fit(slot.scene.as_mut().unwrap(), replay, &log, fit)?,
        (None, None) => String::new(),
        _ => return Err("отсутствует парная сверка размещения участника".into()),
    };
    Ok(format!(
        "{} нс, худшее {} нс: кадры {:?}, пиксели цели {:?}, векторы {arrows}, полосы {bars}, \
         точный мир {exact_world}, {scalar_channels} скалярных каналов, кадры датчиков {:?}, крепления {}, вердикт {:?}, исход {:?}, журнал {:016x} побитово{fit}",
        replay.end.0,
        cursor.0,
        checked_times,
        pixels,
        sensors.iter().map(|(name, _, count)| (name, count)).collect::<Vec<_>>(),
        replay.attachments.len(),
        report.verdict(None),
        report.outcome(),
        replay.journal
    ))
}

fn first_maximum(mut points: Vec<(Time, f64)>) -> Option<(Time, f64)> {
    points.sort_by(|a, b| b.1.total_cmp(&a.1).then(a.0.cmp(&b.0)));
    points.first().copied()
}

fn check_fit(
    scene: &mut Scene,
    replay: &Replay,
    log: &Log,
    fit: &json::Value,
) -> Result<String, String> {
    let patch = fit
        .get("patch")
        .and_then(json::Value::str)
        .ok_or("нет имени пятна")?;
    let [a, b] = fit
        .get("anchors")
        .and_then(json::Value::array)
        .ok_or("нет точек посадки")?
    else {
        return Err("ожидалось две точки посадки".into());
    };
    let site = |name: &json::Value| {
        scene
            .sim
            .id(mjtObj::mjOBJ_SITE, name.str()?)
            .and_then(|id| isize::try_from(id).ok())
    };
    let (a, b) = (
        site(a).ok_or("нет первой точки посадки")?,
        site(b).ok_or("нет второй точки посадки")?,
    );
    let vector = |v: &json::Value| -> Option<Vec3> {
        let [x, y, z] = v.array()? else {
            return None;
        };
        Some(Vec3::new(x.f64()?, y.f64()?, z.f64()?))
    };
    let normal = fit
        .get("normal")
        .and_then(vector)
        .ok_or("нет нормали пятна")?;
    let tangent = fit
        .get("tangent")
        .and_then(vector)
        .ok_or("нет касательной пятна")?;
    let marker = format!("\"{patch}\": PatchSample {{ ");
    let mut max_error = 0.0_f64;
    let mut distances = Vec::new();
    let mut positions = Vec::new();
    let rows = log.process_entries();
    for target in [Time::ZERO, Time(replay.end.0 / 2), replay.end] {
        let start = rows.partition_point(|(time, _)| *time < target);
        let (end, patch) = rows[start..]
            .iter()
            .find_map(|(at, row)| row.split_once(&marker).map(|(_, p)| (*at, p)))
            .ok_or("отсутствует измерение посадки")?;
        let (raw_start, _) = patch
            .split_once("interval: Interval { start: Time(")
            .and_then(|(_, body)| body.split_once("), end: Time("))
            .ok_or("интервал измерения посадки недоступен")?;
        let start = Time(raw_start.parse::<u64>().map_err(|e| e.to_string())?);
        let at = if patch.contains("owner: Model, reaction:") {
            start
        } else {
            end
        };
        let (_, offset) = patch
            .split_once("offset_m: ")
            .ok_or("нет вычисленного смещения")?;
        let (offset, _) = offset.split_once(']').ok_or("неполный вектор смещения")?;
        let offset = json::parse(&format!("{offset}]"))?;
        let Some([u, v]) = offset.array() else {
            return Err("неверное смещение".into());
        };
        let (Some(u), Some(v)) = (u.f64(), v.f64()) else {
            return Err("неконечное смещение".into());
        };
        if scene.seek(replay, at)? != at {
            return Err("поза не записана на границе измерения".into());
        }
        let data = scene.sim.data();
        // SAFETY: a и b разрешены по именам в актуальной модели. MuJoCo хранит
        // по три координаты и по девять элементов матрицы для каждого site.
        let (first, second, rotation) = unsafe {
            (
                Vec3::from_column_slice(std::slice::from_raw_parts(
                    data.site_xpos.offset(3 * a),
                    3,
                )),
                Vec3::from_column_slice(std::slice::from_raw_parts(
                    data.site_xpos.offset(3 * b),
                    3,
                )),
                Matrix3::from_row_slice(std::slice::from_raw_parts(
                    data.site_xmat.offset(9 * a),
                    9,
                )),
            )
        };
        let delta = second - first;
        let local = rotation.transpose() * delta;
        let error = (local.dot(&tangent) - u)
            .abs()
            .max((local.dot(&normal.cross(&tangent)) - v).abs());
        // 1 мкм покрывает накопленное округление преобразований многозвенной системы.
        if error > 1e-6 {
            return Err(format!(
                "посадка на {} нс расходится с расчётом на {error:.9} м",
                at.0
            ));
        }
        max_error = max_error.max(error);
        distances.push(delta.norm() * 1000.0);
        positions.push((first, second));
    }
    let movement = positions
        .iter()
        .map(|(a, b)| (a - positions[0].0).norm().max((b - positions[0].1).norm()))
        .fold(0.0, f64::max);
    Ok(format!(
        "; посадка {:?} мм, ход {:.6} мм, невязка {max_error:.3e} м",
        distances,
        movement * 1000.0
    ))
}

#[test]
fn audit_cross_registry_and_coverage_are_required_for_approval() {
    let json = r#"{"version":1,"registry_version":3,"status":"completed",
      "historical_open_rows":null,"assemblies":{
        "блок":{"verdict":"PassedSimulation","readiness":"Passed",
          "coverage":{"required":1,"executed":1,"omitted":0,"open":0},
          "depth_coverage":{"status":"completed","open_rows":[]},
          "runs":{"блок/ход":{"status":"completed","outcome":"pass","required":1,"executed":1}}}}}"#;
    let report = Report::parse(json.into()).unwrap();
    assert_eq!(report.verdict(Some("блок")), Verdict::ProvenInSimulation);
    let old = Report::parse(json.replace("registry_version\":3", "registry_version\":2")).unwrap();
    assert_eq!(old.verdict(Some("блок")), Verdict::NotProven);
    assert_eq!(old.readiness("блок"), Some("NeedsWork"));
    assert!(old.reasons(Some("блок")).iter().any(|r| r.contains("реестр v2")));

    let partial = Report::parse(json.replace(
        r#""depth_coverage":{"status":"completed","open_rows":[]}"#,
        r#""depth_coverage":{"status":"incomplete","open_rows":[{"reason":"нет измерения"}]}"#,
    )).unwrap();
    assert_eq!(partial.verdict(Some("блок")), Verdict::NotProven);
    assert!(partial.reasons(Some("блок")).contains(&"нет измерения"));

    let history = json.replace(
        r#""historical_open_rows":null"#,
        r#""historical_open_rows":{"rows":[{"key":{"station":"блок/ход"},"outcome":"not_analyzed","reason":"строка не закрыта"}]}"#,
    );
    let report = Report::parse(history.clone()).unwrap();
    assert_eq!(report.verdict(Some("блок")), Verdict::NotProven);
    assert!(report.reasons(Some("блок")).contains(&"строка не закрыта"));
    let report = Report::parse(history.replace(r#""station":"блок/ход""#, r#""station":"стержень/ход""#)).unwrap();
    assert_eq!(report.verdict(Some("блок")), Verdict::ProvenInSimulation);
}
