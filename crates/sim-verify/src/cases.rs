//! Эталонные кейсы: модель прогоняется НАШИМ движком, результат сравнивается с аналитикой.
//! Те же функции зовут `cargo test`, отчёт `verify-report` и окно приложения.

use nalgebra::{DVector, UnitQuaternion};
use sim_core::dynamics::{
    kinetic_energy, potential_energy, spatial_momentum_world, static_holding_torques, world_to_body,
};
use sim_core::integrate::{PointForce, external_forces};
use sim_core::spatial::ang;
use sim_core::{
    Body, G_STANDARD, Inertia, Inputs, Integrator, Joint, Model, Pose, Shape, State, Vec3, Visual,
    Xform, step,
};
use sim_fem::{Cantilever, Loads, solve_cantilever};

use crate::analytic as an;
use crate::check::{Check, Tolerance};
use crate::measure::CrossingDetector;

const BOB_COLOR: [f32; 4] = [0.85, 0.35, 0.2, 1.0];
const ROD_COLOR: [f32; 4] = [0.75, 0.75, 0.78, 1.0];

/// Математический маятник: точечная масса `m` на невесомом стержне длины `l`,
/// ось вращения — Y, подвес в `pivot`. Угол 0 — висит вниз.
pub fn simple_pendulum(l: f64, m: f64, pivot: Vec3) -> Model {
    let mut model = Model::earth();
    model.add_body(
        Body::new(
            "маятник",
            None,
            Joint::revolute(Vec3::new(0.0, -1.0, 0.0)),
            Xform::translation(pivot),
            Inertia::point(m, Vec3::new(0.0, 0.0, -l)),
        )
        .with_visual(Visual::new(
            Shape::Cylinder {
                radius: 0.006,
                length: l,
            },
            Pose::at(Vec3::new(0.0, 0.0, -l / 2.0)),
            ROD_COLOR,
        ))
        .with_visual(Visual::new(
            Shape::Sphere { radius: 0.05 },
            Pose::at(Vec3::new(0.0, 0.0, -l)),
            BOB_COLOR,
        )),
    );
    model
}

/// Прогнать маятник (одно тело на вращательном шарнире) и измерить период.
pub fn run_pendulum_period(
    model: &Model,
    theta0: f64,
    dt: f64,
    periods: usize,
    method: Integrator,
) -> f64 {
    let mut s = State::rest(model);
    s.q[0] = theta0;
    let inputs = Inputs::zero(model);
    let mut det = CrossingDetector::new();
    let mut steps = 0usize;
    while det.crossings().len() < periods + 1 && steps < 10_000_000 {
        step(model, &mut s, &inputs, dt, method);
        det.push(s.t, s.q[0], s.v[0]); // пересечение θ = 0 снизу вверх, производная — θ̇
        steps += 1;
    }
    det.mean_period().unwrap_or(f64::NAN)
}

pub fn pendulum_small_angle() -> Check {
    let (l, theta0) = (1.0, 1.0_f64.to_radians());
    let model = simple_pendulum(l, 1.0, Vec3::zeros());
    let t = run_pendulum_period(&model, theta0, 1e-3, 10, Integrator::Rk4);
    Check {
        id: "pendulum.small",
        title: "Маятник, малая амплитуда 1°: период".into(),
        simulated: t,
        reference: an::pendulum_period_small(l, G_STANDARD),
        unit: "с",
        method: "ABA + RK4, dt = 1 мс, среднее по 10 периодам".into(),
        reference_method: "T = 2π√(L/g)".into(),
        tolerance: Tolerance::Relative(1e-4),
        note: "формула верна только для малых углов: при 1° истинный период длиннее на θ₀²/16 ≈ 1.9e-5".into(),
    }
}

pub fn pendulum_large_amplitude(theta0_deg: f64) -> Check {
    let l = 1.0;
    let theta0 = theta0_deg.to_radians();
    let model = simple_pendulum(l, 1.0, Vec3::zeros());
    let t = run_pendulum_period(&model, theta0, 1e-3, 5, Integrator::Rk4);
    Check {
        id: if theta0_deg < 120.0 {
            "pendulum.large90"
        } else {
            "pendulum.large170"
        },
        title: format!("Маятник, амплитуда {theta0_deg:.0}°: период (нелинейный режим)"),
        simulated: t,
        reference: an::pendulum_period_exact(l, G_STANDARD, theta0),
        unit: "с",
        method: "ABA + RK4, dt = 1 мс, среднее по 5 периодам".into(),
        reference_method: "T = 4√(L/g)·K(sin(θ₀/2)), K — через AGM".into(),
        tolerance: Tolerance::Relative(1e-6),
        note: format!(
            "формула малых углов здесь ошиблась бы на {:.1}%",
            100.0
                * (an::pendulum_period_exact(l, G_STANDARD, theta0)
                    / an::pendulum_period_small(l, G_STANDARD)
                    - 1.0)
        ),
    }
}

pub fn physical_pendulum_rod() -> Check {
    let (l, m) = (1.0, 2.0);
    let mut model = Model::earth();
    // Тонкий однородный стержень, подвес на конце: I_цм = mL²/12.
    model.add_body(Body::new(
        "стержень",
        None,
        Joint::revolute(Vec3::new(0.0, -1.0, 0.0)),
        Xform::IDENTITY,
        Inertia::segment_z(m, Vec3::new(0.0, 0.0, -l / 2.0), l / 12.0_f64.sqrt(), 0.0),
    ));
    let t = run_pendulum_period(&model, 1.0_f64.to_radians(), 1e-3, 10, Integrator::Rk4);
    Check {
        id: "pendulum.physical",
        title: "Физический маятник (стержень), 1°: период".into(),
        simulated: t,
        reference: an::physical_pendulum_period_small(m * l * l / 3.0, m, l / 2.0, G_STANDARD),
        unit: "с",
        method: "ABA + RK4, dt = 1 мс, инерция стержня через пространственную инерцию".into(),
        reference_method: "T = 2π√(I_оси/(m·g·d)), I_оси = mL²/3 (Штейнер)".into(),
        tolerance: Tolerance::Relative(1e-4),
        note: "проверяет перенос инерции (теорема Штейнера) внутри движка".into(),
    }
}

pub fn pendulum_energy_rk4() -> Check {
    let l = 1.0;
    let model = simple_pendulum(l, 1.0, Vec3::zeros());
    let mut s = State::rest(&model);
    s.q[0] = 60.0_f64.to_radians();
    let inputs = Inputs::zero(&model);
    let e0 = kinetic_energy(&model, &s.q, &s.v) + potential_energy(&model, &s.q);
    let scale = 1.0 * G_STANDARD * l;
    let mut worst: f64 = 0.0;
    for _ in 0..60_000 {
        step(&model, &mut s, &inputs, 1e-3, Integrator::Rk4);
        let e = kinetic_energy(&model, &s.q, &s.v) + potential_energy(&model, &s.q);
        worst = worst.max((e - e0).abs() / scale);
    }
    Check {
        id: "pendulum.energy",
        title: "Маятник 60°, 60 с: дрейф полной энергии".into(),
        simulated: worst,
        reference: 0.0,
        unit: "доля m·g·L",
        method: "ABA + RK4, dt = 1 мс, максимум |E(t) − E₀| за 60 с".into(),
        reference_method: "закон сохранения энергии (трения нет)".into(),
        tolerance: Tolerance::Absolute(1e-6),
        note: "явный Эйлер на этом кейсе «накачивает» энергию — см. сценарий маятника в окне"
            .into(),
    }
}

/// Свободное тело (шар) на высоте `z0`.
pub fn free_ball(radius: f64, mass: f64) -> Model {
    let mut model = Model::earth();
    model.add_body(
        Body::new(
            "шар",
            None,
            Joint::Free,
            Xform::IDENTITY,
            Inertia::solid_sphere(mass, radius, Vec3::zeros()),
        )
        .with_visual(Visual::new(
            Shape::Sphere { radius },
            Pose::IDENTITY,
            [0.2, 0.55, 0.9, 1.0],
        )),
    );
    model
}

fn drop_ball(z0: f64, t_end: f64, dt: f64, method: Integrator) -> (f64, f64) {
    let model = free_ball(0.1, 1.0);
    let mut s = State::rest(&model);
    s.q[2] = z0;
    let inputs = Inputs::zero(&model);
    let n = (t_end / dt).round() as usize;
    for _ in 0..n {
        step(&model, &mut s, &inputs, dt, method);
    }
    (s.q[2], s.v[5])
}

pub fn free_fall_position() -> Check {
    let (z0, t) = (10.0, 1.0);
    let (z, _) = drop_ball(z0, t, 1e-3, Integrator::Rk4);
    Check {
        id: "fall.position",
        title: "Свободное падение с 10 м, t = 1 с: высота".into(),
        simulated: z,
        reference: an::free_fall_z(z0, 0.0, G_STANDARD, t),
        unit: "м",
        method: "свободное тело (6 степеней свободы, кватернион) + RK4, dt = 1 мс".into(),
        reference_method: "z = z₀ − g·t²/2".into(),
        tolerance: Tolerance::Absolute(1e-9),
        note: "без сопротивления воздуха; RK4 точен для квадратичного движения — остаётся только округление".into(),
    }
}

pub fn free_fall_velocity() -> Check {
    let (z0, t) = (10.0, 1.0);
    let (_, vz) = drop_ball(z0, t, 1e-3, Integrator::Rk4);
    Check {
        id: "fall.velocity",
        title: "Свободное падение, t = 1 с: скорость".into(),
        simulated: vz,
        reference: an::free_fall_v(0.0, G_STANDARD, t),
        unit: "м/с",
        method: "свободное тело + RK4, dt = 1 мс".into(),
        reference_method: "v = −g·t".into(),
        tolerance: Tolerance::Absolute(1e-9),
        note: String::new(),
    }
}

/// Порядок сходимости полунеявного Эйлера: ошибка должна падать ровно вдвое при dt/2,
/// и совпадать с теоретической g·t·dt/2.
pub fn free_fall_euler_error() -> Check {
    let (z0, t, dt) = (10.0, 1.0, 1e-2);
    let (z, _) = drop_ball(z0, t, dt, Integrator::SemiImplicitEuler);
    let exact = an::free_fall_z(z0, 0.0, G_STANDARD, t);
    // Схема z_{n+1} = z_n + dt·v_{n+1} опережает: тело падает ниже на g·t·dt/2.
    Check {
        id: "fall.euler_error",
        title: "Полунеявный Эйлер, dt = 10 мс: ошибка схемы совпадает с теорией".into(),
        simulated: exact - z,
        reference: an::semi_implicit_euler_fall_error(G_STANDARD, t, dt),
        unit: "м",
        method: "свободное тело + полунеявный Эйлер, ошибка = эталон − симуляция".into(),
        reference_method: "Δz = g·t·dt/2 (вывод из суммы ряда схемы)".into(),
        tolerance: Tolerance::Relative(1e-6),
        note: "проверяем, что знаем ошибку движка заранее, а не только что она мала".into(),
    }
}

pub fn free_fall_convergence_order() -> Check {
    let (z0, t) = (10.0, 1.0);
    let exact = an::free_fall_z(z0, 0.0, G_STANDARD, t);
    let e1 = (drop_ball(z0, t, 1e-2, Integrator::SemiImplicitEuler).0 - exact).abs();
    let e2 = (drop_ball(z0, t, 5e-3, Integrator::SemiImplicitEuler).0 - exact).abs();
    Check {
        id: "fall.order",
        title: "Полунеявный Эйлер: порядок сходимости (ошибка(dt) / ошибка(dt/2))".into(),
        simulated: e1 / e2,
        reference: 2.0,
        unit: "раз",
        method: "два прогона, dt = 10 мс и 5 мс".into(),
        reference_method: "1-й порядок ⇒ отношение ошибок = 2".into(),
        tolerance: Tolerance::Relative(1e-6),
        note: String::new(),
    }
}

/// Вращение свободного тела без моментов (неустойчивость средней оси, «эффект Джанибекова»).
fn tumbling_box() -> (Model, State) {
    let mut model = Model::new(Vec3::zeros());
    model.add_body(Body::new(
        "брус",
        None,
        Joint::Free,
        Xform::IDENTITY,
        Inertia::solid_box(2.0, Vec3::new(0.05, 0.1, 0.15), Vec3::zeros()),
    ));
    let mut s = State::rest(&model);
    let quat = UnitQuaternion::from_euler_angles(0.2, 0.1, -0.3);
    s.q[3] = quat.w;
    s.q[4] = quat.i;
    s.q[5] = quat.j;
    s.q[6] = quat.k;
    s.v[0] = 0.02;
    s.v[1] = 4.0; // вокруг средней оси — неустойчиво
    s.v[2] = 0.02;
    (model, s)
}

pub fn torque_free_momentum() -> Check {
    let (model, mut s) = tumbling_box();
    let inputs = Inputs::zero(&model);
    let l0 = ang(&spatial_momentum_world(&model, &s.q, &s.v));
    let mut worst: f64 = 0.0;
    for _ in 0..10_000 {
        step(&model, &mut s, &inputs, 1e-3, Integrator::Rk4);
        let l = ang(&spatial_momentum_world(&model, &s.q, &s.v));
        worst = worst.max((l - l0).norm() / l0.norm());
    }
    Check {
        id: "rigid.momentum",
        title: "Кувыркающийся брус, 10 с: сохранение момента импульса".into(),
        simulated: worst,
        reference: 0.0,
        unit: "доля |L₀|",
        method: "свободное тело + RK4, dt = 1 мс, вращение вокруг неустойчивой средней оси".into(),
        reference_method: "закон сохранения момента импульса (моментов нет)".into(),
        tolerance: Tolerance::Absolute(1e-6),
        note: "проверяет уравнения Эйлера для вращения внутри ABA и интегрирование кватерниона"
            .into(),
    }
}

pub fn torque_free_energy() -> Check {
    let (model, mut s) = tumbling_box();
    let inputs = Inputs::zero(&model);
    let e0 = kinetic_energy(&model, &s.q, &s.v);
    let mut worst: f64 = 0.0;
    for _ in 0..10_000 {
        step(&model, &mut s, &inputs, 1e-3, Integrator::Rk4);
        worst = worst.max((kinetic_energy(&model, &s.q, &s.v) - e0).abs() / e0);
    }
    Check {
        id: "rigid.energy",
        title: "Кувыркающийся брус, 10 с: сохранение энергии".into(),
        simulated: worst,
        reference: 0.0,
        unit: "доля E₀",
        method: "свободное тело + RK4, dt = 1 мс".into(),
        reference_method: "закон сохранения энергии".into(),
        tolerance: Tolerance::Absolute(1e-6),
        note: String::new(),
    }
}

/// Стальной квадрат 10×10 мм, L = 1 м.
pub const BEAM_L: f64 = 1.0;
pub fn beam_ei() -> f64 {
    sim_fem::section::STEEL_S235.e * sim_fem::section::rect_i(0.01, 0.01)
}

pub fn cantilever_fem_tip() -> Check {
    let (p, ei) = (10.0, beam_ei());
    let s = solve_cantilever(
        &Cantilever {
            length: BEAM_L,
            ei,
            elements: 10,
        },
        &Loads {
            tip_force: p,
            distributed: 0.0,
        },
    );
    Check {
        id: "beam.fem_tip",
        title: "Консоль, сила 10 Н на конце: прогиб конца (МКЭ)".into(),
        simulated: s.tip_deflection(),
        reference: an::cantilever_tip_point(p, BEAM_L, ei),
        unit: "м",
        method: "МКЭ, 10 эрмитовых элементов Эйлера–Бернулли".into(),
        reference_method: "δ = P·L³/(3EI)".into(),
        tolerance: Tolerance::Relative(1e-9),
        note: "сталь 10×10 мм, L = 1 м; малые прогибы, сдвиг не учитывается".into(),
    }
}

pub fn cantilever_fem_slope() -> Check {
    let (p, ei) = (10.0, beam_ei());
    let s = solve_cantilever(
        &Cantilever {
            length: BEAM_L,
            ei,
            elements: 10,
        },
        &Loads {
            tip_force: p,
            distributed: 0.0,
        },
    );
    Check {
        id: "beam.fem_slope",
        title: "Консоль, сила на конце: угол поворота конца (МКЭ)".into(),
        simulated: s.tip_slope(),
        reference: an::cantilever_tip_slope_point(p, BEAM_L, ei),
        unit: "рад",
        method: "МКЭ, 10 элементов".into(),
        reference_method: "θ = P·L²/(2EI)".into(),
        tolerance: Tolerance::Relative(1e-9),
        note: String::new(),
    }
}

pub fn cantilever_fem_shape() -> Check {
    let (p, ei) = (10.0, beam_ei());
    let s = solve_cantilever(
        &Cantilever {
            length: BEAM_L,
            ei,
            elements: 10,
        },
        &Loads {
            tip_force: p,
            distributed: 0.0,
        },
    );
    let worst =
        s.x.iter()
            .zip(s.w.iter())
            .map(|(x, w)| (w - an::cantilever_deflection_point(*x, p, BEAM_L, ei)).abs())
            .fold(0.0, f64::max);
    Check {
        id: "beam.fem_shape",
        title: "Консоль: форма изогнутой оси во всех узлах (МКЭ)".into(),
        simulated: worst / an::cantilever_tip_point(p, BEAM_L, ei),
        reference: 0.0,
        unit: "доля δ",
        method: "МКЭ, 10 элементов, max |w_мкэ(x) − w(x)| по узлам".into(),
        reference_method: "w(x) = P·x²·(3L − x)/(6EI)".into(),
        tolerance: Tolerance::Absolute(1e-9),
        note: String::new(),
    }
}

pub fn cantilever_fem_udl() -> Check {
    let (q, ei) = (50.0, beam_ei());
    let s = solve_cantilever(
        &Cantilever {
            length: BEAM_L,
            ei,
            elements: 10,
        },
        &Loads {
            tip_force: 0.0,
            distributed: q,
        },
    );
    Check {
        id: "beam.fem_udl",
        title: "Консоль, равномерная нагрузка 50 Н/м: прогиб конца (МКЭ)".into(),
        simulated: s.tip_deflection(),
        reference: an::cantilever_tip_udl(q, BEAM_L, ei),
        unit: "м",
        method: "МКЭ, 10 элементов, согласованный вектор нагрузки".into(),
        reference_method: "δ = q·L⁴/(8EI)".into(),
        tolerance: Tolerance::Relative(1e-9),
        note: String::new(),
    }
}

/// Та же консоль, но собранная из N жёстких звеньев многотельного движка с пружинами
/// в шарнирах (цепочка Хенки). Равновесие ищем через обратную динамику (RNEA).
pub fn bar_chain_tip_deflection(n: usize, p: f64) -> f64 {
    bar_chain_tip(n, p, BEAM_L, beam_ei())
}

/// Прогиб конца консоли длины `length` с жёсткостью `ei` под силой `p`, посчитанный цепочкой
/// из `n` жёстких звеньев. Геометрическая нелинейность учитывается (большие повороты).
/// `NaN`, если равновесие не нашлось (прогиб слишком большой для простых итераций).
pub fn bar_chain_tip(n: usize, p: f64, length: f64, ei: f64) -> f64 {
    let h = length / n as f64;
    let k = ei / h;
    let mut model = Model::new(Vec3::zeros());
    let seg_mass = 0.01;
    for i in 0..n {
        let len = if i == n - 1 { h / 2.0 } else { h };
        let placement = if i == 0 {
            Xform::translation(Vec3::new(h / 2.0, 0.0, 0.0))
        } else {
            Xform::translation(Vec3::new(h, 0.0, 0.0))
        };
        model.add_body(
            Body::new(
                &format!("звено{i}"),
                if i == 0 { None } else { Some(i - 1) },
                Joint::revolute(Vec3::y()),
                placement,
                Inertia::point(seg_mass, Vec3::new(len / 2.0, 0.0, 0.0)),
            )
            .with_spring(k, 0.0),
        );
    }
    let tip = PointForce {
        body: n - 1,
        point_body: Vec3::new(h / 2.0, 0.0, 0.0),
        force_world: Vec3::new(0.0, 0.0, -p),
    };
    let mut q = DVector::zeros(n);
    let mut converged = false;
    for _ in 0..3000 {
        let f = external_forces(&model, &q, &[tip]);
        let hold = static_holding_torques(&model, &q, &f);
        // Равновесие без приводов: −k·θ = τ_удержания  ⇒  θ = −τ/k (с релаксацией 0.7
        // — при больших поворотах простые итерации иначе раскачиваются).
        let q_target = -hold / k;
        let q_new = &q + (&q_target - &q) * 0.7;
        let diff = (&q_new - &q).norm();
        q = q_new;
        if diff <= 1e-14 * q.norm().max(1e-30) {
            converged = true;
            break;
        }
    }
    if !converged || !q.iter().all(|x| x.is_finite()) {
        return f64::NAN;
    }
    let xw = world_to_body(&model, &q);
    let tip_world = xw[n - 1].point_to_parent(&tip.point_body);
    -tip_world.z
}

pub fn bar_chain_vs_discrete() -> Check {
    let (n, p) = (20, 0.1);
    Check {
        id: "beam.chain_discrete",
        title: "Цепочка из 20 звеньев с пружинами: прогиб против точного решения цепочки".into(),
        simulated: bar_chain_tip_deflection(n, p),
        reference: an::hencky_chain_tip(p, BEAM_L, beam_ei(), n),
        unit: "м",
        method: "многотельный движок: RNEA + итерации равновесия".into(),
        reference_method: "δ = P·L³/(3EI)·(1 − 1/(4N²))".into(),
        tolerance: Tolerance::Relative(1e-5),
        note: "малый прогиб (P = 0.1 Н), геометрическая нелинейность ~ (δ/L)² пренебрежима".into(),
    }
}

pub fn bar_chain_vs_continuum() -> Check {
    let (n, p) = (20, 0.1);
    Check {
        id: "beam.chain_continuum",
        title: "Цепочка из 20 звеньев против балки Эйлера–Бернулли".into(),
        simulated: bar_chain_tip_deflection(n, p),
        reference: an::cantilever_tip_point(p, BEAM_L, beam_ei()),
        unit: "м",
        method: "многотельный движок (другой способ, не МКЭ)".into(),
        reference_method: "δ = P·L³/(3EI)".into(),
        tolerance: Tolerance::Relative(1e-3),
        note: "ожидаемая ошибка дискретизации 1/(4N²) = 6.25e-4 — укладывается в допуск 1e-3"
            .into(),
    }
}

pub fn bar_chain_vs_elastica() -> Check {
    let n = 40;
    let ei = beam_ei();
    // Нагрузка P·L²/EI = 1: прогиб ~30% длины — линейная формула тут уже врёт на 10%.
    let p = ei / (BEAM_L * BEAM_L);
    Check {
        id: "beam.chain_elastica",
        title: "Большой прогиб (30% длины): цепочка из 40 звеньев против эластики Эйлера".into(),
        simulated: bar_chain_tip(n, p, BEAM_L, ei),
        reference: an::elastica_tip_deflection(p, BEAM_L, ei),
        unit: "м",
        method: "многотельный движок: RNEA + итерации равновесия, большие повороты".into(),
        reference_method: "EI·θ'' = −P·cos θ, метод стрельбы (RK4 + бисекция)".into(),
        tolerance: Tolerance::Relative(1e-3),
        note: format!(
            "линейная формула P·L³/(3EI) дала бы {:.4} м — на {:.1}% больше",
            an::cantilever_tip_point(p, BEAM_L, ei),
            100.0
                * (an::cantilever_tip_point(p, BEAM_L, ei)
                    / an::elastica_tip_deflection(p, BEAM_L, ei)
                    - 1.0)
        ),
    }
}

/// Все эталонные проверки (порядок — как в отчёте).
pub fn run_all() -> Vec<Check> {
    vec![
        pendulum_small_angle(),
        pendulum_large_amplitude(90.0),
        pendulum_large_amplitude(170.0),
        physical_pendulum_rod(),
        pendulum_energy_rk4(),
        free_fall_position(),
        free_fall_velocity(),
        free_fall_euler_error(),
        free_fall_convergence_order(),
        torque_free_momentum(),
        torque_free_energy(),
        cantilever_fem_tip(),
        cantilever_fem_slope(),
        cantilever_fem_shape(),
        cantilever_fem_udl(),
        bar_chain_vs_discrete(),
        bar_chain_vs_continuum(),
        bar_chain_vs_elastica(),
    ]
}
