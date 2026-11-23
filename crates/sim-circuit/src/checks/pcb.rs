//! Расчёты платы против опубликованных значений: калькулятор IPC-2221 третьей стороны,
//! графики IPC-2152, пример из учебника для микрополоска.

use sim_verify::{Check, Tolerance};

use super::mk;
use crate::pcb::{
    IpcLayer, MIL, ipc2221_area_mil2, microstrip_hammerstad_jensen, microstrip_ipc2141,
};

pub fn all() -> Vec<Check> {
    vec![
        ipc2221_vs_calculator(3.0, "pcb.ipc2221_ext_3a", 74.2),
        ipc2221_vs_calculator(10.0, "pcb.ipc2221_ext_10a", 390.3),
        ipc2221_internal_vs_2152_conservative(),
        ipc2221_external_vs_2152_with_plane(),
        microstrip_steer_z0(),
        microstrip_steer_eps_eff(),
        microstrip_steer_50ohm(),
        microstrip_ipc_vs_hj(1.6e-3, 2.9e-3, "pcb.microstrip_ipc_vs_hj_1p6mm"),
        microstrip_ipc_vs_hj(0.2e-3, 0.36e-3, "pcb.microstrip_ipc_vs_hj_0p2mm"),
    ]
}

/// Калькулятор Atlas PCB (<https://atlaspcb.com/tools/trace-width-calculator/>): наружный
/// слой, ΔT = 10 К — 3 А → 74.2 mil², 10 А → 390.3 mil².
fn ipc2221_vs_calculator(current: f64, id: &'static str, published: f64) -> Check {
    mk(
        id,
        &format!("IPC-2221: сечение наружной дорожки на {current:.0} А при ΔT = 10 К"),
        ipc2221_area_mil2(current, 10.0, IpcLayer::External),
        published,
        "mil²",
        "наша формула A = (I/(k·ΔT^0.44))^(1/0.725), k = 0.048",
        "опубликованный калькулятор IPC-2221 (Atlas PCB)",
        Tolerance::Relative(2e-3),
        "проверяет реализацию формулы и константы, а не физику; ширина зависит от принятой \
         толщины 1 oz (калькулятор: 1.378 mil, у нас 34.3 мкм = 1.350 mil по IPC-4562)",
    )
}

/// IPC-2152, «conservative chart» (по сути внутренний график IPC-2221): 10 А, ΔT = 20 К →
/// ≈ 680 mil² (снято с графика). J. Olson, «Determining Circuit Board Current Carrying
/// Capability», <https://blog.mbedded.ninja/pdfs/JackOlson-HowTo2152.pdf>.
fn ipc2221_internal_vs_2152_conservative() -> Check {
    mk(
        "pcb.ipc2221_int_vs_2152",
        "Внутренняя дорожка 10 А, ΔT 20 К: IPC-2221 против графика IPC-2152",
        ipc2221_area_mil2(10.0, 20.0, IpcLayer::Internal),
        680.0,
        "mil²",
        "IPC-2221, k = 0.024",
        "IPC-2152, консервативный график (≈ 680 mil², снято с графика, пример J. Olson)",
        Tolerance::Relative(0.05),
        "консервативный график IPC-2152 — это старые данные NBS 1956 г.; IPC-2152 указывает, \
         что реальные внутренние дорожки греются почти как наружные, т.е. обе оценки сильно \
         завышают нужное сечение",
    )
}

/// Пример J. Olson по IPC-2152: универсальный график (рис. 5-2: 3 oz, полиимид 0.070",
/// без полигона) 10 А / 20 К → ≈ 520 mil²; множители: медь 1 oz ×0.84, медный полигон 1 oz на
/// 20 mil под дорожкой ×0.47, плата 0.062" ×1.08, FR-4 ×1.02 (без запаса 5%).
fn ipc2221_external_vs_2152_with_plane() -> Check {
    let ipc2152 = 520.0 * 0.84 * 0.47 * 1.08 * 1.02;
    let no_plane = 520.0 * 0.84 * 1.08 * 1.02;
    let ours = ipc2221_area_mil2(10.0, 20.0, IpcLayer::External);
    mk(
        "pcb.ipc2221_ext_vs_2152",
        "Наружная дорожка 10 А, ΔT 20 К: IPC-2221 против IPC-2152 (с полигоном)",
        ours,
        ipc2152,
        "mil²",
        &format!(
            "IPC-2221, k = 0.048 (ширина {:.0} mil при 1 oz)",
            ours / (34.3e-6 / MIL)
        ),
        "IPC-2152 по примеру J. Olson: 520·0.84·0.47·1.08·1.02 mil² (значение графика ≈, снято с графика)",
        Tolerance::Relative(0.2),
        &format!(
            "расхождение ожидаемо и не подгоняется: с полигоном под дорожкой 2221 консервативен; \
             без полигона IPC-2152 требует ≈ {no_plane:.0} mil² — почти вдвое больше, т.е. на \
             маленькой плате без полигонов формула 2221 НЕ консервативна"
        ),
    )
}

/// Steer, «Fundamentals of Microwave and RF Design», LibreTexts, §4.4, пример 2:
/// w = 600 мкм, h = 635 мкм, εr = 4.1, t = 0 → εe = 2.967, Z0 = 75.4 Ом
/// (открытый учебник на eng.libretexts.org, раздел «Microstrip Transmission Lines», CC BY-NC).
fn microstrip_steer_z0() -> Check {
    let m = microstrip_hammerstad_jensen(600e-6, 635e-6, 0.0, 4.1);
    mk(
        "pcb.microstrip_hj_steer_z0",
        "Микрополосок: Z0 (Хаммерстад–Йенсен) против примера из учебника",
        m.z0,
        75.4,
        "Ом",
        "формулы Хаммерстада–Йенсена (1980), η₀ = μ₀c",
        "M. Steer, Fundamentals of Microwave and RF Design, §4.4, пример 2",
        Tolerance::Relative(5e-3),
        "в учебнике промежуточные величины округлены (Z0 в воздухе 129.7 Ом) — расхождение ~0.2%",
    )
}

fn microstrip_steer_eps_eff() -> Check {
    let m = microstrip_hammerstad_jensen(600e-6, 635e-6, 0.0, 4.1);
    mk(
        "pcb.microstrip_hj_steer_eeff",
        "Микрополосок: εэфф (Хаммерстад–Йенсен) против примера из учебника",
        m.eps_eff,
        2.967,
        "",
        "формулы Хаммерстада–Йенсена (1980)",
        "M. Steer, §4.4, пример 2: εe = 2.967",
        Tolerance::Relative(1e-3),
        "",
    )
}

fn microstrip_steer_50ohm() -> Check {
    let m = microstrip_hammerstad_jensen(2.056, 1.0, 0.0, 4.0);
    mk(
        "pcb.microstrip_hj_steer_50ohm",
        "Микрополосок εr = 4, w/h = 2.056: Z0 против таблицы учебника (50 Ом)",
        m.z0,
        50.0,
        "Ом",
        "формулы Хаммерстада–Йенсена (1980)",
        "M. Steer, §4.4, таблица синтеза: εr = 4, Z0 = 50 Ом → w/h = 2.056",
        Tolerance::Relative(3e-3),
        "w/h в таблице дано с 4 значащими цифрами",
    )
}

/// IPC-2141 против Хаммерстада–Йенсена для типичной 50-омной линии на FR-4 (εr = 4.3, 1 oz).
fn microstrip_ipc_vs_hj(h: f64, w: f64, id: &'static str) -> Check {
    let t = 35e-6;
    let er = 4.3;
    let ipc = microstrip_ipc2141(w, h, t, er);
    let hj = microstrip_hammerstad_jensen(w, h, t, er);
    mk(
        id,
        &format!(
            "Микрополосок h = {:.2} мм, w = {:.2} мм: IPC-2141 против Хаммерстада–Йенсена",
            h * 1e3,
            w * 1e3
        ),
        ipc.z0,
        hj.z0,
        "Ом",
        &format!(
            "IPC-2141: 87/√(εr+1.41)·ln(5.98h/(0.8w+t)), w/h = {:.2} (в диапазоне 0.1…2: {})",
            w / h,
            if ipc.valid { "да" } else { "нет" }
        ),
        "Хаммерстад–Йенсен с поправкой на толщину (точность ~0.2%)",
        Tolerance::Relative(0.05),
        "упрощённая формула IPC-2141 расходится с точной на единицы процентов и тем сильнее, \
         чем толще медь относительно h (t/h = 0.02 и 0.18): это свойство формулы, не ошибка",
    )
}
