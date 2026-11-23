//! Аккумулятор и плата: сценарии использования API.

use sim_circuit::battery::{Cell, CellParams, CellWarning, Pack};
use sim_circuit::pcb::{self, Board, DesignRules, Part, Trace, TraceLayer};

#[test]
fn pack_discharge_heats_and_warns_at_cutoff() {
    // Батарея 4S2P для привода экзоскелета: 8 А из батареи, 4 А на ячейку.
    let mut pack = Pack::new(CellParams::lg_m50(), 4, 2);
    assert!((pack.ocv() - 4.0 * 4.193).abs() < 0.01);
    let mut warned_low = false;
    for _ in 0..(3 * 3600) {
        let w = pack.step(8.0, 1.0, 25.0);
        if w.iter()
            .any(|w| matches!(w, CellWarning::UnderVoltage { .. }))
        {
            warned_low = true;
            break;
        }
    }
    assert!(
        warned_low,
        "к концу разряда должно прийти предупреждение о напряжении"
    );
    assert!(pack.soc() < 0.05, "SOC в конце {}", pack.soc());
    assert!(pack.heat_w() > 0.0);
    let cell = pack.cell();
    assert!(cell.irreversible_heat_w() > 4.0 * 4.0 * cell.r0() * 0.99);
}

#[test]
fn cold_cell_has_less_capacity_and_more_resistance() {
    let c = Cell::new(CellParams::lg_m50());
    assert!(c.available_capacity_ah(0.0) < c.available_capacity_ah(25.0));
    let p = CellParams::lg_m50();
    let ratio = p.r0_at(0.5, 0.0) / p.r0_at(0.5, 25.0);
    // Ea = 15 кДж/моль → R(0 °C)/R(25 °C) ≈ 1.74 (измерено Brosa Planella 2021: 1.75).
    assert!((ratio - 1.74).abs() < 0.02, "{ratio}");
}

#[test]
fn board_report_for_motor_driver() {
    let mut b = Board::new(0.06, 0.04);
    b.traces.push(Trace {
        points: vec![[0.002, 0.02], [0.058, 0.02]],
        width: 2.5e-3,
        layer: TraceLayer::Top,
        net: "VBAT".into(),
        current: 8.0,
    });
    b.traces.push(Trace {
        points: vec![[0.01, 0.005], [0.01, 0.035]],
        width: 0.5e-3,
        layer: TraceLayer::Inner(1),
        net: "SENSE".into(),
        current: 0.05,
    });
    b.parts.push(Part {
        name: "Q1".into(),
        footprint: [6.5e-3, 10e-3],
        pos: [0.03, 0.01],
        power_w: 1.2,
        theta_ja: pcb::package("TO-252").unwrap().theta_ja,
        tj_max_c: 175.0,
    });
    let rep = pcb::evaluate(&b, 35.0, &DesignRules::default());
    let t = &rep.traces[0];
    // Перегрев по IPC-2221 и сопротивление при рабочей температуре согласованы с формулами.
    assert!(
        (t.delta_t - pcb::ipc2221_delta_t(8.0, t.area_mil2, pcb::IpcLayer::External)).abs() < 1e-12
    );
    assert!(
        (t.resistance - pcb::trace_resistance(0.056, 2.5e-3, pcb::oz_to_m(1.0), 35.0 + t.delta_t))
            .abs()
            < 1e-15
    );
    assert!((rep.parts[0].tj_c - (35.0 + 1.2 * 45.1)).abs() < 1e-9);
    for w in &rep.warnings {
        println!("{w}");
    }
}
