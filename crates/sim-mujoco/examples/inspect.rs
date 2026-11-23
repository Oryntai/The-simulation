//! Посмотреть, что внутри модели тела: `cargo run -p sim-mujoco --example inspect -- full|arm|myoarm [фильтр]`.
use sim_mujoco::{JointKind, Sim, compose_myoarm, models};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let which = args.get(1).map(String::as_str).unwrap_or("full");
    let filter = args.get(2).cloned().unwrap_or_default();
    let s = match which {
        "myoarm" => compose_myoarm(),
        "arm" => Sim::load(&models::ms_human_700_arm()),
        _ => Sim::load(&models::ms_human_700()),
    }
    .expect("модель");
    println!("== суставы ({}):", s.njnt());
    for j in s.joints() {
        if j.name.contains(&filter) {
            let kind = match j.kind {
                JointKind::Free => "free",
                JointKind::Ball => "ball",
                JointKind::Slide => "slide",
                JointKind::Hinge => "hinge",
            };
            println!(
                "  {:>3} {:<28} {kind:<5} q[{}] = {:+.3}  предел {:+.3}…{:+.3}{}",
                j.id,
                j.name,
                j.qpos_adr,
                s.qpos()[j.qpos_adr],
                j.range[0],
                j.range[1],
                if j.limited {
                    ""
                } else {
                    " (без предела)"
                }
            );
        }
    }
    println!("== тела ({}):", s.nbody());
    for (i, (name, m)) in s.bodies().iter().enumerate() {
        if name.contains(&filter) {
            println!(
                "  {i:>3} {name:<24} масса {m:.3} кг, с потомками {:.3} кг",
                s.subtree_mass(i)
            );
        }
    }
    println!("== мышцы ({}):", s.muscles().len());
    for m in s.muscles() {
        if m.name.contains(&filter) {
            println!(
                "  {:<16} F₀ = {:7.1} Н   длина {:.3}…{:.3} м",
                m.name, m.f_max, m.length_range[0], m.length_range[1]
            );
        }
    }
}
