//! Печатает пары геометрий, которые MuJoCo может столкнуть в мире: им нужны калибровки
//! в разделе `[contacts]` физики режима.
//!
//! cargo run --offline -q -p sim-cli --example contact_candidates -- мир.toml

fn main() -> Result<(), String> {
    let path = std::env::args().nth(1).ok_or("нужен путь к миру")?;
    let world = sim_world::load_and_compile(std::path::Path::new(&path))?;
    let sim = sim_mujoco::Sim::from_xml(&world.mjcf)?;
    for [a, b] in sim.contact_candidates()? {
        println!("{a}\t{b}");
    }
    Ok(())
}
