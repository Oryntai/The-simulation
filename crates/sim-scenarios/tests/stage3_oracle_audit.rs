use sim_scenarios::Scenario;
use sim_scenarios::exo_loop::ExoLoop;

#[test]
fn сдвиг_отключения_и_масса_меняют_старый_контур() {
    // Условия исходного журнала: 25 мкс шаг, 3 с опыта. 5 кг — половина
    // исходной нагрузки, перенос отключения на 200 мс — 20 кадров по 10 мс.
    for (name, preset, mass, cutoff) in
        [("mass", 0, Some(5.0), None), ("fault", 2, None, Some(120))]
    {
        let mut scene = ExoLoop::default();
        scene.apply_preset(preset);
        if let Some(mass) = mass {
            scene.set_param("payload", mass);
        }
        scene.start_trace(200);
        for frame in 0..300 {
            if Some(frame) == cutoff {
                scene.run_action("estop");
            }
            scene.advance(0.01);
        }
        let trace = scene.take_trace().unwrap();
        assert_eq!(trace.steps, 120_000);
        assert_eq!(trace.samples.len(), 600);
        let min = trace
            .samples
            .iter()
            .map(|p| p.theta)
            .reduce(f64::min)
            .unwrap();
        if name == "fault" {
            assert!(min < 0.0);
            let off = trace
                .samples
                .iter()
                .find(|p| p.t > 1.0 && !p.driver_on)
                .unwrap();
            assert!((1.2..=1.205).contains(&off.t), "{}", off.t);
        } else {
            assert!(min > 1.5);
            assert!(trace.samples.iter().any(|p| p.current_a.abs() > 1.0));
        }
        for row in trace.samples {
            println!(
                "сверка,{name},{},{},{},{},{}",
                row.t, row.theta, row.omega, row.joint_torque, row.current_a
            );
        }
    }
}
