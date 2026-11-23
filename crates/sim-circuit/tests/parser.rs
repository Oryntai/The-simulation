//! Разбор нетлиста: ошибки с номером строки, подсхемы (макромодель ОУ), регистр.

use sim_circuit::{Circuit, SimError, operating_point, parse};

#[test]
fn errors_carry_line_numbers_and_russian_text() {
    let e = parse("заголовок\nR1 a 0 1k\nC1 a 0 -1u\n").unwrap_err();
    assert_eq!(e.line, 3);
    assert!(e.message.contains("положительной"), "{e}");
    let e = parse("t\nV1 a 0 PULSE(0 1 0 1n)\nV2 b 0 SIN(0 1)\n").unwrap_err();
    assert_eq!(e.line, 3);
    let e = parse("t\n.subckt x a b\nR1 a b 1\n").unwrap_err();
    assert!(e.message.contains(".ends"));
    // Ошибка модели обнаруживается при построении схемы, тоже со строкой.
    match Circuit::parse("t\nV1 a 0 1\nD1 a 0 nomodel\n") {
        Err(SimError::Element { line, message, .. }) => {
            assert_eq!(line, 3);
            assert!(message.contains("не найдена"));
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn opamp_macro_inverting_amplifier() {
    // Макромодель ОУ: входное сопротивление, ИНУН с усилением 1e5, выходное сопротивление.
    let src = "Инвертирующий усилитель на макромодели ОУ
.subckt opamp inp inn out
Rin inp inn 1meg
E1 int 0 inp inn 1e5
Rout int out 75
.ends
Vin in 0 DC 0.1
R1 in n 1k
R2 n out 10k
XU1 0 n out opamp
Rl out 0 10k
.op
.END
";
    let op = operating_point(&Circuit::parse(src).unwrap()).unwrap();
    let v = op.voltage("out").unwrap();
    // Идеальный ОУ: −R2/R1·Vin = −1 В; конечное усиление A даёт ошибку ≈ (1+R2/R1)/A.
    assert!((v + 1.0).abs() < 2e-4, "{v}");
    assert!(
        op.voltage("xu1.int").is_ok(),
        "внутренний узел подсхемы доступен по имени"
    );
}

#[test]
fn case_insensitive_names_and_ground_alias() {
    let ckt = Circuit::parse("t\nv1 IN gnd dc 5\nR1 in OUT 1K\nr2 out 0 1k\n.OP\n").unwrap();
    let op = operating_point(&ckt).unwrap();
    assert!((op.voltage("OUT").unwrap() - 2.5).abs() < 1e-8);
    assert!((op.current("V1").unwrap() + 2.5e-3).abs() < 1e-9);
    assert_eq!(op.voltage("gnd").unwrap(), 0.0);
}

#[test]
fn singular_circuit_is_reported() {
    let ckt = Circuit::parse("t\nV1 a 0 1\nV2 a 0 2\n").unwrap();
    assert!(matches!(operating_point(&ckt), Err(SimError::Singular(_))));
    assert!(Circuit::parse("t\nR1 a b 1\n").is_err(), "нет земли");
}
