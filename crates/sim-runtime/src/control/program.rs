use sim_control::ControlContract;
use sim_control::builtin::{
    Block, BuiltinConfig, BuiltinController, ImageAxis, ImageProjection, MissingOutput, NodeId,
    OutputBinding, PidConfig, RgbTarget, TiltConfig,
};

use super::schema::{Table, parse_number, tables};

pub(super) fn builtin(text: &str, contract: ControlContract) -> Result<BuiltinController, String> {
    let mut tables = tables(text)?;
    let mut header = tables.remove(0);
    if header.number::<u32>("version")? != 1 {
        return Err(header.error("неподдержанная версия графа"));
    }
    header.finish()?;
    let mut blocks = Vec::new();
    let mut outputs = Vec::new();
    for mut t in tables {
        match t.kind.as_str() {
            "block" => blocks.push(block(&mut t, &contract)?),
            "bind" => {
                let name = t.text("output")?;
                let channel = contract
                    .outputs()
                    .find(|out| out.name == name)
                    .ok_or_else(|| t.error("выход отсутствует в схеме"))?
                    .id;
                let node = NodeId(t.number("node")?);
                let missing = match t.text("missing")?.as_str() {
                    "omit" => MissingOutput::Omit,
                    "error" => MissingOutput::Error,
                    "value" => MissingOutput::Value(t.finite("value")?),
                    _ => return Err(t.error("неизвестная реакция графа")),
                };
                outputs.push(OutputBinding {
                    channel,
                    node,
                    missing,
                });
            }
            _ => return Err(t.error("неизвестная таблица графа")),
        }
        t.source()?;
        t.finish()?;
    }
    BuiltinController::new(contract, BuiltinConfig { blocks, outputs }).map_err(|e| e.to_string())
}

fn block(t: &mut Table, contract: &ControlContract) -> Result<Block, String> {
    let kind = t.text("kind")?;
    let signal = |t: &mut Table| {
        let name = t.text("signal")?;
        contract
            .inputs()
            .find(|input| input.name == name)
            .map(|input| input.id)
            .ok_or_else(|| t.error("вход отсутствует в схеме"))
    };
    let range =
        |t: &mut Table, low, high| -> Result<_, String> { Ok([t.finite(low)?, t.finite(high)?]) };
    let node = |t: &mut Table, key| t.number(key).map(NodeId);
    Ok(match kind.as_str() {
        "scalar" => Block::Scalar(signal(t)?),
        "constant" => Block::Constant(t.finite("value")?),
        "affine" => {
            let nodes = t.array("nodes", parse_number::<usize>)?;
            let weights = t.array("weights", |raw| {
                parse_number::<f64>(raw).filter(|n| n.is_finite())
            })?;
            if nodes.len() != weights.len() {
                return Err(t.error("разное число входов и весов"));
            }
            Block::Affine {
                terms: nodes.into_iter().map(NodeId).zip(weights).collect(),
                offset: t.finite("offset")?,
            }
        }
        "in_range" => Block::InRange {
            input: node(t, "input")?,
            bounds: range(t, "min", "max")?,
        },
        "gate" => Block::Gate {
            input: node(t, "input")?,
            condition: node(t, "condition")?,
        },
        "pid" => Block::Pid {
            error: node(t, "error")?,
            enable: t.has("enable").then(|| node(t, "enable")).transpose()?,
            config: PidConfig {
                kp: t.finite("kp")?,
                ki: t.finite("ki")?,
                kd: t.finite("kd")?,
                integral_limits: range(t, "integral_min", "integral_max")?,
                output_limits: range(t, "min", "max")?,
            },
        },
        "tilt" => Block::PlanarTilt {
            sin: node(t, "sin")?,
            cos: node(t, "cos")?,
            rate: node(t, "rate")?,
            config: TiltConfig {
                acceleration_range: range(t, "acceleration_min", "acceleration_max")?,
                time_constant_s: t.finite("time_constant_s")?,
            },
        },
        "rgb" => {
            let image = signal(t)?;
            let min = t
                .bytes("min")?
                .try_into()
                .map_err(|_| t.error("ожидаются три компонента цвета"))?;
            let max = t
                .bytes("max")?
                .try_into()
                .map_err(|_| t.error("ожидаются три компонента цвета"))?;
            let min_pixels = t.number("min_pixels")?;
            let axis = match t.text("axis")?.as_str() {
                "x" => ImageAxis::X,
                "y" => ImageAxis::Y,
                _ => return Err(t.error("неизвестная ось изображения")),
            };
            let projection = match t.text("projection")?.as_str() {
                "pixel" => ImageProjection::Pixel,
                "bearing" => ImageProjection::Bearing {
                    center_px: t.finite("center_px")?,
                    focal_px: t.finite("focal_px")?,
                },
                _ => return Err(t.error("неизвестная проекция")),
            };
            Block::Rgb {
                image,
                target: RgbTarget {
                    min,
                    max,
                    min_pixels,
                    axis,
                    projection,
                },
            }
        }
        _ => return Err(t.error("неизвестный блок графа")),
    })
}
