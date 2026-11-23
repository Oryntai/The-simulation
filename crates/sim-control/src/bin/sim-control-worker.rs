//! Нейтральный работник. Конфигурация читается из аргументов, протокол — из stdin/stdout.

use std::error::Error;
use std::io;
use std::process::ExitCode;
use std::str::FromStr;

use sim_control::builtin::{
    Block, BuiltinConfig, BuiltinController, ImageAxis, ImageProjection, MissingOutput, NodeId,
    OutputBinding, PidConfig, RgbTarget,
};
use sim_control::transport::serve_worker;
use sim_control::{ControlContract, InputChannel, InputKind, OutputChannel};
use sim_signal::{ChannelId, PixelFormat, SignalId};

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}

struct Args(std::env::Args);

impl Args {
    fn take(&mut self, name: &str) -> Result<String, String> {
        self.0.next().ok_or_else(|| format!("нет параметра {name}"))
    }

    fn parse<T: FromStr>(&mut self, name: &str) -> Result<T, String> {
        self.take(name)?
            .parse()
            .map_err(|_| format!("неверный параметр {name}"))
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let mut args = Args(std::env::args());
    args.0.next();
    let mode = args.take("режим")?;
    let limit = args.parse("размер сообщения")?;
    let input_id = SignalId(args.parse("номер входа")?);
    let input_name = args.take("имя входа")?;
    let input_unit = args.take("единица входа")?;
    let output = OutputChannel {
        id: ChannelId(args.parse("номер выхода")?),
        name: args.take("имя выхода")?,
        unit: args.take("единица выхода")?,
        max_age_ns: args.parse("срок выхода")?,
    };
    let gain = args.parse("масштаб")?;
    let offset = args.parse("смещение")?;
    let missing = match args.take("реакция на отсутствие")?.as_str() {
        "omit" => MissingOutput::Omit,
        "error" => MissingOutput::Error,
        value => MissingOutput::Value(value.parse()?),
    };
    let (kind, source) = match mode.as_str() {
        "scalar" | "pid" => (
            InputKind::Scalar {
                max_age_ns: args.parse("срок входа")?,
            },
            Block::Scalar(input_id),
        ),
        "rgb" => {
            let kind = InputKind::Image {
                format: PixelFormat::Rgb8,
                max_capture_age_ns: args.parse("срок съёмки")?,
                max_delivery_age_ns: args.parse("срок доставки кадра")?,
            };
            let axis = match args.take("ось изображения")?.as_str() {
                "x" => ImageAxis::X,
                "y" => ImageAxis::Y,
                _ => return Err("неизвестная ось изображения".into()),
            };
            let target = RgbTarget {
                min: [
                    args.parse("R min")?,
                    args.parse("G min")?,
                    args.parse("B min")?,
                ],
                max: [
                    args.parse("R max")?,
                    args.parse("G max")?,
                    args.parse("B max")?,
                ],
                min_pixels: args.parse("минимум пикселей")?,
                axis,
                projection: match args.take("проекция")?.as_str() {
                    "pixel" => ImageProjection::Pixel,
                    "bearing" => ImageProjection::Bearing {
                        center_px: args.parse("центр проекции")?,
                        focal_px: args.parse("фокусное расстояние")?,
                    },
                    _ => return Err("неизвестная проекция".into()),
                },
            };
            (
                kind,
                Block::Rgb {
                    image: input_id,
                    target,
                },
            )
        }
        _ => return Err("неизвестный режим работника".into()),
    };
    let mut blocks = vec![source];
    if mode == "pid" {
        blocks.push(Block::Pid {
            error: NodeId(0),
            enable: None,
            config: PidConfig {
                kp: args.parse("Kp")?,
                ki: args.parse("Ki")?,
                kd: args.parse("Kd")?,
                integral_limits: [
                    args.parse("нижняя граница интеграла")?,
                    args.parse("верхняя граница интеграла")?,
                ],
                output_limits: [
                    args.parse("нижняя граница регулятора")?,
                    args.parse("верхняя граница регулятора")?,
                ],
            },
        });
    }
    let last = NodeId(blocks.len() - 1);
    blocks.push(Block::Affine {
        terms: vec![(last, gain)],
        offset,
    });
    let output_node = NodeId(blocks.len() - 1);
    if args.0.next().is_some() {
        return Err("лишние параметры работника".into());
    }
    let output_id = output.id;
    let contract = ControlContract::new(
        vec![InputChannel {
            id: input_id,
            name: input_name,
            unit: input_unit,
            kind,
        }],
        vec![output],
    )?;
    let config = BuiltinConfig {
        blocks,
        outputs: vec![OutputBinding {
            channel: output_id,
            node: output_node,
            missing,
        }],
    };
    let controller = BuiltinController::new(contract.clone(), config)?;
    serve_worker(
        io::stdin().lock(),
        io::stdout().lock(),
        &contract,
        limit,
        controller,
    )?;
    Ok(())
}
