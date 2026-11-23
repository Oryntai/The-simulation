//! Независимый работник для проверки транспорта по stdin/stdout.

use std::io;

use sim_control::transport::serve_worker;
use sim_control::{
    ControlContract, ControlError, Controller, InputChannel, InputKind, OutputChannel,
};
use sim_signal::{ChannelId, Command, Observation, Rng, SignalId};

struct Worker {
    input: SignalId,
    output: ChannelId,
}

impl Controller for Worker {
    fn step(&mut self, input: &Observation, _: &mut Rng) -> Result<Command, ControlError> {
        if input.samples.iter().any(|sample| sample.value == -1.0) {
            // Специальное значение стенда вызывает завершение внешнего процесса.
            return Err(ControlError::Execution("сбой стенда".into()));
        }
        Ok(Command {
            targets: input
                .latest(self.input)
                .map(|value| (self.output, 2.0 * value))
                .into_iter()
                .collect(),
        })
    }
}

fn main() -> Result<(), ControlError> {
    let max_age_ns = std::env::args()
        .nth(1)
        .expect("срок выхода")
        .parse()
        .expect("целое число");
    let signal = SignalId(
        std::env::args()
            .nth(2)
            .expect("номер входа")
            .parse()
            .expect("целое число"),
    );
    let channel = ChannelId(
        std::env::args()
            .nth(3)
            .expect("номер выхода")
            .parse()
            .expect("целое число"),
    );
    let contract = ControlContract::new(
        vec![InputChannel {
            id: signal,
            name: "измерение".into(),
            unit: "m".into(),
            kind: InputKind::Scalar {
                max_age_ns: 2_000_000_000,
            },
        }],
        vec![OutputChannel {
            id: channel,
            name: "верх".into(),
            unit: "m".into(),
            max_age_ns,
        }],
    )?;
    // Граница протокола стенда в байтах. Требуется место для соглашения и одного скаляра.
    serve_worker(
        io::stdin().lock(),
        io::stdout().lock(),
        &contract,
        4096,
        Worker {
            input: signal,
            output: channel,
        },
    )
}
