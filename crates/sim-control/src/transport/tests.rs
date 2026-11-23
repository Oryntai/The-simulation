use std::io::Cursor;

use sim_signal::{ChannelId, ImageFrame, PixelFormat, Sample, SignalId};

use super::*;
use crate::{InputChannel, InputKind, OutputChannel};

const OPTIONS: SessionOptions = SessionOptions {
    first_request: RequestId(7),
    seed: 0x0102_0304_0506_0708,
    max_message_bytes: 128,
};

fn contract() -> ControlContract {
    ControlContract::new(
        vec![InputChannel {
            id: SignalId(4),
            name: "x".into(),
            unit: "m".into(),
            kind: InputKind::Scalar { max_age_ns: 1 },
        }],
        vec![OutputChannel {
            id: ChannelId(5),
            name: "y".into(),
            unit: "m".into(),
            max_age_ns: 3,
        }],
    )
    .unwrap()
}

fn hello(kind: u8) -> Vec<u8> {
    // Независимо собранные байты v1. Здесь литералы — поля проволочной схемы тестового сеанса.
    vec![
        kind, b'S', b'I', b'M', b'C', 1, 0, 0, 0, 128, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 8, 7, 6, 5,
        4, 3, 2, 1, 1, 0, 0, 0, 4, 0, 0, 0, 1, 0, 0, 0, b'x', 1, 0, 0, 0, b'm', 1, 1, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 5, 0, 0, 0, 1, 0, 0, 0, b'y', 1, 0, 0, 0, b'm', 3, 0, 0, 0, 0, 0, 0,
        0,
    ]
}

fn wire(body: &[u8]) -> Vec<u8> {
    let mut bytes = (body.len() as u32).to_le_bytes().to_vec();
    bytes.extend_from_slice(body);
    bytes
}

fn raw_response(id: u64, output: u32, value: f64) -> Vec<u8> {
    let mut bytes = vec![4];
    bytes.extend_from_slice(&id.to_le_bytes());
    bytes.extend_from_slice(&[1, 0, 0, 0]);
    bytes.extend_from_slice(&output.to_le_bytes());
    bytes.extend_from_slice(&value.to_bits().to_le_bytes());
    wire(&bytes)
}

fn peer(remaining: &[u8]) -> StreamTransport<Cursor<Vec<u8>>, Vec<u8>> {
    let mut received = wire(&hello(2));
    received.extend_from_slice(remaining);
    StreamTransport::open(Cursor::new(received), Vec::new(), contract(), OPTIONS).unwrap()
}

fn request(id: u64, time: u64) -> ControlRequest {
    ControlRequest {
        id: RequestId(id),
        observation: Observation {
            time: Time(time),
            ..Observation::default()
        },
    }
}

#[test]
fn stream_handshake_and_scalar_request_match_independent_wire_bytes() {
    let mut session = peer(&raw_response(7, 5, 2.5));
    let mut req = request(7, 12);
    req.observation.samples = vec![
        Sample {
            signal: SignalId(4),
            value: 2.5,
            at: Time(12),
            valid: true,
        },
        Sample {
            signal: SignalId(4),
            value: 1.0,
            at: Time(11),
            valid: true,
        },
    ];
    let response = session.exchange(&req).unwrap();
    assert_eq!(response.id, req.id);
    assert_eq!(response.command.targets, [(ChannelId(5), 2.5)]);
    let mut expected = wire(&hello(1));
    expected.extend(wire(&[
        3, 7, 0, 0, 0, 0, 0, 0, 0, 12, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 4, 64, 12, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    ]));
    assert_eq!(session.writer, expected);
}

#[test]
fn unexpected_or_repeated_response_poison_session() {
    let mut replies = raw_response(7, 5, 1.);
    replies.extend(raw_response(7, 5, 2.)); // Повтор номера первого запроса.
    replies.extend(raw_response(8, 5, 3.)); // Уже не принимается.
    let mut session = peer(&replies);
    assert_eq!(
        session.exchange(&request(7, 0)).unwrap().command.targets,
        [(ChannelId(5), 1.)]
    );
    for _ in 0..2 {
        assert!(matches!(
            session.exchange(&request(8, 1)),
            Err(ControlError::Transport(_))
        ));
    }
    assert!(!session.healthy);
    let mut future = peer(&raw_response(8, 5, 1.));
    assert!(future.exchange(&request(7, 0)).is_err());
}

#[test]
fn response_wrong_type_unknown_output_nonfinite_and_broken_frame_are_rejected() {
    let correct = raw_response(7, 5, 1.);
    for reply in [
        vec![],
        correct[..2].to_vec(),
        correct[..correct.len() - 1].to_vec(),
        wire(&[3]),
        raw_response(7, 6, 1.),
        raw_response(7, 5, f64::NAN),
        raw_response(7, 5, f64::INFINITY),
        129_u32.to_le_bytes().to_vec(),
        wire(&[4, 7, 0, 0, 0, 0, 0, 0, 0, 255, 255, 255, 255]),
    ] {
        let mut session = peer(&reply);
        assert!(
            session.exchange(&request(7, 0)).is_err(),
            "неверный кадр принят: {reply:?}"
        );
        assert!(!session.healthy);
    }
}

#[test]
fn handshake_rejects_wrong_version_and_different_fields() {
    for (index, byte) in [(5, 2), (9, 127), (13, 8), (21, 9), (37, b'z'), (42, b's')] {
        let mut hello = hello(2);
        hello[index] = byte;
        assert!(
            StreamTransport::open(Cursor::new(wire(&hello)), Vec::new(), contract(), OPTIONS)
                .is_err()
        );
    }
    let mut trailing = hello(2);
    trailing.push(0);
    assert!(
        StreamTransport::open(
            Cursor::new(wire(&trailing)),
            Vec::new(),
            contract(),
            OPTIONS
        )
        .is_err()
    );
}

#[test]
fn limit_rejects_big_outgoing_frame_without_advancing_or_writing() {
    let options = SessionOptions {
        max_message_bytes: 256,
        ..OPTIONS
    };
    let contract = ControlContract::new(
        vec![InputChannel {
            id: SignalId(4),
            name: "кадр".into(),
            unit: "rgb".into(),
            kind: InputKind::Image {
                format: PixelFormat::Rgb8,
                max_capture_age_ns: 0,
                max_delivery_age_ns: 0,
            },
        }],
        contract().outputs().cloned().collect(),
    )
    .unwrap();
    let mut responses = wire(&codec::encode_hello(&contract, options, codec::READY).unwrap());
    responses.extend(raw_response(7, 5, 1.));
    let mut session =
        StreamTransport::open(Cursor::new(responses), Vec::new(), contract, options).unwrap();
    let before = session.writer.len();
    let mut request = request(7, 0);
    request.observation.frames.push(ImageFrame {
        signal: SignalId(4),
        captured: Time(0),
        at: Time(0),
        width: 20,
        height: 20,
        format: PixelFormat::Rgb8,
        valid: true,
        data: std::sync::Arc::from([0; 1200]),
    });
    assert!(session.exchange(&request).is_err());
    assert_eq!(session.writer.len(), before);
    request.observation.frames.clear();
    assert!(session.exchange(&request).is_ok());
}

#[test]
fn worker_rejects_nan_even_in_invalid_wire_sample_and_bad_sequence() {
    struct ConstantController;
    impl Controller for ConstantController {
        fn step(&mut self, _: &Observation, _: &mut Rng) -> Result<Command, ControlError> {
            Ok(Command {
                targets: vec![(ChannelId(5), 1.)],
            })
        }
    }
    let mut bad_sample = request(7, 0);
    bad_sample.observation.samples.push(Sample {
        signal: SignalId(4),
        value: f64::NAN,
        at: Time(0),
        valid: false,
    });
    for request in [bad_sample, request(8, 0)] {
        let mut input = wire(&hello(1));
        input.extend(wire(
            &codec::encode_request(&request, OPTIONS.max_message_bytes).unwrap(),
        ));
        let mut output = Vec::new();
        assert!(
            serve_worker(
                Cursor::new(input),
                &mut output,
                &contract(),
                OPTIONS.max_message_bytes,
                ConstantController
            )
            .is_err()
        );
        let mut expected = wire(&hello(2));
        expected.extend(wire(&[5]));
        assert_eq!(output, expected);
    }
}

struct FixedResponse(f64);

impl ControlTransport for FixedResponse {
    fn exchange(&mut self, request: &ControlRequest) -> Result<ControlResponse, ControlError> {
        Ok(ControlResponse {
            id: request.id,
            command: Command {
                targets: vec![(ChannelId(5), self.0)],
            },
        })
    }
}

#[test]
fn restoration_detects_even_sign_bit_difference() {
    let request = request(7, 0);
    let response = FixedResponse(-0.).exchange(&request).unwrap();
    let snapshot = ProcessSnapshot {
        spec: ProcessSpec {
            program: PathBuf::new(),
            args: vec![],
        },
        contract: contract(),
        options: OPTIONS,
        transcript: vec![(request, response)],
        max_history_bytes: 2048,
    };
    assert!(snapshot.reexecute(&mut FixedResponse(-0.)).is_ok());
    assert!(snapshot.reexecute(&mut FixedResponse(0.)).is_err());
}

#[test]
fn time_and_number_progression_are_checked_before_writing() {
    let mut replies = raw_response(7, 5, 1.);
    replies.extend(raw_response(8, 5, 2.));
    let mut session = peer(&replies);
    assert!(session.exchange(&request(7, 10)).is_ok());
    let before = session.writer.len();
    assert!(session.exchange(&request(8, 9)).is_err());
    assert!(session.exchange(&request(7, 11)).is_err());
    assert_eq!(session.writer.len(), before);
    assert!(session.exchange(&request(8, 11)).is_ok());

    let mut seq = Sequence::new(RequestId(u64::MAX));
    let last = request(u64::MAX, 0);
    assert!(seq.check(&last).is_ok());
    seq.advance(&last);
    assert!(seq.check(&request(0, 1)).is_err());
}

#[test]
fn adapter_fails_closed_over_custom_transport() {
    struct IncorrectThenCorrect(usize);
    impl ControlTransport for IncorrectThenCorrect {
        fn exchange(&mut self, request: &ControlRequest) -> Result<ControlResponse, ControlError> {
            self.0 += 1;
            let id = RequestId(request.id.0 + u64::from(self.0 == 1));
            Ok(ControlResponse {
                id,
                command: Command::default(),
            })
        }
    }
    let mut adapter =
        ExternalController::new(IncorrectThenCorrect(0), contract(), OPTIONS.first_request);
    let observation = Observation::default();
    assert!(adapter.request(&observation).is_err());
    assert!(adapter.request(&observation).is_err());
    assert_eq!(adapter.transport.0, 1);
}
