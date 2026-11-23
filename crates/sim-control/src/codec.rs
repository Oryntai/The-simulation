//! Двоичные сообщения протокола. Все числа в little-endian; f64 передаётся побитово.

use std::io::{Read, Write};
use std::sync::Arc;

use sim_signal::{
    ChannelId, Command, ImageFrame, Observation, PixelFormat, Sample, SignalId, Time,
};

use crate::transport::{PROTOCOL_VERSION, SessionOptions};
use crate::{
    ControlContract, ControlError, ControlRequest, ControlResponse, InputChannel, InputKind,
    OutputChannel, RequestId,
};

pub(crate) const HELLO: u8 = 1;
pub(crate) const READY: u8 = 2;
pub(crate) const REQUEST: u8 = 3;
pub(crate) const RESPONSE: u8 = 4;
pub(crate) const REJECTED: u8 = 5;
// Маркер формата, затем версия протокола. Типы сообщений — номера схемы версии 1.
const MAGIC: &[u8] = b"SIMC";

pub(crate) fn error(message: impl Into<String>) -> ControlError {
    ControlError::Transport(message.into())
}

pub(crate) fn io_error(err: std::io::Error) -> ControlError {
    error(format!("ошибка канала: {err}"))
}

pub(crate) fn send(writer: &mut impl Write, payload: &[u8]) -> Result<(), ControlError> {
    let size = u32::try_from(payload.len()).map_err(|_| error("размер кадра превышает u32"))?;
    writer.write_all(&size.to_le_bytes()).map_err(io_error)?;
    writer.write_all(payload).map_err(io_error)?;
    writer.flush().map_err(io_error)
}

/// EOF разрешён только между целыми сообщениями на принимающей стороне.
pub(crate) fn receive(reader: &mut impl Read, limit: u32) -> Result<Option<Vec<u8>>, ControlError> {
    let mut header = [0; size_of::<u32>()];
    loop {
        match reader.read(&mut header[..1]) {
            Ok(0) => return Ok(None),
            Ok(_) => break,
            Err(err) if err.kind() == std::io::ErrorKind::Interrupted => continue,
            Err(err) => return Err(io_error(err)),
        }
    }
    reader.read_exact(&mut header[1..]).map_err(io_error)?;
    let size = u32::from_le_bytes(header);
    if size == 0 || size > limit {
        return Err(error(format!("недопустимый размер кадра {size}")));
    }
    let mut payload = Vec::new();
    payload
        .try_reserve_exact(size as usize)
        .map_err(|_| error("не удалось выделить память под сообщение"))?;
    payload.resize(size as usize, 0);
    reader.read_exact(&mut payload).map_err(io_error)?;
    Ok(Some(payload))
}

struct Encoder {
    bytes: Vec<u8>,
    limit: usize,
}

impl Encoder {
    fn new(tag: u8, limit: u32) -> Result<Self, ControlError> {
        let mut encoder = Self {
            bytes: Vec::new(),
            limit: limit as usize,
        };
        encoder.put(&[tag])?;
        Ok(encoder)
    }

    fn put(&mut self, bytes: &[u8]) -> Result<(), ControlError> {
        if bytes.len() > self.limit - self.bytes.len() {
            return Err(error("сообщение превышает согласованный размер"));
        }
        self.bytes.extend_from_slice(bytes);
        Ok(())
    }

    fn u32(&mut self, value: u32) -> Result<(), ControlError> {
        self.put(&value.to_le_bytes())
    }
    fn u64(&mut self, value: u64) -> Result<(), ControlError> {
        self.put(&value.to_le_bytes())
    }
    fn count(&mut self, count: usize) -> Result<(), ControlError> {
        self.u32(u32::try_from(count).map_err(|_| error("поле превышает u32"))?)
    }

    fn blob(&mut self, bytes: &[u8]) -> Result<(), ControlError> {
        self.count(bytes.len())?;
        self.put(bytes)
    }
}

struct Decoder<'a>(&'a [u8]);

impl<'a> Decoder<'a> {
    fn new(payload: &'a [u8], tag: u8) -> Result<Self, ControlError> {
        match payload.split_first() {
            Some((&kind, tail)) if kind == tag => Ok(Self(tail)),
            _ => Err(error("неожиданный тип сообщения")),
        }
    }

    fn take(&mut self, size: usize) -> Result<&'a [u8], ControlError> {
        let (head, tail) = self
            .0
            .split_at_checked(size)
            .ok_or_else(|| error("поле оборвано"))?;
        self.0 = tail;
        Ok(head)
    }

    fn array<const N: usize>(&mut self) -> Result<[u8; N], ControlError> {
        let mut bytes = [0; N];
        bytes.copy_from_slice(self.take(N)?);
        Ok(bytes)
    }

    fn u8(&mut self) -> Result<u8, ControlError> {
        Ok(self.array::<1>()?[0])
    }
    fn u32(&mut self) -> Result<u32, ControlError> {
        Ok(u32::from_le_bytes(self.array()?))
    }
    fn u64(&mut self) -> Result<u64, ControlError> {
        Ok(u64::from_le_bytes(self.array()?))
    }

    fn f64(&mut self) -> Result<f64, ControlError> {
        let value = f64::from_bits(self.u64()?);
        if value.is_finite() {
            Ok(value)
        } else {
            Err(error("неконечное число в сообщении"))
        }
    }

    fn boolean(&mut self) -> Result<bool, ControlError> {
        match self.u8()? {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err(error("недопустимое логическое значение")),
        }
    }

    fn count(&mut self, item_min_size: usize) -> Result<usize, ControlError> {
        let count = self.u32()? as usize;
        if count > self.0.len() / item_min_size {
            return Err(error("число элементов превышает размер сообщения"));
        }
        Ok(count)
    }

    fn blob(&mut self) -> Result<&'a [u8], ControlError> {
        let size = self.u32()? as usize;
        self.take(size)
    }

    fn string(&mut self) -> Result<String, ControlError> {
        String::from_utf8(self.blob()?.to_vec()).map_err(|_| error("строка не UTF-8"))
    }

    fn end(self) -> Result<(), ControlError> {
        if self.0.is_empty() {
            Ok(())
        } else {
            Err(error("лишние байты сообщения"))
        }
    }
}

fn format_id(format: PixelFormat) -> u8 {
    match format {
        PixelFormat::Rgb8 => 1,
        PixelFormat::DepthMm16 => 2,
    }
}

fn parse_format(id: u8) -> Result<PixelFormat, ControlError> {
    match id {
        1 => Ok(PixelFormat::Rgb8),
        2 => Ok(PixelFormat::DepthMm16),
        _ => Err(error("неизвестный формат кадра")),
    }
}

pub(crate) fn encode_hello(
    contract: &ControlContract,
    options: SessionOptions,
    tag: u8,
) -> Result<Vec<u8>, ControlError> {
    let mut out = Encoder::new(tag, options.max_message_bytes)?;
    out.put(MAGIC)?;
    out.u32(PROTOCOL_VERSION)?;
    out.u32(options.max_message_bytes)?;
    out.u64(options.first_request.0)?;
    out.u64(options.seed)?;
    out.count(contract.inputs.len())?;
    for input in contract.inputs() {
        out.u32(input.id.0)?;
        out.blob(input.name.as_bytes())?;
        out.blob(input.unit.as_bytes())?;
        match input.kind {
            InputKind::Scalar { max_age_ns } => {
                out.put(&[1])?;
                out.u64(max_age_ns)?;
            }
            InputKind::Image {
                format,
                max_capture_age_ns,
                max_delivery_age_ns,
            } => {
                out.put(&[2, format_id(format)])?;
                out.u64(max_capture_age_ns)?;
                out.u64(max_delivery_age_ns)?;
            }
        }
    }
    out.count(contract.outputs.len())?;
    for output in contract.outputs() {
        out.u32(output.id.0)?;
        out.blob(output.name.as_bytes())?;
        out.blob(output.unit.as_bytes())?;
        out.u64(output.max_age_ns)?;
    }
    Ok(out.bytes)
}

pub(crate) fn decode_hello(
    payload: &[u8],
    tag: u8,
) -> Result<(ControlContract, SessionOptions), ControlError> {
    let mut input = Decoder::new(payload, tag)?;
    if input.take(MAGIC.len())? != MAGIC || input.u32()? != PROTOCOL_VERSION {
        return Err(error("несовместимая версия протокола"));
    }
    let options = SessionOptions {
        max_message_bytes: input.u32()?,
        first_request: RequestId(input.u64()?),
        seed: input.u64()?,
    };
    let mut inputs = Vec::new();
    // Минимальная запись: id, две длины строк, тип и срок.
    for _ in 0..input.count(3 * size_of::<u32>() + size_of::<u8>() + size_of::<u64>())? {
        let id = SignalId(input.u32()?);
        let name = input.string()?;
        let unit = input.string()?;
        let kind = match input.u8()? {
            1 => InputKind::Scalar {
                max_age_ns: input.u64()?,
            },
            2 => InputKind::Image {
                format: parse_format(input.u8()?)?,
                max_capture_age_ns: input.u64()?,
                max_delivery_age_ns: input.u64()?,
            },
            _ => return Err(error("неизвестный тип входа")),
        };
        inputs.push(InputChannel {
            id,
            name,
            unit,
            kind,
        });
    }
    let mut outputs = Vec::new();
    for _ in 0..input.count(3 * size_of::<u32>() + size_of::<u64>())? {
        outputs.push(OutputChannel {
            id: ChannelId(input.u32()?),
            name: input.string()?,
            unit: input.string()?,
            max_age_ns: input.u64()?,
        });
    }
    input.end()?;
    Ok((ControlContract::new(inputs, outputs)?, options))
}

pub(crate) fn encode_request(
    request: &ControlRequest,
    limit: u32,
) -> Result<Vec<u8>, ControlError> {
    let observation = &request.observation;
    let mut out = Encoder::new(REQUEST, limit)?;
    out.u64(request.id.0)?;
    out.u64(observation.time.0)?;
    out.count(observation.samples.len())?;
    for sample in &observation.samples {
        out.u32(sample.signal.0)?;
        out.u64(sample.value.to_bits())?;
        out.u64(sample.at.0)?;
        out.put(&[u8::from(sample.valid)])?;
    }
    out.count(observation.frames.len())?;
    for frame in &observation.frames {
        out.u32(frame.signal.0)?;
        out.u64(frame.captured.0)?;
        out.u64(frame.at.0)?;
        out.u32(frame.width)?;
        out.u32(frame.height)?;
        out.put(&[format_id(frame.format), u8::from(frame.valid)])?;
        out.blob(&frame.data)?;
    }
    Ok(out.bytes)
}

pub(crate) fn decode_request(payload: &[u8]) -> Result<ControlRequest, ControlError> {
    let mut input = Decoder::new(payload, REQUEST)?;
    let id = RequestId(input.u64()?);
    let time = Time(input.u64()?);
    let mut samples = Vec::new();
    for _ in 0..input.count(size_of::<u32>() + 2 * size_of::<u64>() + size_of::<u8>())? {
        samples.push(Sample {
            signal: SignalId(input.u32()?),
            value: input.f64()?,
            at: Time(input.u64()?),
            valid: input.boolean()?,
        });
    }
    let mut frames = Vec::new();
    for _ in 0..input.count(4 * size_of::<u32>() + 2 * size_of::<u64>() + 2 * size_of::<u8>())? {
        frames.push(ImageFrame {
            signal: SignalId(input.u32()?),
            captured: Time(input.u64()?),
            at: Time(input.u64()?),
            width: input.u32()?,
            height: input.u32()?,
            format: parse_format(input.u8()?)?,
            valid: input.boolean()?,
            data: Arc::from(input.blob()?),
        });
    }
    input.end()?;
    Ok(ControlRequest {
        id,
        observation: Observation {
            time,
            samples,
            frames,
        },
    })
}

pub(crate) fn encode_response(
    response: &ControlResponse,
    limit: u32,
) -> Result<Vec<u8>, ControlError> {
    let mut out = Encoder::new(RESPONSE, limit)?;
    out.u64(response.id.0)?;
    out.count(response.command.targets.len())?;
    for &(id, value) in &response.command.targets {
        out.u32(id.0)?;
        out.u64(value.to_bits())?;
    }
    Ok(out.bytes)
}

pub(crate) fn decode_response(payload: &[u8]) -> Result<ControlResponse, ControlError> {
    let mut input = Decoder::new(payload, RESPONSE)?;
    let id = RequestId(input.u64()?);
    let mut targets = Vec::new();
    for _ in 0..input.count(size_of::<u32>() + size_of::<u64>())? {
        targets.push((ChannelId(input.u32()?), input.f64()?));
    }
    input.end()?;
    Ok(ControlResponse {
        id,
        command: Command { targets },
    })
}

pub(crate) fn rejection(limit: u32) -> Result<Vec<u8>, ControlError> {
    Ok(Encoder::new(REJECTED, limit)?.bytes)
}
