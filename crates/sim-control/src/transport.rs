//! Сеанс протокола через потоки и локальный процесс через stdin/stdout.

use std::ffi::OsString;
use std::io::{Read, Write};
use std::path::PathBuf;
use std::process::{Child, ChildStdin, ChildStdout, Stdio};

use sim_signal::{Command, Observation, Rng, Time};

use crate::codec;
use crate::{
    ControlContract, ControlError, ControlRequest, ControlResponse, ControlTransport, Controller,
    RequestId,
};

pub const PROTOCOL_VERSION: u32 = 1;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SessionOptions {
    pub first_request: RequestId,
    /// Зерно потока работника, одинаковое при восстановлении.
    pub seed: u64,
    /// Верхняя граница размера полезной нагрузки одного сообщения, в байтах.
    pub max_message_bytes: u32,
}

#[derive(Clone, Copy)]
struct Sequence {
    next: Option<RequestId>,
    last: Option<Time>,
}

impl Sequence {
    fn new(id: RequestId) -> Self {
        Self {
            next: Some(id),
            last: None,
        }
    }

    fn check(&self, request: &ControlRequest) -> Result<(), ControlError> {
        if self.next != Some(request.id) {
            return Err(codec::error(format!(
                "неожиданный номер запроса {}",
                request.id.0
            )));
        }
        if self
            .last
            .is_some_and(|previous| request.observation.time < previous)
        {
            return Err(ControlError::InvalidObservation(
                "время сеанса пошло назад".into(),
            ));
        }
        Ok(())
    }

    fn advance(&mut self, request: &ControlRequest) {
        self.next = request.id.0.checked_add(1).map(RequestId);
        self.last = Some(request.observation.time);
    }
}

/// Один согласованный сеанс. Один запрос в работе, ответы принимаются строго по порядку.
pub struct StreamTransport<R, W> {
    reader: R,
    writer: W,
    contract: ControlContract,
    options: SessionOptions,
    sequence: Sequence,
    healthy: bool,
}

impl<R: Read, W: Write> StreamTransport<R, W> {
    pub fn open(
        mut reader: R,
        mut writer: W,
        contract: ControlContract,
        options: SessionOptions,
    ) -> Result<Self, ControlError> {
        let hello = codec::encode_hello(&contract, options, codec::HELLO)?;
        codec::send(&mut writer, &hello)?;
        let payload = expect_message(&mut reader, options.max_message_bytes)?;
        let (peer, agreed) = codec::decode_hello(&payload, codec::READY)?;
        if peer != contract || agreed != options {
            return Err(codec::error("работник подтвердил другую схему сеанса"));
        }
        Ok(Self {
            reader,
            writer,
            contract,
            options,
            sequence: Sequence::new(options.first_request),
            healthy: true,
        })
    }

    fn exchange_measured(
        &mut self,
        request: &ControlRequest,
    ) -> Result<(ControlResponse, ControlRequest, usize), ControlError> {
        if !self.healthy {
            return Err(codec::error("сеанс прекращён после ошибки обмена"));
        }
        self.sequence.check(request)?;
        // Проверяем все входные данные до фильтрации, включая NaN в недействительном отсчёте.
        let request = ControlRequest {
            id: request.id,
            observation: self.contract.observation(&request.observation)?.usable(),
        };
        let limit = self.options.max_message_bytes;
        let outgoing = codec::encode_request(&request, limit)?;
        // После отправки сбой оставляет состояние работника неизвестным.
        self.healthy = false;
        codec::send(&mut self.writer, &outgoing)?;
        let payload = expect_message(&mut self.reader, limit)?;
        let response = codec::decode_response(&payload)?;
        self.contract
            .validate_response(&request, &response, request.observation.time)?;
        self.sequence.advance(&request);
        self.healthy = true;
        Ok((
            response,
            request,
            outgoing.len().saturating_add(payload.len()),
        ))
    }
}

impl<R: Read, W: Write> ControlTransport for StreamTransport<R, W> {
    fn exchange(&mut self, request: &ControlRequest) -> Result<ControlResponse, ControlError> {
        self.exchange_measured(request)
            .map(|(response, _, _)| response)
    }
}

fn expect_message(reader: &mut impl Read, limit: u32) -> Result<Vec<u8>, ControlError> {
    let payload =
        codec::receive(reader, limit)?.ok_or_else(|| codec::error("поток закрылся до ответа"))?;
    if payload == [codec::REJECTED] {
        return Err(ControlError::Execution(
            "работник отклонил сообщение и завершил сеанс".into(),
        ));
    }
    Ok(payload)
}

/// Цикл работника для stdin/stdout или тех же потоков стенда.
/// При ошибке подтверждение отказа передаётся один раз, а сеанс закрывается.
pub fn serve_worker(
    mut reader: impl Read,
    mut writer: impl Write,
    contract: &ControlContract,
    max_message_bytes: u32,
    mut controller: impl Controller,
) -> Result<(), ControlError> {
    let result = run_worker(
        &mut reader,
        &mut writer,
        contract,
        max_message_bytes,
        &mut controller,
    );
    if result.is_err()
        && let Ok(payload) = codec::rejection(max_message_bytes)
    {
        let _ = codec::send(&mut writer, &payload);
    }
    result
}

fn run_worker(
    reader: &mut impl Read,
    writer: &mut impl Write,
    contract: &ControlContract,
    max_message_bytes: u32,
    controller: &mut impl Controller,
) -> Result<(), ControlError> {
    let hello = expect_message(reader, max_message_bytes)?;
    let (peer, options) = codec::decode_hello(&hello, codec::HELLO)?;
    let limit = options.max_message_bytes;
    if peer != *contract || limit > max_message_bytes {
        return Err(codec::error(
            "каналы или предел работника не совпадают с запросом",
        ));
    }
    let ready = codec::encode_hello(contract, options, codec::READY)?;
    codec::send(writer, &ready)?;
    let mut sequence = Sequence::new(options.first_request);
    let mut rng = Rng::stream(options.seed, "внешний контроллер");
    while let Some(payload) = codec::receive(reader, limit)? {
        let mut request = codec::decode_request(&payload)?;
        sequence.check(&request)?;
        request.observation = contract.observation(&request.observation)?.usable();
        let command = controller.step(&request.observation, &mut rng)?;
        let response = ControlResponse {
            id: request.id,
            command,
        };
        contract.validate_response(&request, &response, request.observation.time)?;
        let encoded = codec::encode_response(&response, limit)?;
        codec::send(writer, &encoded)?;
        sequence.advance(&request);
    }
    Ok(())
}

#[derive(Clone, Debug)]
pub struct ProcessSpec {
    /// Путь исполняемого файла. Аргументы передаются без оболочки.
    pub program: PathBuf,
    pub args: Vec<OsString>,
}

struct Worker(Child);

impl Drop for Worker {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// Снимок для повторного живого исполнения с побитовой проверкой ответов.
#[derive(Clone, Debug)]
pub struct ProcessSnapshot {
    spec: ProcessSpec,
    contract: ControlContract,
    options: SessionOptions,
    transcript: Vec<(ControlRequest, ControlResponse)>,
    max_history_bytes: usize,
}

pub struct ProcessTransport {
    session: StreamTransport<ChildStdout, ChildStdin>,
    worker: Worker,
    spec: ProcessSpec,
    transcript: Option<Vec<(ControlRequest, ControlResponse)>>,
    max_history_bytes: Option<usize>,
    history_bytes: usize,
}

impl ProcessTransport {
    /// История ограничена суммой размеров полезной нагрузки запросов и ответов.
    /// При превышении лимита или при `None` продолжение возможно, снимок возвращает ошибку.
    pub fn spawn(
        spec: ProcessSpec,
        contract: ControlContract,
        options: SessionOptions,
        max_history_bytes: Option<usize>,
    ) -> Result<Self, ControlError> {
        Self::spawn_supervised(spec, contract, options, max_history_bytes, |_| {})
    }

    /// Супервизор получает PID перед блокирующим рукопожатием.
    pub fn spawn_supervised(
        spec: ProcessSpec,
        contract: ControlContract,
        options: SessionOptions,
        max_history_bytes: Option<usize>,
        on_spawn: impl FnOnce(u32),
    ) -> Result<Self, ControlError> {
        let mut worker = Worker(
            std::process::Command::new(&spec.program)
                .args(&spec.args)
                .env_clear()
                .stdin(Stdio::piped())
                .stdout(Stdio::piped())
                .spawn()
                .map_err(codec::io_error)?,
        );
        on_spawn(worker.0.id());
        let stdout = worker
            .0
            .stdout
            .take()
            .ok_or_else(|| codec::error("нет stdout работника"))?;
        let stdin = worker
            .0
            .stdin
            .take()
            .ok_or_else(|| codec::error("нет stdin работника"))?;
        let session = StreamTransport::open(stdout, stdin, contract, options)?;
        Ok(Self {
            session,
            worker,
            spec,
            max_history_bytes,
            transcript: max_history_bytes.map(|_| Vec::new()),
            history_bytes: 0,
        })
    }

    /// Внешний супервизор может завершить зависшего работника по этому PID.
    pub fn worker_id(&self) -> u32 {
        self.worker.0.id()
    }

    pub fn snapshot(&self) -> Result<ProcessSnapshot, ControlError> {
        if let (true, Some(transcript), Some(max_history_bytes)) = (
            self.session.healthy,
            &self.transcript,
            self.max_history_bytes,
        ) {
            Ok(ProcessSnapshot {
                spec: self.spec.clone(),
                contract: self.session.contract.clone(),
                options: self.session.options,
                transcript: transcript.clone(),
                max_history_bytes,
            })
        } else {
            Err(codec::error("восстановление живого сеанса недоступно"))
        }
    }

    pub fn restore(snapshot: &ProcessSnapshot) -> Result<Self, ControlError> {
        Self::restore_supervised(snapshot, |_| {})
    }

    pub fn restore_supervised(
        snapshot: &ProcessSnapshot,
        on_spawn: impl FnOnce(u32),
    ) -> Result<Self, ControlError> {
        let mut process = Self::spawn_supervised(
            snapshot.spec.clone(),
            snapshot.contract.clone(),
            snapshot.options,
            Some(snapshot.max_history_bytes),
            on_spawn,
        )?;
        snapshot.reexecute(&mut process)?;
        Ok(process)
    }
}

impl ProcessSnapshot {
    fn reexecute(&self, process: &mut impl ControlTransport) -> Result<(), ControlError> {
        let limit = self.options.max_message_bytes;
        for (request, expected) in &self.transcript {
            let actual = process.exchange(request)?;
            if codec::encode_response(expected, limit)? != codec::encode_response(&actual, limit)? {
                return Err(codec::error(
                    "ответ при восстановлении отличается от исходного",
                ));
            }
        }
        Ok(())
    }
}

impl ControlTransport for ProcessTransport {
    fn exchange(&mut self, request: &ControlRequest) -> Result<ControlResponse, ControlError> {
        let (response, sent, size) = self.session.exchange_measured(request)?;
        if let (Some(history), Some(limit)) = (&mut self.transcript, self.max_history_bytes) {
            if let Some(total) = self
                .history_bytes
                .checked_add(size)
                .filter(|&total| total <= limit)
            {
                history.push((sent, response.clone()));
                self.history_bytes = total;
            } else {
                self.transcript = None;
            }
        }
        Ok(response)
    }
}

/// Адаптер симуляции, выделяющий номера запросов. Доставку и срок регулирует принимающая сторона.
pub struct ExternalController<T> {
    transport: T,
    contract: ControlContract,
    sequence: Sequence,
    healthy: bool,
}

impl<T: ControlTransport> ExternalController<T> {
    pub fn new(transport: T, contract: ControlContract, first: RequestId) -> Self {
        Self {
            transport,
            contract,
            sequence: Sequence::new(first),
            healthy: true,
        }
    }

    /// Возвращает запрос вместе с ответом, чтобы принимающая сторона проверила номер и срок.
    pub fn request(
        &mut self,
        observation: &Observation,
    ) -> Result<(ControlRequest, ControlResponse), ControlError> {
        if !self.healthy {
            return Err(codec::error("сеанс прекращён после ошибки обмена"));
        }
        let id = self
            .sequence
            .next
            .ok_or_else(|| codec::error("номера запросов исчерпаны"))?;
        let request = ControlRequest {
            id,
            observation: self.contract.observation(observation)?.usable(),
        };
        // После передачи ошибки транспорта и проверки ответа делают ход сеанса неоднозначным.
        self.sequence.check(&request)?;
        self.healthy = false;
        let response = self.transport.exchange(&request)?;
        self.contract
            .validate_response(&request, &response, observation.time)?;
        self.sequence.advance(&request);
        self.healthy = true;
        Ok((request, response))
    }
}

impl ExternalController<ProcessTransport> {
    pub fn fork(&self) -> Result<Self, ControlError> {
        self.fork_supervised(|_| {})
    }

    pub fn fork_supervised(&self, on_spawn: impl FnOnce(u32)) -> Result<Self, ControlError> {
        if !self.healthy {
            return Err(codec::error("восстановление живого сеанса недоступно"));
        }
        Ok(Self {
            transport: ProcessTransport::restore_supervised(&self.transport.snapshot()?, on_spawn)?,
            contract: self.contract.clone(),
            sequence: self.sequence,
            healthy: true,
        })
    }
}

impl<T: ControlTransport> Controller for ExternalController<T> {
    fn step(&mut self, observation: &Observation, _rng: &mut Rng) -> Result<Command, ControlError> {
        self.request(observation)
            .map(|(_, response)| response.command)
    }
}

#[cfg(test)]
mod tests;
