//! `sim-signal` — общий словарь платформы. Здесь нет физики и нет истинного состояния мира:
//! этот крейт можно давать контроллеру.
//!
//! * [`Time`], [`Rate`], [`Clock`] — целочисленная шкала времени без накопления ошибки;
//! * [`Rng`] — детерминированные потоки случайности по зерну мира и имени компонента;
//! * [`Observation`], [`Command`] — всё, чем контроллер обменивается с миром;
//! * [`Phase`] — порядок работы компонентов внутри одного момента времени.

mod rng;
mod signal;
mod time;

pub use rng::Rng;
pub use signal::{
    ChannelId, Command, ImageFrame, Observation, PixelFormat, Sample, SignalId, SignalSpec,
    SignalTable,
};
pub use time::{Clock, Phase, Rate, Time};

/// FNV-1a, 64 бита: стабильна между запусками и версиями Rust, в отличие от `DefaultHasher`.
pub fn fnv1a(bytes: impl IntoIterator<Item = u8>) -> u64 {
    bytes.into_iter().fold(0xcbf2_9ce4_8422_2325, |h, b| {
        (h ^ u64::from(b)).wrapping_mul(0x0000_0100_0000_01b3)
    })
}
