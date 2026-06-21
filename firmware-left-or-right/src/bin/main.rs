#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::timer::timg::TimerGroup;
use esp_println as _;

// This creates a default app-descriptor required by the esp-idf bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

// --- Tunable parameters -----------------------------------------------------
//
// Mics are ~5cm apart on the breadboard. Speed of sound ~343 m/s, so the
// largest physically-possible arrival difference (sound coming straight off one
// side) is 0.05 / 343 ≈ 146 µs. Real KY-038 modules add comparator + capsule
// jitter that is a sizeable chunk of that, so we do NOT pretend to resolve a
// fine angle. We report three states: LEFT, RIGHT, and CENTER (delta too small
// to trust). The LED encodes the call -- see report() below.

/// Spacing between the two mics. Informational only for now.
const MIC_SPACING_CM: u32 = 5;

/// Below this |delta|, we can't trust which mic was truly first -- call it
/// CENTER. Start generous (jitter between two cheap modules is real) and tighten
/// it once you've watched the logged deltas from known left/right claps.
const CENTER_DEADBAND_US: u64 = 60;

/// After a clap, ignore everything for this long. One clap in a room produces
/// the direct impulse plus echoes arriving within milliseconds; without this the
/// reflections re-trigger and spam garbage.
const BLANKING_MS: u64 = 200;

/// If the first mic fires but the second never does (only one heard it, or the
/// other module is mistuned), stop waiting after this and report the side that
/// did fire.
const PAIR_TIMEOUT_MS: u64 = 50;
// ----------------------------------------------------------------------------

#[derive(Clone, Copy, defmt::Format)]
enum Side {
    Left,
    Right,
    Center,
}

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    // generator parameters: --chip esp32c3 -o unstable-hal -o embassy -o defmt -o esp-backtrace
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    // LED on GPIO3 (same as the listener build): GPIO3 -> 330R -> LED -> GND.
    let mut led = Output::new(peripherals.GPIO3, Level::Low, OutputConfig::default());

    // Two mic D0 lines. LEFT mic on GPIO4 (where the single mic lived), RIGHT
    // mic on GPIO5. No pull -- the LM393 comparator drives the line actively.
    // D0 goes HIGH on a clap, so we wait for the rising edge.
    let in_cfg = InputConfig::default().with_pull(Pull::None);
    let mut left_mic = Input::new(peripherals.GPIO4, in_cfg);
    let mut right_mic = Input::new(peripherals.GPIO5, in_cfg);

    info!(
        "left/right detector ready. mics ~{}cm apart, deadband {}us",
        MIC_SPACING_CM, CENTER_DEADBAND_US
    );

    loop {
        // Race both rising edges. Whichever resolves first is the near mic --
        // its sound front arrived first. `select` polls both and hands back the
        // one that completed.
        let first =
            select(left_mic.wait_for_rising_edge(), right_mic.wait_for_rising_edge()).await;
        let t_first = Instant::now();
        let near = match first {
            Either::First(()) => Side::Left,
            Either::Second(()) => Side::Right,
        };

        // Now wait for the *other* mic's edge to learn the gap. Bound it so a
        // lone trigger doesn't hang us forever.
        let other_edge = async {
            match near {
                Side::Left => right_mic.wait_for_rising_edge().await,
                _ => left_mic.wait_for_rising_edge().await,
            }
        };
        let t_second =
            match select(other_edge, Timer::after(Duration::from_millis(PAIR_TIMEOUT_MS))).await {
                Either::First(()) => Some(Instant::now()),
                Either::Second(()) => None, // other mic never fired
            };

        let side = match t_second {
            Some(t2) => {
                let delta_us = t2.duration_since(t_first).as_micros();
                if delta_us <= CENTER_DEADBAND_US {
                    info!("delta {}us within deadband -> CENTER", delta_us);
                    Side::Center
                } else {
                    info!("near={} by {}us", near, delta_us);
                    near
                }
            }
            None => {
                info!("only {} fired (other silent) -> {}", near, near);
                near
            }
        };

        report(&mut led, side).await;

        // Blank out echoes and any pot-induced chatter, then resume listening.
        Timer::after(Duration::from_millis(BLANKING_MS)).await;
    }
}

/// Show the decision on the single LED:
///   LEFT   -> 1 blink
///   RIGHT  -> 2 blinks
///   CENTER -> 1 long solid pulse
/// A cheap unambiguous readout until the motors exist to point at the source.
async fn report(led: &mut Output<'_>, side: Side) {
    let blinks = match side {
        Side::Left => 1,
        Side::Right => 2,
        Side::Center => {
            led.set_high();
            Timer::after(Duration::from_millis(600)).await;
            led.set_low();
            return;
        }
    };
    for _ in 0..blinks {
        led.set_high();
        Timer::after(Duration::from_millis(120)).await;
        led.set_low();
        Timer::after(Duration::from_millis(150)).await;
    }
}
