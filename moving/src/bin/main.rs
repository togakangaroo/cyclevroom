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
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{DriveMode, Level, Output, OutputConfig};
use esp_hal::ledc::channel::{self, ChannelIFace};
use esp_hal::ledc::timer::{self, TimerIFace};
use esp_hal::ledc::{LSGlobalClkSource, Ledc, LowSpeed};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_println as _;

// This creates a default app-descriptor required by the esp-idf bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

// --- Tunables ---------------------------------------------------------------
//
// This is a dead-reckoned drive test: no encoders, no mic feedback. We drive
// each motor for a fixed *time*, so "how far" and "how much of a turn" are set
// by the durations below, tuned by watching the bot on the floor. Expect to
// twiddle these -- the two motors don't spin at identical RPM (~20% mismatch is
// known and fine) and the foam chassis isn't precise.

/// Straight-line speed as a % duty cycle on the L298N enable pins. "Slow" per
/// the plan -- gentle, easy to film, easy on the foam chassis. Below ~30% the
/// motors may not overcome stiction and stall; raise if a wheel doesn't start.
const DRIVE_DUTY: u8 = 40;

/// Turn speed. In-place spins fight more friction (both wheels scrubbing
/// sideways), so give them a bit more push than the straights.
const TURN_DUTY: u8 = 50;

/// Long straight leg of the rectangle.
const LONG_LEG_MS: u64 = 1000;
/// Short straight leg of the rectangle.
const SHORT_LEG_MS: u64 = 500;

/// How long an in-place 90-degree spin takes at TURN_DUTY. Pure guess to start
/// -- watch the bot and adjust until a spin lands near a right angle. A 180 is
/// just two of these back to back.
const TURN_90_MS: u64 = 600;

/// Coast/settle pause between primitive moves. Lets momentum die so one leg
/// doesn't bleed into the next, and makes the choreography legible when
/// watching.
const SETTLE_MS: u64 = 300;

/// PWM frequency for the enable pins. A few kHz is well above the motor's
/// mechanical response, so the motor sees the average voltage; low enough that
/// the L298N's slow switching isn't a problem. Out of audible-whine territory.
const PWM_FREQ_HZ: u32 = 5000;
// ----------------------------------------------------------------------------

/// One DC motor behind an L298N channel: a PWM'd enable line (speed) plus two
/// direction lines. (in1, in2) = (High, Low) spins one way, (Low, High) the
/// other, (Low, Low) is a fast brake. Which physical direction is "forward"
/// depends on how you soldered the motor leads -- flip the leads (or swap the
/// two `set_*` calls) if a wheel runs backwards.
struct Motor<'d> {
    pwm: channel::Channel<'d, LowSpeed>,
    in1: Output<'d>,
    in2: Output<'d>,
}

impl<'d> Motor<'d> {
    fn forward(&mut self, duty_pct: u8) {
        self.in1.set_high();
        self.in2.set_low();
        self.pwm.set_duty(duty_pct).unwrap();
    }

    fn reverse(&mut self, duty_pct: u8) {
        self.in1.set_low();
        self.in2.set_high();
        self.pwm.set_duty(duty_pct).unwrap();
    }

    /// Brake: both direction lines low, enable off. Motor terminals shorted =
    /// active stop, quicker than coasting.
    fn stop(&mut self) {
        self.in1.set_low();
        self.in2.set_low();
        self.pwm.set_duty(0).unwrap();
    }
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

    // --- LEDC PWM setup -----------------------------------------------------
    // The LEDC peripheral generates the PWM for the two L298N enable pins. One
    // shared low-speed timer sets the frequency + resolution; each motor gets
    // its own channel so they can run at independent duty cycles.
    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    lstimer0
        .configure(timer::config::Config {
            // 8-bit duty resolution -> 256 steps, plenty for motor speed. The
            // set_duty() API takes a percentage regardless, but resolution must
            // be high enough that PWM_FREQ_HZ is achievable from the APB clock.
            duty: timer::config::Duty::Duty8Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(PWM_FREQ_HZ),
        })
        .unwrap();

    // --- Motor A (LEFT wheel) ----------------------------------------------
    //   ENA -> GPIO0   (PWM speed)
    //   IN1 -> GPIO1
    //   IN2 -> GPIO10
    let mut ch_a = ledc.channel(channel::Number::Channel0, peripherals.GPIO0);
    ch_a.configure(channel::config::Config {
        timer: &lstimer0,
        duty_pct: 0,
        drive_mode: DriveMode::PushPull,
    })
    .unwrap();
    let mut left = Motor {
        pwm: ch_a,
        in1: Output::new(peripherals.GPIO1, Level::Low, OutputConfig::default()),
        in2: Output::new(peripherals.GPIO10, Level::Low, OutputConfig::default()),
    };

    // --- Motor B (RIGHT wheel) ---------------------------------------------
    //   ENB -> GPIO6   (PWM speed)
    //   IN3 -> GPIO7
    //   IN4 -> GPIO5
    let mut ch_b = ledc.channel(channel::Number::Channel1, peripherals.GPIO6);
    ch_b.configure(channel::config::Config {
        timer: &lstimer0,
        duty_pct: 0,
        drive_mode: DriveMode::PushPull,
    })
    .unwrap();
    let mut right = Motor {
        pwm: ch_b,
        in1: Output::new(peripherals.GPIO7, Level::Low, OutputConfig::default()),
        in2: Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default()),
    };

    info!("motors ready, starting drive routine");

    // Small pause before moving so you can set the bot down / start filming.
    Timer::after(Duration::from_secs(2)).await;

    // --- The routine --------------------------------------------------------
    // Rightward rectangle: long straight, right turn, short straight, right
    // turn, long straight, right turn, short straight. Then a 180, then the
    // mirror-image leftward rectangle.
    drive_rectangle(&mut left, &mut right, Turn::Right).await;

    settle(&mut left, &mut right).await;
    spin(&mut left, &mut right, Turn::Right, 2 * TURN_90_MS).await; // 180 about-face
    settle(&mut left, &mut right).await;

    drive_rectangle(&mut left, &mut right, Turn::Left).await;

    // Done. Park with everything braked so the bot doesn't creep.
    left.stop();
    right.stop();
    info!("routine complete");

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

#[derive(Clone, Copy)]
enum Turn {
    Left,
    Right,
}

/// Drive one rectangle: long leg, corner, short leg, corner, long leg, corner,
/// short leg. All corners turn the same way (`turn`), so the bot traces a
/// closed-ish loop and ends roughly where it started, facing the start heading.
async fn drive_rectangle(left: &mut Motor<'_>, right: &mut Motor<'_>, turn: Turn) {
    for &leg_ms in &[LONG_LEG_MS, SHORT_LEG_MS, LONG_LEG_MS, SHORT_LEG_MS] {
        straight(left, right, leg_ms).await;
        settle(left, right).await;
        spin(left, right, turn, TURN_90_MS).await;
        settle(left, right).await;
    }
}

/// Both wheels forward at DRIVE_DUTY for `ms`.
async fn straight(left: &mut Motor<'_>, right: &mut Motor<'_>, ms: u64) {
    info!("straight {}ms", ms);
    left.forward(DRIVE_DUTY);
    right.forward(DRIVE_DUTY);
    Timer::after(Duration::from_millis(ms)).await;
    left.stop();
    right.stop();
}

/// In-place spin. Differential drive taken to the extreme: one wheel forward,
/// the other reverse, so the bot rotates about its center. Right turn = left
/// wheel forward, right wheel back.
async fn spin(left: &mut Motor<'_>, right: &mut Motor<'_>, turn: Turn, ms: u64) {
    match turn {
        Turn::Right => {
            info!("spin right {}ms", ms);
            left.forward(TURN_DUTY);
            right.reverse(TURN_DUTY);
        }
        Turn::Left => {
            info!("spin left {}ms", ms);
            left.reverse(TURN_DUTY);
            right.forward(TURN_DUTY);
        }
    }
    Timer::after(Duration::from_millis(ms)).await;
    left.stop();
    right.stop();
}

/// Brake both wheels and pause so momentum dies between moves.
async fn settle(left: &mut Motor<'_>, right: &mut Motor<'_>) {
    left.stop();
    right.stop();
    Timer::after(Duration::from_millis(SETTLE_MS)).await;
}
