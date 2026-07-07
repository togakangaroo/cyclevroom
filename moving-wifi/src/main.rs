use std::thread::sleep;
use std::time::Duration;

use esp_idf_svc::hal::gpio::{Output, PinDriver};
use esp_idf_svc::hal::ledc::config::TimerConfig;
use esp_idf_svc::hal::ledc::{LedcDriver, LedcTimerDriver, Resolution};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::units::FromValueType;

// --- Tunables ---------------------------------------------------------------
//
// Wiring-debug firmware: cycle each wheel through forward and reverse, one at
// a time, forever. Sit next to it with the wiring exposed and watch which
// phases actually produce motion.

/// Duty cycle while a wheel runs. Same reasoning as the no_std version: below
/// ~30% the motors may stall on stiction; 50% is unmissable without being wild.
const DUTY_PCT: u32 = 50;

/// How long each wheel runs in each direction.
const RUN_MS: u64 = 1500;

/// Braked pause between phases, so each phase reads as a distinct event.
const GAP_MS: u64 = 700;

/// PWM frequency on the L298N enable pins (same as the no_std firmware).
const PWM_FREQ_HZ: u32 = 5000;
// ----------------------------------------------------------------------------

/// One DC motor behind an L298N channel, esp-idf-hal flavor: a LEDC PWM
/// channel on the enable pin plus two direction outputs. Same truth table as
/// before: (in1,in2) = (H,L)/(L,H) picks direction, (L,L) brakes.
struct Motor<'d> {
    pwm: LedcDriver<'d>,
    in1: PinDriver<'d, Output>,
    in2: PinDriver<'d, Output>,
}

impl Motor<'_> {
    fn forward(&mut self, duty_pct: u32) {
        self.in1.set_high().unwrap();
        self.in2.set_low().unwrap();
        self.set_duty_pct(duty_pct);
    }

    fn reverse(&mut self, duty_pct: u32) {
        self.in1.set_low().unwrap();
        self.in2.set_high().unwrap();
        self.set_duty_pct(duty_pct);
    }

    /// Brake: direction lines low, enable off.
    fn stop(&mut self) {
        self.in1.set_low().unwrap();
        self.in2.set_low().unwrap();
        self.pwm.set_duty(0).unwrap();
    }

    fn set_duty_pct(&mut self, pct: u32) {
        // LedcDriver duty is in timer-resolution ticks, not percent.
        let max = self.pwm.get_max_duty();
        self.pwm.set_duty(max * pct / 100).unwrap();
    }
}

fn main() {
    // It is necessary to call this function once. Otherwise, some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    let pins = peripherals.pins;

    // One shared LEDC timer sets PWM frequency/resolution; each enable pin
    // gets its own channel so the wheels could run at independent duties.
    let pwm_timer = LedcTimerDriver::new(
        peripherals.ledc.timer0,
        &TimerConfig::new()
            .frequency(PWM_FREQ_HZ.Hz())
            .resolution(Resolution::Bits8),
    )
    .unwrap();

    // Pin map is identical to the no_std `moving` firmware.
    //
    // --- Motor A (LEFT wheel): ENA=GPIO0, IN1=GPIO1, IN2=GPIO10 -------------
    let mut left = Motor {
        pwm: LedcDriver::new(peripherals.ledc.channel0, &pwm_timer, pins.gpio0).unwrap(),
        in1: PinDriver::output(pins.gpio1).unwrap(),
        in2: PinDriver::output(pins.gpio10).unwrap(),
    };

    // --- Motor B (RIGHT wheel): ENB=GPIO6, IN3=GPIO7, IN4=GPIO5 -------------
    let mut right = Motor {
        pwm: LedcDriver::new(peripherals.ledc.channel1, &pwm_timer, pins.gpio6).unwrap(),
        in1: PinDriver::output(pins.gpio7).unwrap(),
        in2: PinDriver::output(pins.gpio5).unwrap(),
    };

    // Indicator LED on GPIO3 (same pin the earlier LED steps used): HIGH
    // whenever the RIGHT wheel is commanded to turn. Visual confirmation that
    // the firmware *thinks* the right wheel is running, independent of whether
    // the wheel actually moves -- separates code problems from wiring problems.
    let mut right_led = PinDriver::output(pins.gpio3).unwrap();

    log::info!("wheel cycle diagnostic: L-fwd, L-rev, R-fwd, R-rev, repeat");

    loop {
        log::info!("LEFT forward");
        left.forward(DUTY_PCT);
        sleep(Duration::from_millis(RUN_MS));
        left.stop();
        sleep(Duration::from_millis(GAP_MS));

        log::info!("LEFT reverse");
        left.reverse(DUTY_PCT);
        sleep(Duration::from_millis(RUN_MS));
        left.stop();
        sleep(Duration::from_millis(GAP_MS));

        log::info!("RIGHT forward");
        right_led.set_high().unwrap();
        right.forward(DUTY_PCT);
        sleep(Duration::from_millis(RUN_MS));
        right.stop();
        right_led.set_low().unwrap();
        sleep(Duration::from_millis(GAP_MS));

        log::info!("RIGHT reverse");
        right_led.set_high().unwrap();
        right.reverse(DUTY_PCT);
        sleep(Duration::from_millis(RUN_MS));
        right.stop();
        right_led.set_low().unwrap();
        sleep(Duration::from_millis(GAP_MS));
    }
}
