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
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::timer::timg::TimerGroup;
use esp_println as _;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.3.0
    // generator parameters: --chip esp32c3 -o unstable-hal -o embassy -o defmt -o esp-backtrace

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    let _ = spawner;

    // LED on GPIO3: GPIO3 -> 330R -> LED anode, cathode -> GND.
    let mut led = Output::new(peripherals.GPIO3, Level::Low, OutputConfig::default());

    // Mic D0 (digital output) on GPIO4. DIAGNOSTIC MODE: rather than assume the
    // edge polarity, poll the raw level fast, log every transition, and mirror
    // the level straight onto the LED. This tells us the idle level and whether
    // clapping moves the line at all -- which decides polarity and threshold.
    // No pull resistor here so we read what the module actually drives; if the
    // line floats (random transitions with nothing happening) the module isn't
    // wired/powered. Once we know the behaviour we'll go back to wait_for_*_edge.
    let mic_cfg = InputConfig::default().with_pull(Pull::None);
    let mic = Input::new(peripherals.GPIO4, mic_cfg);

    let mut last = mic.is_high();
    info!("Mic idle level: {}", last);
    led.set_level(if last { Level::High } else { Level::Low });

    loop {
        let now = mic.is_high();
        if now != last {
            info!("Mic D0 changed -> {}", now);
            led.set_level(if now { Level::High } else { Level::Low });
            last = now;
        }
        // Poll fast enough to catch a clap's comparator pulse (often <1ms).
        Timer::after(Duration::from_micros(200)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.1.0/examples
}
