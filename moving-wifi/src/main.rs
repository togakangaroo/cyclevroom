use std::io::{BufRead, BufReader, Write};
use std::net::TcpListener;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Mutex;
use std::thread::sleep;
use std::time::Duration;

use embedded_svc::wifi::{AuthMethod, ClientConfiguration, Configuration};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::gpio::{Output, PinDriver};
use esp_idf_svc::hal::ledc::config::TimerConfig;
use esp_idf_svc::hal::ledc::{LedcDriver, LedcTimerDriver, Resolution};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::units::FromValueType;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{BlockingWifi, EspWifi};

// --- WiFi credentials ---------------------------------------------------
//
// Baked in at compile time from the build environment, never committed --
// see the "Stage 1: WiFi command server" section of the README for how these
// get set (org-babel prompts for the password, passes both via env into the
// cargo invocation).
const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASS: &str = env!("WIFI_PASS");

/// Command server port. Anything unprivileged and unused works.
const COMMAND_PORT: u16 = 7878;

// --- Tunables ----------------------------------------------------------
//
// Same dead-reckoned-by-time approach as the earlier `moving` firmware, but
// now live-tunable over the `set` command instead of baked in at compile
// time -- these are the starting values.

static DRIVE_DUTY: AtomicU32 = AtomicU32::new(40);
static TURN_DUTY: AtomicU32 = AtomicU32::new(50);
static LONG_LEG_MS: AtomicU32 = AtomicU32::new(1000);
static SHORT_LEG_MS: AtomicU32 = AtomicU32::new(500);
static TURN_90_MS: AtomicU32 = AtomicU32::new(600);
static SETTLE_MS: AtomicU32 = AtomicU32::new(300);

/// PWM frequency on the L298N enable pins.
const PWM_FREQ_HZ: u32 = 5000;
// -------------------------------------------------------------------------

/// One DC motor behind an L298N channel: a PWM'd enable line (speed) plus two
/// direction lines. (in1,in2) = (H,L)/(L,H) picks direction, (L,L) brakes.
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
        let max = self.pwm.get_max_duty();
        self.pwm.set_duty(max * pct / 100).unwrap();
    }
}

#[derive(Clone, Copy)]
enum Turn {
    Left,
    Right,
}

/// Both wheels; owns the GPIO3 indicator LED that lights whenever the right
/// wheel is commanded to turn.
struct Rig<'d> {
    left: Motor<'d>,
    right: Motor<'d>,
    right_led: PinDriver<'d, Output>,
}

impl Rig<'_> {
    fn stop(&mut self) {
        self.left.stop();
        self.right.stop();
        self.right_led.set_low().unwrap();
    }

    fn left_drive(&mut self, dir: i32, duty_pct: u32) {
        match dir {
            d if d > 0 => self.left.forward(duty_pct),
            d if d < 0 => self.left.reverse(duty_pct),
            _ => self.left.stop(),
        }
    }

    fn right_drive(&mut self, dir: i32, duty_pct: u32) {
        match dir {
            d if d > 0 => {
                self.right_led.set_high().unwrap();
                self.right.forward(duty_pct);
            }
            d if d < 0 => {
                self.right_led.set_high().unwrap();
                self.right.reverse(duty_pct);
            }
            _ => {
                self.right.stop();
                self.right_led.set_low().unwrap();
            }
        }
    }

    /// Both wheels forward at DRIVE_DUTY for `ms`.
    fn straight(&mut self, ms: u32) {
        let duty = DRIVE_DUTY.load(Ordering::Relaxed);
        self.left_drive(1, duty);
        self.right_drive(1, duty);
        sleep(Duration::from_millis(ms as u64));
        self.stop();
    }

    /// In-place spin: one wheel forward, the other reverse.
    fn spin(&mut self, turn: Turn, ms: u32) {
        let duty = TURN_DUTY.load(Ordering::Relaxed);
        match turn {
            Turn::Right => {
                self.left_drive(1, duty);
                self.right_drive(-1, duty);
            }
            Turn::Left => {
                self.left_drive(-1, duty);
                self.right_drive(1, duty);
            }
        }
        sleep(Duration::from_millis(ms as u64));
        self.stop();
    }

    fn settle(&mut self) {
        self.stop();
        sleep(Duration::from_millis(
            SETTLE_MS.load(Ordering::Relaxed) as u64
        ));
    }

    /// One rectangle: long leg, corner, short leg, corner, long leg, corner,
    /// short leg -- all corners turning the same way.
    fn drive_rectangle(&mut self, turn: Turn) {
        for _ in 0..4 {
            for &leg in &[
                LONG_LEG_MS.load(Ordering::Relaxed),
                SHORT_LEG_MS.load(Ordering::Relaxed),
            ] {
                self.straight(leg);
                self.settle();
                self.spin(turn, TURN_90_MS.load(Ordering::Relaxed));
                self.settle();
            }
        }
    }

    fn drive_full_routine(&mut self) {
        self.drive_rectangle(Turn::Right);
        self.settle();
        self.spin(Turn::Right, 2 * TURN_90_MS.load(Ordering::Relaxed));
        self.settle();
        self.drive_rectangle(Turn::Left);
        self.stop();
    }
}

fn main() {
    // It is necessary to call this function once. Otherwise, some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    let pins = peripherals.pins;

    let pwm_timer = LedcTimerDriver::new(
        peripherals.ledc.timer0,
        &TimerConfig::new()
            .frequency(PWM_FREQ_HZ.Hz())
            .resolution(Resolution::Bits8),
    )
    .unwrap();

    // Pin map identical to earlier `moving`/`moving-wifi` firmwares.
    let left = Motor {
        pwm: LedcDriver::new(peripherals.ledc.channel0, &pwm_timer, pins.gpio0).unwrap(),
        in1: PinDriver::output(pins.gpio1).unwrap(),
        in2: PinDriver::output(pins.gpio10).unwrap(),
    };
    let right = Motor {
        pwm: LedcDriver::new(peripherals.ledc.channel1, &pwm_timer, pins.gpio6).unwrap(),
        in1: PinDriver::output(pins.gpio7).unwrap(),
        in2: PinDriver::output(pins.gpio5).unwrap(),
    };
    let right_led = PinDriver::output(pins.gpio3).unwrap();

    let rig = Mutex::new(Rig {
        left,
        right,
        right_led,
    });

    log::info!("connecting to wifi ssid={WIFI_SSID}");
    let sys_loop = EspSystemEventLoop::take().unwrap();
    let nvs = EspDefaultNvsPartition::take().unwrap();
    let mut wifi = BlockingWifi::wrap(
        EspWifi::new(peripherals.modem, sys_loop.clone(), Some(nvs)).unwrap(),
        sys_loop,
    )
    .unwrap();
    connect_wifi(&mut wifi).unwrap();

    let ip_info = wifi.wifi().sta_netif().get_ip_info().unwrap();
    log::info!("wifi up, ip={}", ip_info.ip);

    let listener = TcpListener::bind(("0.0.0.0", COMMAND_PORT)).unwrap();
    log::info!("command server listening on port {COMMAND_PORT}");

    // Keep wifi alive for the life of the program -- BlockingWifi must not be
    // dropped or the connection tears down.
    let _wifi = wifi;

    for stream in listener.incoming() {
        let stream = match stream {
            Ok(s) => s,
            Err(e) => {
                log::warn!("accept failed: {e}");
                continue;
            }
        };
        log::info!("client connected");
        handle_client(stream, &rig);
    }
}

fn connect_wifi(wifi: &mut BlockingWifi<EspWifi<'static>>) -> anyhow::Result<()> {
    let config = Configuration::Client(ClientConfiguration {
        ssid: WIFI_SSID.try_into().unwrap(),
        bssid: None,
        auth_method: AuthMethod::WPA2Personal,
        password: WIFI_PASS.try_into().unwrap(),
        channel: None,
        ..Default::default()
    });
    wifi.set_configuration(&config)?;
    wifi.start()?;
    wifi.connect()?;
    wifi.wait_netif_up()?;
    Ok(())
}

/// One client at a time, line-delimited commands, plain text replies. Netcat
/// friendly: `nc <ip> 7878`.
fn handle_client(stream: std::net::TcpStream, rig: &Mutex<Rig>) {
    // lwIP (ESP-IDF's socket layer) doesn't implement dup(), so
    // `TcpStream::try_clone` fails with ENOSYS -- read and write through the
    // one owned stream instead of splitting into cloned reader/writer halves.
    let mut reader = BufReader::new(stream);

    loop {
        let mut line = String::new();
        let n = match reader.read_line(&mut line) {
            Ok(n) => n,
            Err(_) => break,
        };
        if n == 0 {
            break; // EOF
        }
        let reply = handle_command(line.trim(), rig);
        if writeln!(reader.get_mut(), "{reply}").is_err() {
            break;
        }
    }

    // Client dropped -- brake so a lost connection never leaves the bot
    // driving blind.
    rig.lock().unwrap().stop();
    log::info!("client disconnected, motors stopped");
}

fn handle_command(line: &str, rig: &Mutex<Rig>) -> String {
    let mut parts = line.split_whitespace();
    let Some(cmd) = parts.next() else {
        return String::new();
    };

    match cmd {
        "left" | "right" => {
            let Some(dir_word) = parts.next() else {
                return "usage: left|right fwd|rev|stop [duty_pct]".into();
            };
            let dir = match dir_word {
                "fwd" => 1,
                "rev" => -1,
                "stop" => 0,
                _ => return "dir must be fwd, rev, or stop".into(),
            };
            let duty = parts
                .next()
                .and_then(|s| s.parse::<u32>().ok())
                .unwrap_or(DRIVE_DUTY.load(Ordering::Relaxed));

            let mut rig = rig.lock().unwrap();
            if cmd == "left" {
                rig.left_drive(dir, duty);
            } else {
                rig.right_drive(dir, duty);
            }
            format!("ok {cmd} {dir_word} {duty}")
        }

        "stop" => {
            rig.lock().unwrap().stop();
            "ok stop".into()
        }

        "alt" => {
            // Alternate wheels: L-fwd, L-rev, R-fwd, R-rev, one lap. Diagnostic
            // mode for poking wiring while watching which phase moves what.
            let ms = parts
                .next()
                .and_then(|s| s.parse::<u64>().ok())
                .unwrap_or(1000);
            let duty = DRIVE_DUTY.load(Ordering::Relaxed);
            let mut rig = rig.lock().unwrap();
            for (label, dir, is_right) in [
                ("left fwd", 1, false),
                ("left rev", -1, false),
                ("right fwd", 1, true),
                ("right rev", -1, true),
            ] {
                log::info!("alt: {label}");
                if is_right {
                    rig.right_drive(dir, duty);
                } else {
                    rig.left_drive(dir, duty);
                }
                sleep(Duration::from_millis(ms));
                rig.stop();
                sleep(Duration::from_millis(200));
            }
            "ok alt".into()
        }

        "rect" => {
            rig.lock().unwrap().drive_full_routine();
            "ok rect".into()
        }

        "set" => {
            let (Some(key), Some(val)) = (parts.next(), parts.next()) else {
                return "usage: set <key> <value>".into();
            };
            let Ok(val_u64) = val.parse::<u64>() else {
                return "value must be an integer".into();
            };
            match key {
                "drive_duty" => DRIVE_DUTY.store(val_u64 as u32, Ordering::Relaxed),
                "turn_duty" => TURN_DUTY.store(val_u64 as u32, Ordering::Relaxed),
                "long_leg_ms" => LONG_LEG_MS.store(val_u64 as u32, Ordering::Relaxed),
                "short_leg_ms" => SHORT_LEG_MS.store(val_u64 as u32, Ordering::Relaxed),
                "turn90_ms" => TURN_90_MS.store(val_u64 as u32, Ordering::Relaxed),
                "settle_ms" => SETTLE_MS.store(val_u64 as u32, Ordering::Relaxed),
                _ => return format!("unknown key '{key}'"),
            }
            format!("ok set {key} {val_u64}")
        }

        "get" => format!(
            "drive_duty={} turn_duty={} long_leg_ms={} short_leg_ms={} turn90_ms={} settle_ms={}",
            DRIVE_DUTY.load(Ordering::Relaxed),
            TURN_DUTY.load(Ordering::Relaxed),
            LONG_LEG_MS.load(Ordering::Relaxed),
            SHORT_LEG_MS.load(Ordering::Relaxed),
            TURN_90_MS.load(Ordering::Relaxed),
            SETTLE_MS.load(Ordering::Relaxed),
        ),

        "help" => "commands: left|right fwd|rev|stop [duty]; stop; alt [ms]; rect; \
                    set <key> <val>; get; help"
            .into(),

        _ => format!("unknown command '{cmd}' (try 'help')"),
    }
}
