//! cyclevroom: clap-seeking laser bot.
//!
//! Everything the earlier prototypes proved out, in one binary -- the two-mic
//! direction sensing from `firmware-left-or-right`, the differential drive and
//! wifi command server from `moving-wifi`, plus the laser -- wired together by
//! the state machine in `cyclevroom::machine`.
//!
//! Clap twice and it turns toward you, drives at you, waits, and if you stay
//! quiet it paints you with the laser.
//!
//! # Pin map (ESP32-C3-DevKitM-1)
//!
//! | Signal                | GPIO  | Note                                  |
//! |-----------------------|-------|---------------------------------------|
//! | Left speed  (ENA)     | 0     |                                       |
//! | Left dir 1  (IN1)     | 1     |                                       |
//! | Left dir 2  (IN2)     | 10    |                                       |
//! | Right dir 1 (IN3)     | 7     |                                       |
//! | Right dir 2 (IN4)     | 2     | moved off 5 to free it for the mic    |
//! | Right speed (ENB)     | 6     |                                       |
//! | Left mic    (D0)      | 4     | "mic1" -- the negative side of delta  |
//! | Right mic   (D0)      | 5     | "mic2" -- was the right motor's IN4   |
//! | Laser       (KY-008 S)| 3     | replaces the old indicator LED        |
//!
//! GPIO2 is an ESP32-C3 strapping pin, which is why the *motor* line lives
//! there and not the mic: it's driven as an output, so it can't hold the pin
//! low through reset the way an actively-driven KY-038 D0 would. Moving IN4
//! rather than the mic is one jumper on the L298N and needs no extra parts.
//!
//! The mics stay ~5cm apart as on the bench, so the timing constants carry
//! over from `firmware-left-or-right` unchanged.

use std::io::{BufRead, BufReader, Write};
use std::mem::ManuallyDrop;
use std::net::{TcpListener, TcpStream};
use std::os::fd::{AsRawFd, FromRawFd, RawFd};
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, AtomicU8, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::sleep;
use std::time::{Duration, Instant};

use embedded_svc::wifi::{AuthMethod, ClientConfiguration, Configuration};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::gpio::{Input, InterruptType, Output, PinDriver, Pull};
use esp_idf_svc::hal::ledc::config::TimerConfig;
use esp_idf_svc::hal::ledc::{LedcDriver, LedcTimerDriver, Resolution};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::units::FromValueType;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{BlockingWifi, EspWifi};

use cyclevroom::machine::{transition, Event, State, Tunables};
use cyclevroom::parse::{parse_command, Command, LaserCmd, SetKey, Wheel};
use cyclevroom::tdoa::{classify_delta, Side};

// --- WiFi credentials ---------------------------------------------------
//
// Baked in at compile time from the build environment, never committed --
// see the README for the org-babel block that prompts for the password and
// passes both through the environment of the cargo invocation.
const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASS: &str = env!("WIFI_PASS");

/// Command server port, same as moving-wifi. Netcat friendly.
const COMMAND_PORT: u16 = 7878;

/// PWM frequency on the L298N enable pins.
const PWM_FREQ_HZ: u32 = 5000;

// --- Tunables ----------------------------------------------------------
//
// AtomicU32 rather than const so `set` can retune them live over TCP without a
// reflash. (u32 not u64 -- the C3 has no AtomicU64. Milliseconds fit fine.)

static DRIVE_DUTY: AtomicU32 = AtomicU32::new(90);
/// 73% and 600ms turned ~90 degrees on the bench, so ~200ms lands near 30.
static TURN_DUTY: AtomicU32 = AtomicU32::new(73);
static TURN_MS: AtomicU32 = AtomicU32::new(200);
static APPROACH_MS: AtomicU32 = AtomicU32::new(1000);
static ARMED_MS: AtomicU32 = AtomicU32::new(2000);
static ACTIVE_IDLE_MS: AtomicU32 = AtomicU32::new(2000);
static FIRE_AMOUNT: AtomicU32 = AtomicU32::new(5);
static FIRE_ON_MS: AtomicU32 = AtomicU32::new(1000);
static FIRE_OFF_MS: AtomicU32 = AtomicU32::new(1000);
static PAUSED_MS: AtomicU32 = AtomicU32::new(10_000);
static SETTLE_MS: AtomicU32 = AtomicU32::new(300);

// Carried over from firmware-left-or-right, where they were tuned by clapping
// from known positions and watching the logged deltas.
static CENTER_DEADBAND_US: AtomicU32 = AtomicU32::new(60);
static BLANKING_MS: AtomicU32 = AtomicU32::new(200);
static PAIR_TIMEOUT_MS: AtomicU32 = AtomicU32::new(50);

// --- Shared state -------------------------------------------------------

/// The live state, as a plain integer so the server thread can answer `state`
/// without contending for the rig lock.
static CURRENT_STATE: AtomicU8 = AtomicU8::new(State::Listening as u8);

/// Set by `stop` (and by a dropped connection) to cut a drive or laser burst
/// short. Cleared when the machine leaves `Paused`.
static ABORT: AtomicBool = AtomicBool::new(false);

/// A pending operator `state <name>` override, as `State as u8`, or -1 for
/// none. `AtomicI32` so "none" is representable without a separate flag.
static FORCED_STATE: AtomicI32 = AtomicI32::new(-1);

/// Manual (wiring-debug) mode. Set by `left|right ...` and `alt`; holds the
/// machine in `Paused` with no timeout so the behaviour loop can't resume
/// and brake or steer the wheels out from under a hand-driven test. Cleared
/// by `state <name>` (anything but paused) or a dropped connection.
static MANUAL: AtomicBool = AtomicBool::new(false);

/// True once the `Paused` arm has done its entry `all_off()`. A manual drive
/// waits for this, otherwise that entry brake could land just after the
/// drive starts and silently cancel it. Cleared by every `enter()`.
static PAUSE_SETTLED: AtomicBool = AtomicBool::new(false);

fn snapshot_tunables() -> Tunables {
    Tunables {
        active_idle_ms: ACTIVE_IDLE_MS.load(Ordering::Relaxed),
        turn_ms: TURN_MS.load(Ordering::Relaxed),
        approach_ms: APPROACH_MS.load(Ordering::Relaxed),
        armed_ms: ARMED_MS.load(Ordering::Relaxed),
        fire_amount: FIRE_AMOUNT.load(Ordering::Relaxed),
        paused_ms: PAUSED_MS.load(Ordering::Relaxed),
    }
}

fn current_state() -> State {
    State::from_u8(CURRENT_STATE.load(Ordering::Relaxed)).unwrap_or(State::Listening)
}

// --- Clap detection -----------------------------------------------------
//
// The two mic D0 lines are read as GPIO interrupts rather than polled: the ISR
// does nothing but stamp the microsecond clock and bump a sequence counter, so
// the arrival timestamps keep the ~1-2us jitter the direction math assumes. A
// polling loop in a FreeRTOS thread would blur that by milliseconds.

/// Microsecond timestamp of each mic's most recent rising edge, plus a counter
/// that increments on every edge so the consumer can tell "new edge" from
/// "same edge read twice" without clearing anything from the ISR side.
struct MicEdge {
    at_us: AtomicU32,
    seq: AtomicU32,
}

impl MicEdge {
    const fn new() -> Self {
        Self {
            at_us: AtomicU32::new(0),
            seq: AtomicU32::new(0),
        }
    }
}

static LEFT_EDGE: MicEdge = MicEdge::new();
static RIGHT_EDGE: MicEdge = MicEdge::new();

/// The two mic pin drivers, published so the polling loops can re-arm the
/// interrupts.
///
/// `esp-idf-hal` disables a GPIO interrupt the moment it fires -- deliberately,
/// so a level-triggered pin can't livelock the ISR -- and re-arming is only
/// legal from non-ISR context. So the ISR closure cannot do it; whoever
/// *observes* the edge has to. Without this the mics fire exactly once each
/// after boot and the bot is deaf forever after.
static MIC_PINS: Mutex<Option<MicPins>> = Mutex::new(None);

struct MicPins {
    left: &'static mut PinDriver<'static, Input>,
    right: &'static mut PinDriver<'static, Input>,
}

// Safe for the same reason the `Box::leak` above is: these live for the whole
// program and are only ever touched under the mutex, from normal threads.
unsafe impl Send for MicPins {}

/// Re-arm both mic interrupts. Idempotent, and cheap enough to call on every
/// poll pass rather than tracking which side actually fired.
fn rearm_mics() {
    if let Ok(mut guard) = MIC_PINS.lock() {
        if let Some(pins) = guard.as_mut() {
            let _ = pins.left.enable_interrupt();
            let _ = pins.right.enable_interrupt();
        }
    }
}

/// Free-running microsecond clock. Wraps every ~71 minutes at u32; deltas are
/// computed with `wrapping_sub` so a wrap between two edges of the same clap is
/// still correct.
fn now_us() -> u32 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() as u32 }
}

/// Snapshot of both mics' edge counters, used to spot new claps.
#[derive(Clone, Copy, PartialEq, Eq)]
struct EdgeSeq {
    left: u32,
    right: u32,
}

fn edge_seq() -> EdgeSeq {
    EdgeSeq {
        left: LEFT_EDGE.seq.load(Ordering::Acquire),
        right: RIGHT_EDGE.seq.load(Ordering::Acquire),
    }
}

/// Forget any edges recorded so far.
///
/// Called on entry to every listening state. This is how "don't listen while
/// the wheels are turning" is enforced: rather than disabling the interrupts
/// (and risking a missed edge at the boundary), we let them keep firing and
/// simply discard everything that accumulated while we were deaf. Without it,
/// motor brush noise picked up mid-turn would re-trigger the instant we
/// started listening again.
fn drain_edges() -> EdgeSeq {
    // Entry to a listening state is exactly when a one-shot interrupt left
    // disabled by the previous clap needs to come back.
    rearm_mics();
    edge_seq()
}

/// Wait for the first mic edge after `since`, giving up after `timeout`.
/// Returns which mic fired and when, in microseconds.
///
/// Polls the ISR-published counters at 1ms. The *timestamps* come from the
/// ISR, so this poll interval costs us nothing in timing precision -- it only
/// bounds how fast we notice.
fn wait_for_edge(since: EdgeSeq, timeout: Duration) -> Option<(Side, u32, EdgeSeq)> {
    let deadline = Instant::now() + timeout;
    loop {
        let seq = edge_seq();
        // If both fired between polls, the earlier timestamp is the near mic.
        let left_new = seq.left != since.left;
        let right_new = seq.right != since.right;
        match (left_new, right_new) {
            (true, true) => {
                let l = LEFT_EDGE.at_us.load(Ordering::Acquire);
                let r = RIGHT_EDGE.at_us.load(Ordering::Acquire);
                let near = if l.wrapping_sub(r) as i32 <= 0 {
                    (Side::Left, l)
                } else {
                    (Side::Right, r)
                };
                return Some((near.0, near.1, seq));
            }
            (true, false) => {
                return Some((Side::Left, LEFT_EDGE.at_us.load(Ordering::Acquire), seq))
            }
            (false, true) => {
                return Some((Side::Right, RIGHT_EDGE.at_us.load(Ordering::Acquire), seq))
            }
            (false, false) => {}
        }
        if Instant::now() >= deadline || ABORT.load(Ordering::Relaxed) {
            return None;
        }
        // Re-arm while waiting: a spurious edge before the clap we want would
        // otherwise have left that mic disabled for the rest of the window.
        rearm_mics();
        sleep(Duration::from_millis(1));
    }
}

/// Wait for the *other* mic after `near` has already fired, so we can measure
/// the gap. Bounded by `PAIR_TIMEOUT_MS` -- a lone trigger must not hang us.
fn wait_for_pair(near: Side, since: EdgeSeq) -> Option<u32> {
    let timeout = Duration::from_millis(PAIR_TIMEOUT_MS.load(Ordering::Relaxed) as u64);
    let deadline = Instant::now() + timeout;
    // The near mic's edge disabled *its* interrupt; the far mic may also have
    // been left disabled by an earlier edge. Re-arm before waiting, or a clap
    // is permanently seen as single-mic and the delta is never measured.
    rearm_mics();
    loop {
        let seq = edge_seq();
        let fired = match near {
            Side::Left => (seq.right != since.right).then(|| RIGHT_EDGE.at_us.load(Ordering::Acquire)),
            // Center never comes back from wait_for_edge, but treat it as right
            // for exhaustiveness.
            Side::Right | Side::Center => {
                (seq.left != since.left).then(|| LEFT_EDGE.at_us.load(Ordering::Acquire))
            }
        };
        if let Some(t) = fired {
            return Some(t);
        }
        if Instant::now() >= deadline || ABORT.load(Ordering::Relaxed) {
            return None;
        }
        sleep(Duration::from_millis(1));
    }
}

/// One clap, resolved to a direction and the signed delta that produced it.
struct Clap {
    side: Side,
    /// `mic1 - mic2`, i.e. left minus right, in microseconds. `None` when only
    /// one mic ever fired, so there was no pair to difference.
    delta_us: Option<i64>,
}

/// Wait for a full clap: first edge, then (bounded) the second, then classify.
fn listen_for_clap(since: EdgeSeq, timeout: Duration) -> Option<Clap> {
    let (near, t_first, seq) = wait_for_edge(since, timeout)?;
    match wait_for_pair(near, seq) {
        Some(t_second) => {
            // Order the pair back into (left, right) before subtracting, so the
            // sign convention holds regardless of which fired first.
            let (l, r) = match near {
                Side::Left => (t_first, t_second),
                _ => (t_second, t_first),
            };
            let delta_us = (l.wrapping_sub(r) as i32) as i64;
            let side = classify_delta(delta_us, CENTER_DEADBAND_US.load(Ordering::Relaxed));
            Some(Clap {
                side,
                delta_us: Some(delta_us),
            })
        }
        // Only one mic heard it -- report that side, it's all the evidence we
        // have. No delta, so nothing to print for tuning.
        None => Some(Clap {
            side: near,
            delta_us: None,
        }),
    }
}

// --- Hardware -----------------------------------------------------------

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
        self.pwm.set_duty(max * pct.min(100) / 100).unwrap();
    }
}

/// Both wheels plus the laser. Everything that physically actuates lives here,
/// behind one mutex, so the state thread and the command server can't drive
/// conflicting things at once.
struct Rig<'d> {
    left: Motor<'d>,
    right: Motor<'d>,
    laser: PinDriver<'d, Output>,
}

impl Rig<'_> {
    fn stop(&mut self) {
        self.left.stop();
        self.right.stop();
    }

    /// Brake and go dark -- the all-off used by `stop` and on disconnect.
    fn all_off(&mut self) {
        self.stop();
        self.laser_off();
    }

    fn laser_on(&mut self) {
        self.laser.set_high().unwrap();
    }

    fn laser_off(&mut self) {
        self.laser.set_low().unwrap();
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
            d if d > 0 => self.right.forward(duty_pct),
            d if d < 0 => self.right.reverse(duty_pct),
            _ => self.right.stop(),
        }
    }

    /// Start both wheels forward. Non-blocking -- the state loop decides when
    /// to stop, so it stays responsive to `stop` while driving.
    fn start_straight(&mut self) {
        let duty = DRIVE_DUTY.load(Ordering::Relaxed);
        self.left_drive(1, duty);
        self.right_drive(1, duty);
    }

    /// Start an in-place spin: one wheel forward, the other reverse. Also
    /// non-blocking. `Side::Center` means "dead ahead" -- don't turn at all.
    fn start_spin(&mut self, side: Side) {
        let duty = TURN_DUTY.load(Ordering::Relaxed);
        match side {
            Side::Right => {
                self.left_drive(1, duty);
                self.right_drive(-1, duty);
            }
            Side::Left => {
                self.left_drive(-1, duty);
                self.right_drive(1, duty);
            }
            Side::Center => self.stop(),
        }
    }
}

// --- Wiring the state machine to the hardware ---------------------------

/// Sleep in short slices, bailing early on `stop`.
fn abortable_sleep(ms: u64) {
    const SLICE_MS: u64 = 20;
    let mut remaining = ms;
    while remaining > 0 && !ABORT.load(Ordering::Relaxed) {
        let step = remaining.min(SLICE_MS);
        sleep(Duration::from_millis(step));
        remaining -= step;
    }
}

/// The connected client's socket fd, shared so the state thread can push
/// unsolicited lines (state changes, the tuning delta) to whoever is connected.
///
/// A raw fd rather than a second `TcpStream`, because `TcpStream::try_clone()`
/// is `dup()`-backed and lwIP doesn't implement it (the ENOSYS gotcha from
/// Stage 1). Writing to the same fd from two threads is fine here: the reader
/// thread only ever writes short command replies, this only ever writes short
/// notification lines, and a line is a single `write` syscall well under the
/// socket buffer -- so they interleave between lines, never within one.
///
/// `None` when nobody is connected, which makes [`notify`] a silent no-op.
type Client = Arc<Mutex<Option<RawFd>>>;

/// Write a line to the connected client, if any. Best effort -- a dead socket
/// is noticed by the reader thread, so failures here are ignored.
///
/// The fd is borrowed, never owned: `ManuallyDrop` keeps the `TcpStream`
/// wrapper from closing the socket out from under the reader thread when it
/// goes out of scope at the end of this function.
fn notify(client: &Client, msg: &str) {
    let guard = client.lock().unwrap();
    let Some(fd) = *guard else {
        return;
    };
    // Safety: `fd` is owned by the live `TcpStream` in `handle_client`, which
    // clears this slot before dropping it -- so while we hold the lock and see
    // `Some`, the fd is open. ManuallyDrop prevents a double close.
    let mut borrowed = ManuallyDrop::new(unsafe { TcpStream::from_raw_fd(fd) });
    let _ = writeln!(borrowed, "{msg}");
}

/// Move to `next`, logging and telling the client.
fn enter(next: State, client: &Client) {
    PAUSE_SETTLED.store(false, Ordering::Release);
    CURRENT_STATE.store(next as u8, Ordering::Relaxed);
    log::info!("-> {}", next.name());
    notify(client, &format!("state {}", next.name()));
}

/// The behaviour loop. Owns the rig; runs forever on its own thread.
///
/// Each state does its own work, then asks `transition` what's next -- so the
/// table in `cyclevroom::machine` stays the single source of truth for the
/// flow, and this function only knows how to *perform* each state.
fn run_state_machine(rig: Arc<Mutex<Rig<'static>>>, client: Client) -> ! {
    // The direction the last resolved clap pointed at, consumed by `Turning`.
    // The only thing that genuinely carries across states -- the edge counters
    // are re-read fresh on entry to each listening state (see `drain_edges`).
    let mut heading = Side::Center;

    // When the machine last *changed* state. `Listening` and `Armed` re-enter
    // their arm repeatedly (they poll in slices so a `stop` lands promptly), so
    // timing a deadline from the top of the loop would restart the clock on
    // every pass and `Armed` would never time out. Track real entry instead.
    let mut state_since = Instant::now();
    let mut last_state = current_state();
    // True on the first pass through a newly-entered state, so one-shot entry
    // work (braking, draining stale mic edges) doesn't repeat on every poll.
    let mut fresh = true;

    loop {
        let state = current_state();
        if state != last_state {
            last_state = state;
            state_since = Instant::now();
            fresh = true;
        }
        let just_entered = std::mem::replace(&mut fresh, false);
        let t = snapshot_tunables();
        let entered = state_since;

        // An operator `state <name>` beats whatever we were about to do.
        macro_rules! check_forced {
            () => {
                if let Some(target) = take_forced() {
                    if let Some(next) = transition(current_state(), Event::Force(target), &t) {
                        rig.lock().unwrap().all_off();
                        enter(next, &client);
                        continue;
                    }
                }
            };
        }

        match state {
            // Waiting for the first clap. Nothing is moving.
            State::Listening | State::Armed => {
                // Brake once on entry, not on every 250ms poll.
                if just_entered {
                    rig.lock().unwrap().stop();
                }
                // Re-baseline the counters immediately before each wait. On
                // entry that discards everything heard while we were deaf or
                // driving; a clap landing mid-slice bumps the counter and is
                // picked up by the wait that's about to start.
                let seq = drain_edges();

                // Both poll in short slices and re-enter, so a `stop` or a
                // forced state is acted on within ~250ms rather than after the
                // whole wait. `Armed`'s real deadline is measured from
                // `state_since` in the timeout arm below; `Listening` has none.
                match listen_for_clap(seq, Duration::from_millis(250)) {
                    Some(clap) => {
                        heading = clap.side;
                        if let Some(next) = transition(state, Event::Clap, &t) {
                            enter(next, &client);
                        }
                    }
                    None => {
                        check_forced!();
                        if ABORT.load(Ordering::Relaxed) {
                            continue;
                        }
                        // Only Armed has a deadline worth reporting.
                        let waited = entered.elapsed().as_millis() as u32;
                        if let Some(next) = transition(
                            state,
                            Event::Tick {
                                in_state_ms: waited,
                            },
                            &t,
                        ) {
                            enter(next, &client);
                        }
                    }
                }
            }

            // Heard one clap; the next one fixes the direction.
            State::Active => {
                if just_entered {
                    rig.lock().unwrap().stop();
                }
                // Sliced like Listening/Armed; the real ACTIVE_IDLE_MS
                // deadline is applied against `state_since` in the None arm.
                let seq = drain_edges();

                match listen_for_clap(seq, Duration::from_millis(250)) {
                    Some(clap) => {
                        heading = clap.side;

                        // The tuning readout asked for: mic1 - mic2, so
                        // negative means the left mic heard it first.
                        match clap.delta_us {
                            Some(d) => notify(
                                &client,
                                &format!("delta_us={d} dir={}", side_name(clap.side)),
                            ),
                            None => notify(
                                &client,
                                &format!("delta_us=none dir={} (one mic only)", side_name(clap.side)),
                            ),
                        }

                        if let Some(next) = transition(state, Event::Clap, &t) {
                            enter(next, &client);
                        }
                    }
                    None => {
                        check_forced!();
                        if ABORT.load(Ordering::Relaxed) {
                            continue;
                        }
                        let waited = entered.elapsed().as_millis() as u32;
                        if let Some(next) = transition(
                            state,
                            Event::Tick {
                                in_state_ms: waited,
                            },
                            &t,
                        ) {
                            enter(next, &client);
                        }
                    }
                }
            }

            // Deaf states: spin, then drive. Claps here are ignored by
            // construction -- we never call listen_for_clap.
            State::Turning => {
                rig.lock().unwrap().start_spin(heading);
                abortable_sleep(t.turn_ms as u64);
                {
                    let mut rig = rig.lock().unwrap();
                    rig.stop();
                }
                abortable_sleep(SETTLE_MS.load(Ordering::Relaxed) as u64);
                check_forced!();
                if let Some(next) = transition(
                    state,
                    Event::Tick {
                        in_state_ms: t.turn_ms,
                    },
                    &t,
                ) {
                    enter(next, &client);
                }
            }

            State::Approaching => {
                rig.lock().unwrap().start_straight();
                abortable_sleep(t.approach_ms as u64);
                {
                    let mut rig = rig.lock().unwrap();
                    rig.stop();
                }
                abortable_sleep(SETTLE_MS.load(Ordering::Relaxed) as u64);
                check_forced!();
                if let Some(next) = transition(
                    state,
                    Event::Tick {
                        in_state_ms: t.approach_ms,
                    },
                    &t,
                ) {
                    enter(next, &client);
                }
            }

            // Pulse the laser, watching for a clap between pulses.
            State::Firing => {
                let on_ms = FIRE_ON_MS.load(Ordering::Relaxed) as u64;
                let off_ms = FIRE_OFF_MS.load(Ordering::Relaxed) as u64;
                let mut interrupted = false;

                for i in 0..t.fire_amount {
                    if ABORT.load(Ordering::Relaxed) {
                        break;
                    }
                    log::info!("fire {}/{}", i + 1, t.fire_amount);
                    rig.lock().unwrap().laser_on();
                    // Listening *through* the pulse, so a clap cuts the burst
                    // short rather than waiting for the gap between pulses.
                    // Re-baseline before each wait -- a stale `seq` would make
                    // an old edge look like a fresh clap on the next pulse.
                    if let Some(clap) = listen_for_clap(drain_edges(), Duration::from_millis(on_ms))
                    {
                        heading = clap.side;
                        interrupted = true;
                    }
                    rig.lock().unwrap().laser_off();
                    if interrupted {
                        break;
                    }
                    if let Some(clap) =
                        listen_for_clap(drain_edges(), Duration::from_millis(off_ms))
                    {
                        heading = clap.side;
                        interrupted = true;
                        break;
                    }
                }

                rig.lock().unwrap().laser_off();
                check_forced!();

                let event = if interrupted {
                    Event::Clap
                } else {
                    Event::FireComplete
                };
                if let Some(next) = transition(state, event, &t) {
                    enter(next, &client);
                }
            }

            // Halted. Sit out the timer, then resume.
            State::Paused => {
                rig.lock().unwrap().all_off();
                PAUSE_SETTLED.store(true, Ordering::Release);
                let deadline = Instant::now() + Duration::from_millis(t.paused_ms as u64);
                // In manual mode the pause never times out on its own.
                while MANUAL.load(Ordering::Acquire) || Instant::now() < deadline {
                    if let Some(target) = take_forced() {
                        if let Some(next) = transition(current_state(), Event::Force(target), &t) {
                            enter(next, &client);
                            break;
                        }
                    }
                    sleep(Duration::from_millis(50));
                }
                if current_state() == State::Paused {
                    // Pause is over: clear the abort latch so driving works
                    // again. Edges heard while halted are dropped by the
                    // drain on entry to the next listening state.
                    ABORT.store(false, Ordering::Relaxed);
                    if let Some(next) = transition(
                        state,
                        Event::Tick {
                            in_state_ms: t.paused_ms,
                        },
                        &t,
                    ) {
                        enter(next, &client);
                    }
                }
            }
        }
    }
}

/// Switch to manual mode: park the behaviour loop in `Paused` (indefinitely,
/// see `MANUAL`) and wait until it has finished its entry brake, so a manual
/// drive issued right after can't be cancelled by it. Returns false if the
/// loop didn't settle in time (e.g. stuck mid-laser-pulse); MANUAL stays set,
/// so a retry a moment later will find it paused.
fn enter_manual() -> bool {
    MANUAL.store(true, Ordering::Release);
    let settled = || current_state() == State::Paused && PAUSE_SETTLED.load(Ordering::Acquire);
    if settled() {
        return true;
    }
    ABORT.store(true, Ordering::Relaxed);
    FORCED_STATE.store(State::Paused as u8 as i32, Ordering::Release);
    let deadline = Instant::now() + Duration::from_millis(3000);
    while Instant::now() < deadline {
        if settled() {
            return true;
        }
        sleep(Duration::from_millis(10));
    }
    false
}

/// Take a pending `state <name>` override, if one is queued.
fn take_forced() -> Option<State> {
    let v = FORCED_STATE.swap(-1, Ordering::AcqRel);
    if v < 0 {
        None
    } else {
        State::from_u8(v as u8)
    }
}

fn side_name(s: Side) -> &'static str {
    match s {
        Side::Left => "left",
        Side::Right => "right",
        Side::Center => "center",
    }
}

// --- main ---------------------------------------------------------------

fn main() {
    // Necessary once, or some esp-idf-sys runtime patches don't link.
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    let pins = peripherals.pins;

    // Leaked to 'static so `Rig` carries no borrow tied to this stack frame --
    // fine, it must live for the life of the program regardless.
    let pwm_timer: &'static _ = Box::leak(Box::new(
        LedcTimerDriver::new(
            peripherals.ledc.timer0,
            &TimerConfig::new()
                .frequency(PWM_FREQ_HZ.Hz())
                .resolution(Resolution::Bits8),
        )
        .unwrap(),
    ));

    let left = Motor {
        pwm: LedcDriver::new(peripherals.ledc.channel0, pwm_timer, pins.gpio0).unwrap(),
        in1: PinDriver::output(pins.gpio1).unwrap(),
        in2: PinDriver::output(pins.gpio10).unwrap(),
    };
    let right = Motor {
        pwm: LedcDriver::new(peripherals.ledc.channel1, pwm_timer, pins.gpio6).unwrap(),
        in1: PinDriver::output(pins.gpio7).unwrap(),
        // GPIO2 (strapping pin) is safe here precisely because it's an output.
        in2: PinDriver::output(pins.gpio2).unwrap(),
    };
    let laser = PinDriver::output(pins.gpio3).unwrap();

    let rig: Arc<Mutex<Rig<'static>>> = Arc::new(Mutex::new(Rig { left, right, laser }));

    // Mic interrupts. The LM393 comparator drives D0 actively, so no pull, and
    // D0 goes HIGH on a clap -- rising edge. (Same as firmware-left-or-right,
    // which worked on these same pins with the same modules.)
    //
    // Leaked for the same reason as the PWM timer: an ISR subscription must
    // outlive this frame, and `subscribe` is unsafe because the closure runs in
    // interrupt context. Ours only touches atomics and the esp_timer clock --
    // no allocation, no locks, no logging -- which is what makes that sound.
    let left_mic: &'static mut PinDriver<'static, Input> =
        Box::leak(Box::new(PinDriver::input(pins.gpio4, Pull::Floating).unwrap()));
    let right_mic: &'static mut PinDriver<'static, Input> =
        Box::leak(Box::new(PinDriver::input(pins.gpio5, Pull::Floating).unwrap()));

    for (mic, edge) in [
        (&mut *left_mic, &LEFT_EDGE),
        (&mut *right_mic, &RIGHT_EDGE),
    ] {
        mic.set_interrupt_type(InterruptType::PosEdge).unwrap();
        unsafe {
            mic.subscribe(move || {
                edge.at_us.store(now_us(), Ordering::Release);
                edge.seq.fetch_add(1, Ordering::Release);
            })
            .unwrap();
        }
        mic.enable_interrupt().unwrap();
    }

    // Publish them so the poll loops can re-arm after each edge (see MIC_PINS).
    *MIC_PINS.lock().unwrap() = Some(MicPins {
        left: left_mic,
        right: right_mic,
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

    // Keep wifi alive -- dropping BlockingWifi tears the connection down.
    let _wifi = wifi;

    let client: Client = Arc::new(Mutex::new(None));

    // The behaviour loop gets its own thread with a generous stack (the
    // default pthread stack from sdkconfig is tight for this).
    {
        let rig = Arc::clone(&rig);
        let client = Arc::clone(&client);
        std::thread::Builder::new()
            .stack_size(8192)
            .spawn(move || run_state_machine(rig, client))
            .unwrap();
    }

    log::info!("cyclevroom ready, state={}", current_state().name());

    for stream in listener.incoming() {
        let stream = match stream {
            Ok(s) => s,
            Err(e) => {
                log::warn!("accept failed: {e}");
                continue;
            }
        };
        log::info!("client connected");
        handle_client(stream, &rig, &client);
    }

    unreachable!("listener loop never ends");
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

/// One client at a time, line-delimited commands, plain text replies:
/// `nc <ip> 7878`.
///
/// Note the single owned stream -- `TcpStream::try_clone()` is backed by
/// `dup()`, which ESP-IDF's lwIP layer doesn't implement (ENOSYS, and only at
/// the moment the first byte arrives). Read through a `BufReader`, write back
/// through `get_mut()`.
fn handle_client(stream: TcpStream, rig: &Arc<Mutex<Rig<'static>>>, client: &Client) {
    // Publish this connection's fd so the state thread can push state changes
    // and tuning deltas. Not a `try_clone()` -- that's dup()-backed and lwIP
    // returns ENOSYS -- so both threads share the one fd; see `notify`.
    *client.lock().unwrap() = Some(stream.as_raw_fd());

    // Greet with where the machine is right now, so the state stream is
    // self-contained: a client that connects mid-run knows the current state
    // without having to send `state` or wait for the next transition.
    notify(client, &format!("state {}", current_state().name()));

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
        let Some(reply) = handle_command(line.trim(), rig) else {
            continue; // blank line, no reply
        };
        if writeln!(reader.get_mut(), "{reply}").is_err() {
            break;
        }
    }

    // Clear the shared fd *before* `stream` drops at the end of this function,
    // so the state thread can never write to a closed socket.
    *client.lock().unwrap() = None;

    // A dropped connection must never leave the bot driving blind. It also
    // ends manual mode, so the usual pause-then-resume applies.
    MANUAL.store(false, Ordering::Release);
    ABORT.store(true, Ordering::Relaxed);
    FORCED_STATE.store(State::Paused as u8 as i32, Ordering::Release);
    rig.lock().unwrap().all_off();
    log::info!("client disconnected, motors stopped");
}

/// Execute one parsed command. `None` means "say nothing".
fn handle_command(line: &str, rig: &Arc<Mutex<Rig<'static>>>) -> Option<String> {
    Some(match parse_command(line) {
        Command::Empty => return None,

        Command::QueryState => format!("state {}", current_state().name()),

        Command::ForceState(target) => {
            // Queue it; the state thread applies it at its next decision point
            // so we never yank the rig out from under a move in progress.
            ABORT.store(true, Ordering::Relaxed);
            rig.lock().unwrap().all_off();
            FORCED_STATE.store(target as u8 as i32, Ordering::Release);
            // Forcing anywhere other than Paused means "carry on from here",
            // which is also how you leave manual mode.
            if target != State::Paused {
                MANUAL.store(false, Ordering::Release);
                ABORT.store(false, Ordering::Relaxed);
            }
            format!("ok state {}", target.name())
        }

        Command::Stop => {
            ABORT.store(true, Ordering::Relaxed);
            rig.lock().unwrap().all_off();
            FORCED_STATE.store(State::Paused as u8 as i32, Ordering::Release);
            "ok stop (paused)".into()
        }

        Command::Drive { wheel, dir, duty } => {
            if !enter_manual() {
                return Some("err: state machine didn't pause, try again".into());
            }
            let duty = duty.unwrap_or(DRIVE_DUTY.load(Ordering::Relaxed));
            let mut rig = rig.lock().unwrap();
            match wheel {
                Wheel::Left => rig.left_drive(dir.as_i32(), duty),
                Wheel::Right => rig.right_drive(dir.as_i32(), duty),
            }
            format!("ok {wheel:?} {dir:?} {duty}").to_lowercase()
        }

        Command::Alt { ms } => {
            // Wheel-by-wheel wiring check, unchanged in spirit from moving-wifi.
            if !enter_manual() {
                return Some("err: state machine didn't pause, try again".into());
            }
            let rig = Arc::clone(rig);
            ABORT.store(false, Ordering::Relaxed);
            std::thread::spawn(move || {
                let duty = DRIVE_DUTY.load(Ordering::Relaxed);
                let mut rig = rig.lock().unwrap();
                for (label, dir, is_right) in [
                    ("left fwd", 1, false),
                    ("left rev", -1, false),
                    ("right fwd", 1, true),
                    ("right rev", -1, true),
                ] {
                    if ABORT.load(Ordering::Relaxed) {
                        break;
                    }
                    log::info!("alt: {label}");
                    if is_right {
                        rig.right_drive(dir, duty);
                    } else {
                        rig.left_drive(dir, duty);
                    }
                    abortable_sleep(ms);
                    rig.stop();
                    abortable_sleep(200);
                }
            });
            "ok alt".into()
        }

        Command::Laser(what) => {
            match what {
                LaserCmd::On => rig.lock().unwrap().laser_on(),
                LaserCmd::Off => rig.lock().unwrap().laser_off(),
                LaserCmd::Fire => {
                    let rig = Arc::clone(rig);
                    ABORT.store(false, Ordering::Relaxed);
                    std::thread::spawn(move || {
                        let n = FIRE_AMOUNT.load(Ordering::Relaxed);
                        let on = FIRE_ON_MS.load(Ordering::Relaxed) as u64;
                        let off = FIRE_OFF_MS.load(Ordering::Relaxed) as u64;
                        for _ in 0..n {
                            if ABORT.load(Ordering::Relaxed) {
                                break;
                            }
                            rig.lock().unwrap().laser_on();
                            abortable_sleep(on);
                            rig.lock().unwrap().laser_off();
                            abortable_sleep(off);
                        }
                        rig.lock().unwrap().laser_off();
                    });
                }
            }
            format!("ok laser {what:?}").to_lowercase()
        }

        Command::Set { key, value } => {
            store(key, value);
            format!("ok set {} {value}", key.name())
        }

        Command::Get => {
            let mut out = String::new();
            for key in SetKey::ALL {
                if !out.is_empty() {
                    out.push(' ');
                }
                out.push_str(&format!("{}={}", key.name(), load(key)));
            }
            out
        }

        // Raw mic diagnostics. Nothing moves; this only reads.
        //
        // Samples both D0 lines for 500ms and reports what it saw: the ISR edge
        // counters (did an interrupt ever fire?), the level right now, and
        // whether the level changed at all during the window (is the pin even
        // wired to something that moves?). Those three together separate "the
        // interrupt isn't re-arming" from "D0 never transitions" from "D0 idles
        // high so there's no rising edge to catch".
        Command::Mic => {
            let before = edge_seq();
            let (mut l_lo, mut l_hi, mut r_lo, mut r_hi) = (false, false, false, false);
            let (mut l_now, mut r_now) = (false, false);
            {
                let mut guard = MIC_PINS.lock().unwrap();
                let pins = guard.as_mut().unwrap();
                let deadline = Instant::now() + Duration::from_millis(500);
                while Instant::now() < deadline {
                    l_now = pins.left.is_high();
                    r_now = pins.right.is_high();
                    if l_now { l_hi = true } else { l_lo = true }
                    if r_now { r_hi = true } else { r_lo = true }
                    sleep(Duration::from_millis(1));
                }
            }
            let after = edge_seq();
            let moved = |lo: bool, hi: bool| if lo && hi { "yes" } else { "no" };
            format!(
                "mic1(left,gpio4): level={} moved={} edges={} (+{})  \
                 mic2(right,gpio5): level={} moved={} edges={} (+{})",
                if l_now { "high" } else { "low" },
                moved(l_lo, l_hi),
                after.left,
                after.left.wrapping_sub(before.left),
                if r_now { "high" } else { "low" },
                moved(r_lo, r_hi),
                after.right,
                after.right.wrapping_sub(before.right),
            )
        }

        // Is the pin actually connected to a driving source?
        //
        // Force the internal pull-up, read; force the pull-down, read; restore
        // floating. A pin wired to an LM393's push-pull D0 is held by the
        // module and reads the *same* both ways (the module wins against a
        // ~45k internal pull). A pin with nothing on it follows the pull
        // exactly -- high then low. That difference tells us whether the mic
        // is reaching GPIO4/5 at all, which no amount of clapping can.
        Command::MicPull => {
            let mut guard = MIC_PINS.lock().unwrap();
            let pins = guard.as_mut().unwrap();
            // `PinDriver::set_pull` is private in esp-idf-hal, so go straight to
            // the IDF call it wraps. Safe: the pin is already configured as an
            // input by the driver we hold, and we restore the float on the way
            // out.
            use esp_idf_svc::sys::{
                gpio_pull_mode_t_GPIO_FLOATING, gpio_pull_mode_t_GPIO_PULLDOWN_ONLY,
                gpio_pull_mode_t_GPIO_PULLUP_ONLY, gpio_set_pull_mode,
            };
            let mut probe = |p: &mut PinDriver<'static, Input>| -> (bool, bool) {
                let num = p.pin() as i32;
                unsafe { gpio_set_pull_mode(num, gpio_pull_mode_t_GPIO_PULLUP_ONLY) };
                sleep(Duration::from_millis(20));
                let up = p.is_high();
                unsafe { gpio_set_pull_mode(num, gpio_pull_mode_t_GPIO_PULLDOWN_ONLY) };
                sleep(Duration::from_millis(20));
                let down = p.is_high();
                unsafe { gpio_set_pull_mode(num, gpio_pull_mode_t_GPIO_FLOATING) };
                (up, down)
            };
            let (l_up, l_down) = probe(pins.left);
            let (r_up, r_down) = probe(pins.right);
            let verdict = |up: bool, down: bool| match (up, down) {
                (true, false) => "FLOATING (follows pull -- nothing driving this pin)",
                (true, true) => "driven HIGH (module holding it high)",
                (false, false) => "driven LOW (module holding it low)",
                (false, true) => "inverted?? (check wiring)",
            };
            format!(
                "mic1(left,gpio4): pull_up={} pull_down={} -> {}  \
                 mic2(right,gpio5): pull_up={} pull_down={} -> {}",
                l_up as u8, l_down as u8, verdict(l_up, l_down),
                r_up as u8, r_down as u8, verdict(r_up, r_down),
            )
        }

        // Tight-loop level sampler: 2 seconds, no sleep between reads, so it
        // catches pulses far shorter than the 1ms `mic` poll could. Reports the
        // low-going pulse count and the shortest/longest seen, which is what
        // you watch while turning the KY-038 threshold pot.
        Command::MicWatch => {
            let mut guard = MIC_PINS.lock().unwrap();
            let pins = guard.as_mut().unwrap();
            let (mut l_pulses, mut r_pulses) = (0u32, 0u32);
            let (mut l_prev, mut r_prev) = (pins.left.is_high(), pins.right.is_high());
            let (mut l_min, mut l_max) = (u32::MAX, 0u32);
            let (mut l_fell_at, mut r_fell_at) = (0u32, 0u32);
            let (mut r_min, mut r_max) = (u32::MAX, 0u32);
            let deadline = Instant::now() + Duration::from_millis(2000);
            while Instant::now() < deadline {
                let now = now_us();
                let l = pins.left.is_high();
                let r = pins.right.is_high();
                if l != l_prev {
                    if l {
                        // rose: close the low pulse
                        let w = now.wrapping_sub(l_fell_at);
                        l_min = l_min.min(w);
                        l_max = l_max.max(w);
                        l_pulses += 1;
                    } else {
                        l_fell_at = now;
                    }
                    l_prev = l;
                }
                if r != r_prev {
                    if r {
                        let w = now.wrapping_sub(r_fell_at);
                        r_min = r_min.min(w);
                        r_max = r_max.max(w);
                        r_pulses += 1;
                    } else {
                        r_fell_at = now;
                    }
                    r_prev = r;
                }
            }
            let fmt = |n: u32, mn: u32, mx: u32| {
                if n == 0 {
                    "none".to_string()
                } else {
                    format!("{n} pulses, low for {}..{}us", mn, mx)
                }
            };
            format!(
                "2s tight sample -- mic1(left,gpio4): {}  mic2(right,gpio5): {}",
                fmt(l_pulses, l_min, l_max),
                fmt(r_pulses, r_min, r_max),
            )
        }

        Command::Help => "commands: state [name]; stop; left|right fwd|rev|stop [duty]; \
                          alt [ms]; laser on|off|fire; set <key> <val>; get; mic; micpull; micwatch; help \
                          (left/right/alt enter manual mode -- 'state listening' resumes)"
            .into(),

        Command::Error(msg) | Command::Unknown(msg) => msg,
    })
}

fn tunable(key: SetKey) -> &'static AtomicU32 {
    match key {
        SetKey::DriveDuty => &DRIVE_DUTY,
        SetKey::TurnDuty => &TURN_DUTY,
        SetKey::TurnMs => &TURN_MS,
        SetKey::ApproachMs => &APPROACH_MS,
        SetKey::ArmedMs => &ARMED_MS,
        SetKey::ActiveIdleMs => &ACTIVE_IDLE_MS,
        SetKey::FireAmount => &FIRE_AMOUNT,
        SetKey::FireOnMs => &FIRE_ON_MS,
        SetKey::FireOffMs => &FIRE_OFF_MS,
        SetKey::PausedMs => &PAUSED_MS,
        SetKey::SettleMs => &SETTLE_MS,
        SetKey::CenterDeadbandUs => &CENTER_DEADBAND_US,
        SetKey::BlankingMs => &BLANKING_MS,
        SetKey::PairTimeoutMs => &PAIR_TIMEOUT_MS,
    }
}

fn store(key: SetKey, value: u32) {
    tunable(key).store(value, Ordering::Relaxed);
}

fn load(key: SetKey) -> u32 {
    tunable(key).load(Ordering::Relaxed)
}
