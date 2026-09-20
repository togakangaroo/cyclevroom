//! The behaviour state machine.
//!
//! Pure: [`transition`] is a function of (state, event, tunables) with no I/O
//! and no clock of its own -- elapsed time arrives as an [`Event::Tick`]
//! carrying milliseconds-in-state. The bin target runs the real loop, does the
//! driving and the laser pulsing, and asks this module what to do next.
//!
//! ```text
//!            clap                 2nd clap (direction known)
//!  Listening ----> Active ----------------------------------> Turning
//!      ^             |                                           |
//!      |             | ACTIVE_IDLE_MS                            | TURN_MS
//!      +-------------+                                           v
//!      |                                                    Approaching
//!      |                                                         |
//!      |                                                         | APPROACH_MS
//!      |          clap                                           v
//!      |   Active <----------------------------------------- Armed
//!      |      ^                                                  |
//!      |      | clap                                             | ARMED_MS
//!      |      |                                                  v
//!      +--------------------------------------------------- Firing
//!            FIRE_AMOUNT pulses
//!
//!  any state --stop--> Paused --PAUSED_MS--> Listening
//! ```

/// Where the machine currently is.
///
/// `repr(u8)` because the live state is kept in an `AtomicU8` on the device, so
/// the server thread can answer `state` without taking the motor lock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum State {
    /// Waiting for a clap. Initial state. One clap -> `Active`.
    Listening = 0,
    /// Heard one clap, listening for a second to fix the direction.
    Active = 1,
    /// Spinning toward the clap for `turn_ms`. Mics ignored.
    Turning = 2,
    /// Driving forward for `approach_ms`. Mics ignored.
    Approaching = 3,
    /// Done driving: racing `armed_ms` against hearing another clap.
    Armed = 4,
    /// Pulsing the laser `fire_amount` times. A clap aborts the burst.
    Firing = 5,
    /// Everything halted for `paused_ms`. Entered immediately on `stop`.
    Paused = 6,
}

impl State {
    /// Parse the wire name used by the `state <name>` command.
    pub fn from_name(s: &str) -> Option<Self> {
        Some(match s {
            "listening" => State::Listening,
            "active" => State::Active,
            "turning" => State::Turning,
            "approaching" => State::Approaching,
            "armed" => State::Armed,
            "firing" => State::Firing,
            "paused" => State::Paused,
            _ => return None,
        })
    }

    /// The wire name, as reported by `state` and logged on every transition.
    pub fn name(self) -> &'static str {
        match self {
            State::Listening => "listening",
            State::Active => "active",
            State::Turning => "turning",
            State::Approaching => "approaching",
            State::Armed => "armed",
            State::Firing => "firing",
            State::Paused => "paused",
        }
    }

    pub fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => State::Listening,
            1 => State::Active,
            2 => State::Turning,
            3 => State::Approaching,
            4 => State::Armed,
            5 => State::Firing,
            6 => State::Paused,
            _ => return None,
        })
    }

    /// Whether the mics are live in this state.
    ///
    /// False during `Turning`/`Approaching` -- per the design call, we don't
    /// listen while the wheels are running, both because the motion is the
    /// point and because motor brush noise trips cheap KY-038 modules. (The
    /// eventual "live turning" idea would relax exactly this.)
    pub fn listens(self) -> bool {
        matches!(
            self,
            State::Listening | State::Active | State::Armed | State::Firing
        )
    }

    /// Whether the wheels may be running in this state.
    pub fn drives(self) -> bool {
        matches!(self, State::Turning | State::Approaching)
    }
}

/// Live-tunable durations and counts. Mirrors the `AtomicU32` statics the
/// `set`/`get` commands drive on the device; snapshotted into this plain struct
/// once per decision so a `set` landing mid-transition can't tear.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Tunables {
    /// `Active` gives up and returns to `Listening` after this long without a
    /// second clap.
    pub active_idle_ms: u32,
    /// How long to spin for the ~30 degree turn. The `moving-wifi` bench run
    /// put 90 degrees at 600ms, so 30 degrees lands near 200ms.
    pub turn_ms: u32,
    /// How long to drive forward after turning.
    pub approach_ms: u32,
    /// `Armed` fires the laser after this long with no clap.
    pub armed_ms: u32,
    /// How many on/off laser pulses one `Firing` burst runs.
    pub fire_amount: u32,
    /// How long the machine sits in `Paused` before resuming.
    pub paused_ms: u32,
}

impl Default for Tunables {
    fn default() -> Self {
        Self {
            active_idle_ms: 2_000,
            turn_ms: 200,
            approach_ms: 1_000,
            armed_ms: 2_000,
            fire_amount: 5,
            paused_ms: 10_000,
        }
    }
}

/// What happened. Fed to [`transition`] by the device loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// A mic fired. In `Active` this is the *second* clap, so it carries the
    /// direction the pair resolved to.
    Clap,
    /// Time passed. Carries total milliseconds spent in the current state.
    Tick { in_state_ms: u32 },
    /// The `Firing` burst ran all `fire_amount` pulses without interruption.
    FireComplete,
    /// `stop` arrived over TCP. Wins from any state, immediately.
    Stop,
    /// `state <name>` arrived over TCP -- operator override, also unconditional.
    Force(State),
}

/// The whole transition table. Returns `None` when the event doesn't move us.
///
/// Note the asymmetry between `Turning`/`Approaching` and everything else:
/// those two ignore `Clap` entirely (see [`State::listens`]) and advance only
/// on their `Tick` deadline, which is what makes "don't listen while the
/// wheels are turning" true by construction rather than by remembering to
/// check at the call site.
pub fn transition(state: State, event: Event, t: &Tunables) -> Option<State> {
    // Operator overrides outrank the table from every state, including Paused.
    match event {
        Event::Stop => return (state != State::Paused).then_some(State::Paused),
        Event::Force(target) => return (state != target).then_some(target),
        _ => {}
    }

    match (state, event) {
        // A clap wakes us up and starts the direction hunt.
        (State::Listening, Event::Clap) => Some(State::Active),

        // Second clap: the bin target has already resolved the delta into a
        // direction by the time it sends this, so we go straight to turning.
        (State::Active, Event::Clap) => Some(State::Turning),
        (State::Active, Event::Tick { in_state_ms }) if in_state_ms >= t.active_idle_ms => {
            Some(State::Listening)
        }

        // Deaf while driving -- Clap deliberately unmatched here.
        (State::Turning, Event::Tick { in_state_ms }) if in_state_ms >= t.turn_ms => {
            Some(State::Approaching)
        }
        (State::Approaching, Event::Tick { in_state_ms }) if in_state_ms >= t.approach_ms => {
            Some(State::Armed)
        }

        // Armed: a clap beats the clock, the clock beats silence.
        (State::Armed, Event::Clap) => Some(State::Active),
        (State::Armed, Event::Tick { in_state_ms }) if in_state_ms >= t.armed_ms => {
            Some(State::Firing)
        }

        // A clap mid-burst aborts the laser and re-enters the hunt.
        (State::Firing, Event::Clap) => Some(State::Active),
        (State::Firing, Event::FireComplete) => Some(State::Listening),

        (State::Paused, Event::Tick { in_state_ms }) if in_state_ms >= t.paused_ms => {
            Some(State::Listening)
        }

        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t() -> Tunables {
        Tunables::default()
    }

    fn tick(ms: u32) -> Event {
        Event::Tick { in_state_ms: ms }
    }

    const ALL: [State; 7] = [
        State::Listening,
        State::Active,
        State::Turning,
        State::Approaching,
        State::Armed,
        State::Firing,
        State::Paused,
    ];

    // --- the happy path ---------------------------------------------------

    #[test]
    fn clap_wakes_listening_into_active() {
        assert_eq!(
            transition(State::Listening, Event::Clap, &t()),
            Some(State::Active)
        );
    }

    #[test]
    fn second_clap_starts_the_turn() {
        assert_eq!(
            transition(State::Active, Event::Clap, &t()),
            Some(State::Turning)
        );
    }

    #[test]
    fn turning_then_approaching_then_armed() {
        assert_eq!(
            transition(State::Turning, tick(200), &t()),
            Some(State::Approaching)
        );
        assert_eq!(
            transition(State::Approaching, tick(1_000), &t()),
            Some(State::Armed)
        );
    }

    #[test]
    fn armed_times_out_into_firing() {
        assert_eq!(
            transition(State::Armed, tick(2_000), &t()),
            Some(State::Firing)
        );
    }

    #[test]
    fn a_finished_burst_returns_to_listening() {
        assert_eq!(
            transition(State::Firing, Event::FireComplete, &t()),
            Some(State::Listening)
        );
    }

    #[test]
    fn the_full_loop_comes_back_around() {
        // clap, clap, turn, drive, wait, fire, done -> back to listening
        let mut s = State::Listening;
        for (event, expected) in [
            (Event::Clap, State::Active),
            (Event::Clap, State::Turning),
            (tick(200), State::Approaching),
            (tick(1_000), State::Armed),
            (tick(2_000), State::Firing),
            (Event::FireComplete, State::Listening),
        ] {
            s = transition(s, event, &t()).expect("expected a transition");
            assert_eq!(s, expected);
        }
    }

    // --- timeouts ---------------------------------------------------------

    #[test]
    fn active_gives_up_after_active_idle_ms() {
        assert_eq!(
            transition(State::Active, tick(2_000), &t()),
            Some(State::Listening)
        );
    }

    #[test]
    fn timeouts_do_not_fire_early() {
        assert_eq!(transition(State::Active, tick(1_999), &t()), None);
        assert_eq!(transition(State::Turning, tick(199), &t()), None);
        assert_eq!(transition(State::Approaching, tick(999), &t()), None);
        assert_eq!(transition(State::Armed, tick(1_999), &t()), None);
        assert_eq!(transition(State::Paused, tick(9_999), &t()), None);
    }

    #[test]
    fn timeouts_fire_exactly_on_the_boundary() {
        // >= not >, so landing exactly on the deadline counts
        assert!(transition(State::Active, tick(2_000), &t()).is_some());
        assert!(transition(State::Turning, tick(200), &t()).is_some());
        assert!(transition(State::Approaching, tick(1_000), &t()).is_some());
        assert!(transition(State::Armed, tick(2_000), &t()).is_some());
        assert!(transition(State::Paused, tick(10_000), &t()).is_some());
    }

    #[test]
    fn timeouts_follow_the_tunables() {
        let t = Tunables {
            turn_ms: 50,
            approach_ms: 60,
            armed_ms: 70,
            active_idle_ms: 80,
            paused_ms: 90,
            ..Tunables::default()
        };
        assert_eq!(transition(State::Turning, tick(50), &t), Some(State::Approaching));
        assert_eq!(transition(State::Approaching, tick(60), &t), Some(State::Armed));
        assert_eq!(transition(State::Armed, tick(70), &t), Some(State::Firing));
        assert_eq!(transition(State::Active, tick(80), &t), Some(State::Listening));
        assert_eq!(transition(State::Paused, tick(90), &t), Some(State::Listening));
    }

    // --- claps are ignored while the wheels run ---------------------------

    #[test]
    fn driving_states_ignore_claps() {
        assert_eq!(transition(State::Turning, Event::Clap, &t()), None);
        assert_eq!(transition(State::Approaching, Event::Clap, &t()), None);
    }

    #[test]
    fn listens_and_drives_are_mutually_exclusive() {
        // Paused is the one state that does neither -- it's halted. But
        // nothing may ever claim both.
        for s in ALL {
            assert!(
                !(s.listens() && s.drives()),
                "{s:?} cannot both listen and drive"
            );
        }
        assert!(!State::Paused.listens());
        assert!(!State::Paused.drives());
    }

    #[test]
    fn listens_agrees_with_the_transition_table() {
        // `listens()` is what the device loop consults before arming the mic
        // ISRs, so it has to match what the table actually does with a Clap,
        // or the two will drift apart. Paused is deaf and also ignores claps.
        for s in ALL {
            let moved = transition(s, Event::Clap, &t()).is_some();
            assert_eq!(
                s.listens(),
                moved,
                "{s:?}: listens()={} but a clap {} move it",
                s.listens(),
                if moved { "does" } else { "does not" }
            );
        }
    }

    #[test]
    fn a_clap_mid_burst_aborts_the_laser() {
        assert_eq!(
            transition(State::Firing, Event::Clap, &t()),
            Some(State::Active)
        );
    }

    #[test]
    fn a_clap_while_armed_restarts_the_hunt() {
        assert_eq!(
            transition(State::Armed, Event::Clap, &t()),
            Some(State::Active)
        );
    }

    // --- stop / pause -----------------------------------------------------

    #[test]
    fn stop_pauses_from_every_state() {
        for s in ALL {
            if s == State::Paused {
                continue;
            }
            assert_eq!(
                transition(s, Event::Stop, &t()),
                Some(State::Paused),
                "stop from {s:?} must pause"
            );
        }
    }

    #[test]
    fn stop_while_already_paused_is_a_no_op() {
        // no transition, so the device loop won't restart the 10s timer --
        // a second `stop` shouldn't extend the pause
        assert_eq!(transition(State::Paused, Event::Stop, &t()), None);
    }

    #[test]
    fn paused_resumes_into_listening() {
        assert_eq!(
            transition(State::Paused, tick(10_000), &t()),
            Some(State::Listening)
        );
    }

    #[test]
    fn paused_ignores_claps_and_fire_completion() {
        assert_eq!(transition(State::Paused, Event::Clap, &t()), None);
        assert_eq!(transition(State::Paused, Event::FireComplete, &t()), None);
    }

    // --- operator override ------------------------------------------------

    #[test]
    fn force_reaches_every_state_from_every_state() {
        for from in ALL {
            for to in ALL {
                let got = transition(from, Event::Force(to), &t());
                if from == to {
                    assert_eq!(got, None, "forcing {from:?} onto itself is a no-op");
                } else {
                    assert_eq!(got, Some(to), "force {from:?} -> {to:?}");
                }
            }
        }
    }

    #[test]
    fn force_outranks_a_pending_timeout() {
        // even sitting on a deadline, an explicit force wins
        assert_eq!(
            transition(State::Armed, Event::Force(State::Listening), &t()),
            Some(State::Listening)
        );
    }

    // --- misc -------------------------------------------------------------

    #[test]
    fn unrelated_events_do_not_move_anything() {
        assert_eq!(transition(State::Listening, Event::FireComplete, &t()), None);
        assert_eq!(transition(State::Listening, tick(999_999), &t()), None);
        assert_eq!(transition(State::Turning, Event::FireComplete, &t()), None);
    }

    #[test]
    fn names_round_trip() {
        for s in ALL {
            assert_eq!(State::from_name(s.name()), Some(s), "{s:?}");
        }
        assert_eq!(State::from_name("nonsense"), None);
        assert_eq!(State::from_name("Listening"), None); // lowercase on the wire
    }

    #[test]
    fn u8_round_trips() {
        for s in ALL {
            assert_eq!(State::from_u8(s as u8), Some(s), "{s:?}");
        }
        assert_eq!(State::from_u8(7), None);
    }

    #[test]
    fn defaults_match_the_spec() {
        let t = Tunables::default();
        assert_eq!(t.turn_ms, 200);
        assert_eq!(t.approach_ms, 1_000);
        assert_eq!(t.armed_ms, 2_000);
        assert_eq!(t.active_idle_ms, 2_000);
        assert_eq!(t.fire_amount, 5);
        assert_eq!(t.paused_ms, 10_000);
    }
}
