//! The TCP command parser.
//!
//! Split out from the socket handling so the whole command surface is testable
//! on the host. [`parse_command`] turns one line into a [`Command`]; the bin
//! target matches on that and does the actual work.

use crate::machine::State;

/// A tunable addressable by `set <key> <value>` / reported by `get`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SetKey {
    DriveDuty,
    TurnDuty,
    TurnMs,
    ApproachMs,
    ArmedMs,
    ActiveIdleMs,
    FireAmount,
    FireOnMs,
    FireOffMs,
    PausedMs,
    SettleMs,
    CenterDeadbandUs,
    BlankingMs,
    PairTimeoutMs,
}

impl SetKey {
    pub fn from_name(s: &str) -> Option<Self> {
        Some(match s {
            "drive_duty" => SetKey::DriveDuty,
            "turn_duty" => SetKey::TurnDuty,
            "turn_ms" => SetKey::TurnMs,
            "approach_ms" => SetKey::ApproachMs,
            "armed_ms" => SetKey::ArmedMs,
            "active_idle_ms" => SetKey::ActiveIdleMs,
            "fire_amount" => SetKey::FireAmount,
            "fire_on_ms" => SetKey::FireOnMs,
            "fire_off_ms" => SetKey::FireOffMs,
            "paused_ms" => SetKey::PausedMs,
            "settle_ms" => SetKey::SettleMs,
            "center_deadband_us" => SetKey::CenterDeadbandUs,
            "blanking_ms" => SetKey::BlankingMs,
            "pair_timeout_ms" => SetKey::PairTimeoutMs,
            _ => return None,
        })
    }

    pub fn name(self) -> &'static str {
        match self {
            SetKey::DriveDuty => "drive_duty",
            SetKey::TurnDuty => "turn_duty",
            SetKey::TurnMs => "turn_ms",
            SetKey::ApproachMs => "approach_ms",
            SetKey::ArmedMs => "armed_ms",
            SetKey::ActiveIdleMs => "active_idle_ms",
            SetKey::FireAmount => "fire_amount",
            SetKey::FireOnMs => "fire_on_ms",
            SetKey::FireOffMs => "fire_off_ms",
            SetKey::PausedMs => "paused_ms",
            SetKey::SettleMs => "settle_ms",
            SetKey::CenterDeadbandUs => "center_deadband_us",
            SetKey::BlankingMs => "blanking_ms",
            SetKey::PairTimeoutMs => "pair_timeout_ms",
        }
    }

    /// Every key, so `get` and `help` can't drift out of sync with the parser.
    pub const ALL: [SetKey; 14] = [
        SetKey::DriveDuty,
        SetKey::TurnDuty,
        SetKey::TurnMs,
        SetKey::ApproachMs,
        SetKey::ArmedMs,
        SetKey::ActiveIdleMs,
        SetKey::FireAmount,
        SetKey::FireOnMs,
        SetKey::FireOffMs,
        SetKey::PausedMs,
        SetKey::SettleMs,
        SetKey::CenterDeadbandUs,
        SetKey::BlankingMs,
        SetKey::PairTimeoutMs,
    ];
}

/// Which wheel a manual drive command addresses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Wheel {
    Left,
    Right,
}

/// What a manual drive command asks for. `1` forward, `-1` reverse, `0` brake.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dir {
    Fwd,
    Rev,
    Stop,
}

impl Dir {
    pub fn as_i32(self) -> i32 {
        match self {
            Dir::Fwd => 1,
            Dir::Rev => -1,
            Dir::Stop => 0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LaserCmd {
    On,
    Off,
    /// Run one full `fire_amount` burst, as `Firing` would.
    Fire,
}

/// One parsed line.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Command {
    /// Blank line -- no reply.
    Empty,
    /// `state` with no argument: report where we are.
    QueryState,
    /// `state <name>`: force a transition.
    ForceState(State),
    /// `stop`: brake, laser off, enter `Paused`.
    Stop,
    /// `left|right fwd|rev|stop [duty]` -- wiring debug, kept from moving-wifi.
    Drive {
        wheel: Wheel,
        dir: Dir,
        duty: Option<u32>,
    },
    /// `alt [ms]` -- the four-phase wheel cycle, kept from moving-wifi.
    Alt { ms: u64 },
    /// `laser on|off|fire`.
    Laser(LaserCmd),
    Set { key: SetKey, value: u32 },
    Get,
    Help,
    /// `mic`: raw mic diagnostics -- live D0 levels and ISR edge counters.
    /// Read-only and motionless, for working out why a clap isn't heard.
    Mic,
    /// `micpull`: drive the internal pull up then down and read the level back.
    /// A pin connected to an actively-driven D0 barely budges; a floating
    /// (unconnected) pin follows the pull exactly. Distinguishes "module isn't
    /// reaching the pin" from "module is quiet".
    MicPull,
    /// `micwatch`: 2s tight-loop level sampler. Counts low-going pulses and
    /// their widths -- the readout to watch while turning the threshold pot.
    MicWatch,
    /// Recognised shape, bad contents -- the string is the reply to send back.
    Error(String),
    /// Nothing matched.
    Unknown(String),
}

/// Parse one line from the command socket.
pub fn parse_command(line: &str) -> Command {
    let mut parts = line.split_whitespace();
    let Some(cmd) = parts.next() else {
        return Command::Empty;
    };

    match cmd {
        "state" => match parts.next() {
            None => Command::QueryState,
            Some(name) => match State::from_name(name) {
                Some(s) => Command::ForceState(s),
                None => Command::Error(format!(
                    "unknown state '{name}' (listening, active, turning, \
                     approaching, armed, firing, paused)"
                )),
            },
        },

        "stop" => Command::Stop,

        "left" | "right" => {
            let wheel = if cmd == "left" { Wheel::Left } else { Wheel::Right };
            let Some(dir_word) = parts.next() else {
                return Command::Error("usage: left|right fwd|rev|stop [duty_pct]".into());
            };
            let dir = match dir_word {
                "fwd" => Dir::Fwd,
                "rev" => Dir::Rev,
                "stop" => Dir::Stop,
                _ => return Command::Error("dir must be fwd, rev, or stop".into()),
            };
            // An unparseable duty falls back to the configured default rather
            // than erroring, matching moving-wifi's behaviour.
            let duty = parts.next().and_then(|s| s.parse::<u32>().ok());
            Command::Drive { wheel, dir, duty }
        }

        "alt" => Command::Alt {
            ms: parts
                .next()
                .and_then(|s| s.parse::<u64>().ok())
                .unwrap_or(1_000),
        },

        "laser" => match parts.next() {
            Some("on") => Command::Laser(LaserCmd::On),
            Some("off") => Command::Laser(LaserCmd::Off),
            Some("fire") => Command::Laser(LaserCmd::Fire),
            _ => Command::Error("usage: laser on|off|fire".into()),
        },

        "set" => {
            let (Some(key), Some(val)) = (parts.next(), parts.next()) else {
                return Command::Error("usage: set <key> <value>".into());
            };
            let Some(key) = SetKey::from_name(key) else {
                return Command::Error(format!("unknown key '{key}'"));
            };
            let Ok(value) = val.parse::<u32>() else {
                return Command::Error("value must be an integer".into());
            };
            Command::Set { key, value }
        }

        "get" => Command::Get,
        "mic" => Command::Mic,
        "micpull" => Command::MicPull,
        "micwatch" => Command::MicWatch,
        "help" => Command::Help,

        other => Command::Unknown(format!("unknown command '{other}' (try 'help')")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn blank_lines_are_empty() {
        assert_eq!(parse_command(""), Command::Empty);
        assert_eq!(parse_command("   "), Command::Empty);
        assert_eq!(parse_command("\t"), Command::Empty);
    }

    #[test]
    fn state_with_no_argument_queries() {
        assert_eq!(parse_command("state"), Command::QueryState);
    }

    #[test]
    fn state_with_a_name_forces() {
        assert_eq!(
            parse_command("state firing"),
            Command::ForceState(State::Firing)
        );
        assert_eq!(
            parse_command("state listening"),
            Command::ForceState(State::Listening)
        );
    }

    #[test]
    fn every_state_is_reachable_from_the_wire() {
        for s in [
            State::Listening,
            State::Active,
            State::Turning,
            State::Approaching,
            State::Armed,
            State::Firing,
            State::Paused,
        ] {
            assert_eq!(
                parse_command(&format!("state {}", s.name())),
                Command::ForceState(s),
                "{s:?} must be forceable over TCP"
            );
        }
    }

    #[test]
    fn a_bad_state_name_explains_itself() {
        let Command::Error(msg) = parse_command("state sideways") else {
            panic!("expected an error");
        };
        assert!(msg.contains("sideways"), "{msg}");
        assert!(msg.contains("listening"), "should list the valid states: {msg}");
    }

    #[test]
    fn stop_parses() {
        assert_eq!(parse_command("stop"), Command::Stop);
    }

    #[test]
    fn drive_commands_parse() {
        assert_eq!(
            parse_command("left fwd"),
            Command::Drive {
                wheel: Wheel::Left,
                dir: Dir::Fwd,
                duty: None
            }
        );
        assert_eq!(
            parse_command("right rev 80"),
            Command::Drive {
                wheel: Wheel::Right,
                dir: Dir::Rev,
                duty: Some(80)
            }
        );
        assert_eq!(
            parse_command("left stop"),
            Command::Drive {
                wheel: Wheel::Left,
                dir: Dir::Stop,
                duty: None
            }
        );
    }

    #[test]
    fn drive_without_a_direction_is_a_usage_error() {
        assert!(matches!(parse_command("left"), Command::Error(_)));
    }

    #[test]
    fn a_bad_direction_is_rejected() {
        assert!(matches!(parse_command("left sideways"), Command::Error(_)));
    }

    #[test]
    fn a_junk_duty_falls_back_to_the_default() {
        assert_eq!(
            parse_command("left fwd notanumber"),
            Command::Drive {
                wheel: Wheel::Left,
                dir: Dir::Fwd,
                duty: None
            }
        );
    }

    #[test]
    fn dir_maps_to_the_motor_convention() {
        assert_eq!(Dir::Fwd.as_i32(), 1);
        assert_eq!(Dir::Rev.as_i32(), -1);
        assert_eq!(Dir::Stop.as_i32(), 0);
    }

    #[test]
    fn alt_defaults_to_a_second() {
        assert_eq!(parse_command("alt"), Command::Alt { ms: 1_000 });
        assert_eq!(parse_command("alt 250"), Command::Alt { ms: 250 });
        assert_eq!(parse_command("alt junk"), Command::Alt { ms: 1_000 });
    }

    #[test]
    fn laser_commands_parse() {
        assert_eq!(parse_command("laser on"), Command::Laser(LaserCmd::On));
        assert_eq!(parse_command("laser off"), Command::Laser(LaserCmd::Off));
        assert_eq!(parse_command("laser fire"), Command::Laser(LaserCmd::Fire));
        assert!(matches!(parse_command("laser"), Command::Error(_)));
        assert!(matches!(parse_command("laser sideways"), Command::Error(_)));
    }

    #[test]
    fn set_parses() {
        assert_eq!(
            parse_command("set turn_ms 250"),
            Command::Set {
                key: SetKey::TurnMs,
                value: 250
            }
        );
    }

    #[test]
    fn every_key_is_settable_and_round_trips() {
        for key in SetKey::ALL {
            assert_eq!(
                parse_command(&format!("set {} 42", key.name())),
                Command::Set { key, value: 42 },
                "{key:?} must be settable"
            );
            assert_eq!(SetKey::from_name(key.name()), Some(key));
        }
    }

    #[test]
    fn the_spec_keys_all_exist() {
        // the names called out in the design, spelled literally so a rename
        // can't silently break the documented command surface
        for name in [
            "turn_ms",
            "approach_ms",
            "armed_ms",
            "active_idle_ms",
            "fire_amount",
            "turn_duty",
            "drive_duty",
        ] {
            assert!(SetKey::from_name(name).is_some(), "missing key {name}");
        }
    }

    #[test]
    fn set_with_a_bad_key_or_value_errors() {
        let Command::Error(msg) = parse_command("set nonsense 1") else {
            panic!("expected an error");
        };
        assert!(msg.contains("nonsense"), "{msg}");

        assert!(matches!(parse_command("set turn_ms abc"), Command::Error(_)));
        assert!(matches!(parse_command("set turn_ms"), Command::Error(_)));
        assert!(matches!(parse_command("set"), Command::Error(_)));
        // negative values don't fit the u32 tunables
        assert!(matches!(parse_command("set turn_ms -5"), Command::Error(_)));
    }

    #[test]
    fn get_and_help_parse() {
        assert_eq!(parse_command("get"), Command::Get);
        assert_eq!(parse_command("help"), Command::Help);
    }

    #[test]
    fn unknown_commands_are_reported() {
        let Command::Unknown(msg) = parse_command("fly") else {
            panic!("expected unknown");
        };
        assert!(msg.contains("fly"), "{msg}");
        assert!(msg.contains("help"), "{msg}");
    }

    #[test]
    fn extra_whitespace_is_tolerated() {
        assert_eq!(
            parse_command("  set   turn_ms    250  "),
            Command::Set {
                key: SetKey::TurnMs,
                value: 250
            }
        );
        assert_eq!(parse_command("  stop  "), Command::Stop);
    }
}
