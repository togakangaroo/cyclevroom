//! Pure logic for cyclevroom: the behaviour state machine, the two-mic
//! direction classifier, and the command parser.
//!
//! Deliberately free of `esp-idf-svc` (and of any hardware type at all) so the
//! whole thing builds and tests on the host -- `./test.sh`. The bin target owns
//! the GPIO, PWM, wifi and socket work and calls into here for every decision.

pub mod machine;
pub mod parse;
pub mod tdoa;

pub use machine::{Event, State, Tunables};
pub use parse::{Command, SetKey};
pub use tdoa::{classify_delta, Side};
