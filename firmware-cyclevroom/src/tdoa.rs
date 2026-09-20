//! Two-mic direction classification.
//!
//! Same physics as the earlier `firmware-left-or-right` prototype: the near mic
//! hears the clap first, and the gap between the two rising edges tells us which
//! side. Mics are ~5cm apart, so the largest physically-possible gap is
//! `0.05 / 343 ~= 146us` -- we do not pretend to resolve an angle, only LEFT,
//! RIGHT, or CENTER (gap too small to trust against KY-038 jitter).
//!
//! # Sign convention
//!
//! **mic1 is LEFT (GPIO4), mic2 is RIGHT (GPIO5), and delta is `mic1 - mic2`.**
//!
//! So a *negative* delta means the left mic's timestamp is the smaller one --
//! left heard it first -- and the sound is to the LEFT. This is the number
//! reported to the TCP client on every `Active -> Turning` transition, and
//! getting it backwards would make the bot chase claps in the wrong direction,
//! hence [`classify_delta`] being the most heavily tested thing in the crate.

/// Which way the clap came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    Left,
    Right,
    /// |delta| fell inside the deadband -- the two mics fired close enough
    /// together that we can't trust the ordering. Treated as "dead ahead".
    Center,
}

/// Classify a `mic1 - mic2` (left minus right) microsecond delta.
///
/// `deadband_us` is `CENTER_DEADBAND_US`: below this magnitude we return
/// [`Side::Center`] rather than guess. The comparison is inclusive, matching
/// the `delta_us <= CENTER_DEADBAND_US` test in `firmware-left-or-right`.
pub fn classify_delta(delta_us: i64, deadband_us: u32) -> Side {
    if delta_us.unsigned_abs() <= deadband_us as u64 {
        Side::Center
    } else if delta_us < 0 {
        // left timestamp smaller => left fired first => sound is to the left
        Side::Left
    } else {
        Side::Right
    }
}

/// Classify when only one mic ever fired (the other timed out after
/// `PAIR_TIMEOUT_MS`). No delta to compare, so we report the side that did
/// fire -- it's the only evidence we have.
pub fn classify_lone(near: Side) -> Side {
    near
}

#[cfg(test)]
mod tests {
    use super::*;

    const DEADBAND: u32 = 60;

    #[test]
    fn negative_delta_is_left() {
        // left mic timestamp is smaller => left heard it first
        assert_eq!(classify_delta(-87, DEADBAND), Side::Left);
        assert_eq!(classify_delta(-146, DEADBAND), Side::Left);
        assert_eq!(classify_delta(-61, DEADBAND), Side::Left);
    }

    #[test]
    fn positive_delta_is_right() {
        assert_eq!(classify_delta(87, DEADBAND), Side::Right);
        assert_eq!(classify_delta(146, DEADBAND), Side::Right);
        assert_eq!(classify_delta(61, DEADBAND), Side::Right);
    }

    #[test]
    fn within_deadband_is_center() {
        assert_eq!(classify_delta(0, DEADBAND), Side::Center);
        assert_eq!(classify_delta(59, DEADBAND), Side::Center);
        assert_eq!(classify_delta(-59, DEADBAND), Side::Center);
    }

    #[test]
    fn deadband_boundary_is_inclusive_center() {
        // exactly at the deadband counts as center, matching the
        // `<=` in firmware-left-or-right
        assert_eq!(classify_delta(60, DEADBAND), Side::Center);
        assert_eq!(classify_delta(-60, DEADBAND), Side::Center);
        // one microsecond past it commits to a side
        assert_eq!(classify_delta(61, DEADBAND), Side::Right);
        assert_eq!(classify_delta(-61, DEADBAND), Side::Left);
    }

    #[test]
    fn sign_convention_is_mic1_minus_mic2() {
        // Spelled out as arithmetic so this test fails loudly if the
        // subtraction order is ever flipped at the call site.
        let mic1_left_us: i64 = 1_000; // left heard it first
        let mic2_right_us: i64 = 1_087;
        assert_eq!(
            classify_delta(mic1_left_us - mic2_right_us, DEADBAND),
            Side::Left
        );

        let mic1_left_us: i64 = 1_087; // right heard it first
        let mic2_right_us: i64 = 1_000;
        assert_eq!(
            classify_delta(mic1_left_us - mic2_right_us, DEADBAND),
            Side::Right
        );
    }

    #[test]
    fn zero_deadband_never_returns_center_for_nonzero_delta() {
        assert_eq!(classify_delta(1, 0), Side::Right);
        assert_eq!(classify_delta(-1, 0), Side::Left);
        // a genuine tie is still center even with the deadband off
        assert_eq!(classify_delta(0, 0), Side::Center);
    }

    #[test]
    fn a_huge_deadband_swallows_everything() {
        assert_eq!(classify_delta(146, 1_000), Side::Center);
        assert_eq!(classify_delta(-146, 1_000), Side::Center);
    }

    #[test]
    fn lone_mic_reports_that_side() {
        assert_eq!(classify_lone(Side::Left), Side::Left);
        assert_eq!(classify_lone(Side::Right), Side::Right);
    }
}
