//! What a servo definition states about itself beyond its registers.
//!
//! A caller going through the raw address API gets bytes back and has to know how to
//! read them: in which byte order, whether a value carries a sign and how, how many
//! steps make a turn, which register value selects a baud rate. `generate_servo!`
//! collects these from the definition into [`ServoInfo`] and the `encoding` of each
//! [`RegisterInfo`](crate::servo::RegisterInfo), so they live next to the registers
//! instead of being copied by every consumer.

/// Byte order of multi-byte registers on the wire.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum WordOrder {
    /// Least significant byte first: Dynamixel, Feetech STS and SMS.
    Little,
    /// Most significant byte first within each 16-bit word: Feetech SCS.
    Big,
}

impl WordOrder {
    pub const fn as_str(self) -> &'static str {
        match self {
            WordOrder::Little => "little",
            WordOrder::Big => "big",
        }
    }
}

/// How a register's raw bytes carry a sign.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Encoding {
    /// No sign: the bytes are a plain unsigned integer.
    Unsigned,
    /// Two's complement over the register's full width: Dynamixel.
    TwosComplement,
    /// Magnitude in the low bits and the sign in bit `sign_bit`: Feetech STS and SMS.
    SignMagnitude { sign_bit: u8 },
}

impl Encoding {
    pub const fn as_str(self) -> &'static str {
        match self {
            Encoding::Unsigned => "unsigned",
            Encoding::TwosComplement => "twos_complement",
            Encoding::SignMagnitude { .. } => "sign_magnitude",
        }
    }

    /// The sign bit of a sign-magnitude register, `None` for the other encodings.
    pub const fn sign_bit(self) -> Option<u8> {
        match self {
            Encoding::SignMagnitude { sign_bit } => Some(sign_bit),
            _ => None,
        }
    }
}

/// Facts about a servo that are not registers.
///
/// Every field has a default, so a definition only states what applies to it: a servo
/// without an encoder has no resolution, and one that does not say otherwise is
/// little-endian and answers Sync Read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub struct ServoInfo {
    /// Encoder steps per turn, when the servo reports positions in steps.
    pub resolution: Option<u32>,
    pub word_order: WordOrder,
    /// Whether the firmware answers the Sync Read instruction.
    pub supports_sync_read: bool,
    /// Serial rates the servo can be set to, as (baud rate, baud rate register value).
    pub baudrates: &'static [(u32, u8)],
}

/// Default encoding of a register type, when the servo definition says nothing about
/// the register: two's complement for signed integers, unsigned for everything else.
pub trait RegisterType {
    const DEFAULT_ENCODING: Encoding;
}

macro_rules! register_types {
    ($encoding:expr => $($t:ty),+ $(,)?) => {
        $(impl RegisterType for $t {
            const DEFAULT_ENCODING: Encoding = $encoding;
        })+
    };
}

register_types!(Encoding::TwosComplement => i8, i16, i32, i64);
register_types!(Encoding::Unsigned => u8, u16, u32, u64, f32, f64, bool);

// The Orbita definitions read and write whole structs; a sign is not a property of those.
macro_rules! unsigned_generic_register_types {
    ($($t:ty),+ $(,)?) => {
        $(impl<T> RegisterType for $t {
            const DEFAULT_ENCODING: Encoding = Encoding::Unsigned;
        })+
    };
}

use crate::servo::orbita::{orbita2d_foc, orbita2d_poulpe, orbita3d_foc, orbita3d_poulpe};

register_types!(Encoding::Unsigned =>
    orbita2d_foc::MotorPositionSpeedLoad, orbita2d_foc::Pid,
    orbita2d_poulpe::MotorPositionSpeedLoad, orbita2d_poulpe::Pid,
    orbita3d_foc::DiskPositionSpeedLoad, orbita3d_foc::Pid,
    orbita3d_poulpe::MotorPositionSpeedLoad, orbita3d_poulpe::Pid,
);
unsigned_generic_register_types!(
    orbita2d_foc::MotorValue<T>,
    orbita2d_foc::Vec3d<T>,
    orbita2d_poulpe::MotorValue<T>,
    orbita3d_foc::DiskValue<T>,
    orbita3d_foc::Vec3d<T>,
    orbita3d_poulpe::MotorValue<T>,
    orbita3d_poulpe::Vec3d<T>,
);

/// The encoding of the register called `name`: the servo's override for it when there
/// is one, else the default for its type. Runs at compile time inside `REGISTERS`.
pub const fn encoding_for(
    name: &str,
    overrides: &[(&str, Encoding)],
    default: Encoding,
) -> Encoding {
    let mut i = 0;
    while i < overrides.len() {
        if str_eq(overrides[i].0, name) {
            return overrides[i].1;
        }
        i += 1;
    }
    default
}

const fn str_eq(a: &str, b: &str) -> bool {
    let (a, b) = (a.as_bytes(), b.as_bytes());
    if a.len() != b.len() {
        return false;
    }
    let mut i = 0;
    while i < a.len() {
        if a[i] != b[i] {
            return false;
        }
        i += 1;
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn overrides_win_over_the_type_default() {
        const OVERRIDES: &[(&str, Encoding)] =
            &[("present_load", Encoding::SignMagnitude { sign_bit: 10 })];
        assert_eq!(
            encoding_for("present_load", OVERRIDES, Encoding::Unsigned),
            Encoding::SignMagnitude { sign_bit: 10 }
        );
        assert_eq!(
            encoding_for("present_loa", OVERRIDES, Encoding::Unsigned),
            Encoding::Unsigned
        );
        assert_eq!(
            encoding_for(
                "goal_position",
                &[],
                <i32 as RegisterType>::DEFAULT_ENCODING
            ),
            Encoding::TwosComplement
        );
    }
}
