//! What a servo definition states about itself beyond its registers.
//!
//! A caller going through the raw address API gets bytes back and has to know how to
//! read them: in which byte order, whether a value carries a sign and how, how many
//! steps make a turn, which register value selects a baud rate. `generate_servo!`
//! collects these from the definition into [`ServoInfo`] and the `encoding` of each
//! [`RegisterInfo`](crate::servo::RegisterInfo), so they live next to the registers
//! instead of being copied by every consumer.

use std::fmt;

use crate::servo::RegisterInfo;

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

    /// The low `size` bytes of `raw`, laid out as the servo sends them.
    ///
    /// Little-endian is the plain byte string. A big-endian servo swaps the two bytes of
    /// each 16-bit word and keeps the low word first, so a 4-byte value is mixed-endian:
    /// `0x12345678` goes out as `56 78 12 34`.
    pub fn to_bytes(self, raw: u64, size: usize) -> Vec<u8> {
        let mut bytes = raw.to_le_bytes()[..size].to_vec();
        if self == WordOrder::Big {
            bytes.chunks_exact_mut(2).for_each(|word| word.swap(0, 1));
        }
        bytes
    }

    /// The integer that `bytes`, as the servo sent them, stand for.
    pub fn from_bytes(self, bytes: &[u8]) -> u64 {
        let mut raw = [0u8; 8];
        raw[..bytes.len()].copy_from_slice(bytes);
        if self == WordOrder::Big {
            raw[..bytes.len()]
                .chunks_exact_mut(2)
                .for_each(|word| word.swap(0, 1));
        }
        u64::from_le_bytes(raw)
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

    /// The wire value carrying `value` in a register `size` bytes wide, or `None` when
    /// the value does not fit.
    pub fn encode(self, value: i64, size: u8) -> Option<u64> {
        let bits = 8 * u32::from(size);
        let value = i128::from(value);
        let fits = match self {
            Encoding::Unsigned => (0..1i128 << bits).contains(&value),
            Encoding::TwosComplement => {
                (-(1i128 << (bits - 1))..1i128 << (bits - 1)).contains(&value)
            }
            Encoding::SignMagnitude { sign_bit } => value.unsigned_abs() < 1u128 << sign_bit,
        };
        if !fits {
            return None;
        }
        Some(match self {
            Encoding::SignMagnitude { sign_bit } if value < 0 => {
                value.unsigned_abs() as u64 | 1 << sign_bit
            }
            _ => (value & ((1i128 << bits) - 1)) as u64,
        })
    }

    /// The value that a register `size` bytes wide carries as `raw` on the wire.
    pub fn decode(self, raw: u64, size: u8) -> i64 {
        let bits = 8 * u32::from(size);
        match self {
            Encoding::Unsigned => raw as i64,
            Encoding::TwosComplement => {
                let raw = i128::from(raw);
                let signed = if raw >= 1i128 << (bits - 1) {
                    raw - (1i128 << bits)
                } else {
                    raw
                };
                signed as i64
            }
            Encoding::SignMagnitude { sign_bit } => {
                let magnitude = (raw & ((1 << sign_bit) - 1)) as i64;
                if raw >> sign_bit & 1 == 1 {
                    -magnitude
                } else {
                    magnitude
                }
            }
        }
    }
}

/// Why a register could not be read or written by name.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RegisterError {
    /// The servo definition has no register of that name.
    Unknown(String),
    /// The register is wider than the integers this API carries.
    NotAnInteger { name: &'static str, size: u8 },
    /// The value does not fit the register's width and encoding.
    OutOfRange { name: &'static str, value: i64 },
    /// A Sync Read or Sync Write reached motors whose definitions put the register at
    /// different addresses or sizes; one instruction carries a single address and length.
    Layout(String),
}

impl fmt::Display for RegisterError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            RegisterError::Unknown(name) => write!(f, "no register named '{name}'"),
            RegisterError::NotAnInteger { name, size } => {
                write!(f, "register '{name}' is {size} bytes wide, not an integer")
            }
            RegisterError::OutOfRange { name, value } => {
                write!(f, "{value} does not fit register '{name}'")
            }
            RegisterError::Layout(name) => write!(
                f,
                "register '{name}' is not at the same address and size on every motor asked"
            ),
        }
    }
}

impl std::error::Error for RegisterError {}

/// A servo definition as a value: its name, the protocol it speaks, what it states
/// about itself and its registers.
///
/// Every servo module has one as `DEFINITION`. A controller addresses a motor of
/// another definition on its bus through it, see `set_definition` on the controllers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(
    feature = "python",
    pyo3_stub_gen::derive::gen_stub_pyclass,
    pyo3::pyclass(frozen, eq, from_py_object)
)]
pub struct ServoDefinition {
    pub name: &'static str,
    /// The Dynamixel protocol version the servo speaks: 1 or 2.
    pub protocol: u8,
    pub info: ServoInfo,
    pub registers: &'static [RegisterInfo],
}

impl ServoDefinition {
    /// Look up a register by name, as spelled in `registers`.
    pub fn register(&self, name: &str) -> Option<RegisterInfo> {
        self.registers.iter().copied().find(|r| r.name == name)
    }
}

#[cfg(feature = "python")]
#[pyo3_stub_gen::derive::gen_stub_pymethods]
#[pyo3::pymethods]
impl ServoDefinition {
    /// The servo's name, as its controller class spells it (`XL330`).
    #[getter]
    fn name(&self) -> &'static str {
        self.name
    }

    /// The Dynamixel protocol version the servo speaks: 1 or 2.
    #[getter]
    fn protocol(&self) -> u8 {
        self.protocol
    }

    fn __repr__(&self) -> String {
        format!("ServoDefinition('{}')", self.name)
    }
}

impl RegisterInfo {
    /// The bytes that write `value` to this register, in the servo's `order`.
    pub fn encode(&self, order: WordOrder, value: i64) -> Result<Vec<u8>, RegisterError> {
        self.integer_width()?;
        let raw = self
            .encoding
            .encode(value, self.size)
            .ok_or(RegisterError::OutOfRange {
                name: self.name,
                value,
            })?;
        Ok(order.to_bytes(raw, self.size.into()))
    }

    /// The value this register carries in `bytes`, read in the servo's `order`.
    pub fn decode(&self, order: WordOrder, bytes: &[u8]) -> Result<i64, RegisterError> {
        self.integer_width()?;
        Ok(self.encoding.decode(order.from_bytes(bytes), self.size))
    }

    fn integer_width(&self) -> Result<(), RegisterError> {
        if self.size > 8 {
            return Err(RegisterError::NotAnInteger {
                name: self.name,
                size: self.size,
            });
        }
        Ok(())
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

    fn reg(name: &'static str, size: u8, encoding: Encoding) -> RegisterInfo {
        RegisterInfo {
            name,
            addr: 0,
            size,
            access: crate::servo::RegisterAccess::ReadWrite,
            encoding,
        }
    }

    #[test]
    fn big_endian_swaps_the_bytes_of_each_word_low_word_first() {
        for (order, size, expected) in [
            (WordOrder::Little, 1, vec![0x78]),
            (WordOrder::Big, 1, vec![0x78]),
            (WordOrder::Little, 2, vec![0x78, 0x56]),
            (WordOrder::Big, 2, vec![0x56, 0x78]),
            (WordOrder::Little, 4, vec![0x78, 0x56, 0x34, 0x12]),
            (WordOrder::Big, 4, vec![0x56, 0x78, 0x12, 0x34]),
        ] {
            assert_eq!(
                order.to_bytes(0x12345678, size),
                expected,
                "{order:?} {size}"
            );
            assert_eq!(
                order.from_bytes(&expected),
                0x12345678 & ((1 << (8 * size)) - 1),
                "{order:?} {size}"
            );
        }
    }

    #[test]
    fn encodings_round_trip_through_the_wire_value() {
        for (encoding, size, value, raw) in [
            (Encoding::Unsigned, 2, 4095, 4095),
            (Encoding::TwosComplement, 2, -1, 0xFFFF),
            (Encoding::TwosComplement, 4, -709, 0xFFFF_FD3B),
            (Encoding::TwosComplement, 4, 1624, 1624),
            (
                Encoding::SignMagnitude { sign_bit: 11 },
                2,
                -709,
                0x0800 | 709,
            ),
            (Encoding::SignMagnitude { sign_bit: 15 }, 2, 1337, 1337),
            (
                Encoding::SignMagnitude { sign_bit: 15 },
                2,
                -1337,
                0x8000 | 1337,
            ),
        ] {
            assert_eq!(
                encoding.encode(value, size),
                Some(raw),
                "{encoding:?} {value}"
            );
            assert_eq!(encoding.decode(raw, size), value, "{encoding:?} {raw}");
        }
    }

    #[test]
    fn values_that_do_not_fit_are_refused() {
        assert_eq!(Encoding::Unsigned.encode(-1, 1), None);
        assert_eq!(Encoding::Unsigned.encode(256, 1), None);
        assert_eq!(Encoding::Unsigned.encode(255, 1), Some(255));
        assert_eq!(Encoding::TwosComplement.encode(128, 1), None);
        assert_eq!(Encoding::TwosComplement.encode(-129, 1), None);
        assert_eq!(Encoding::TwosComplement.encode(-128, 1), Some(0x80));
        let offset = Encoding::SignMagnitude { sign_bit: 11 };
        assert_eq!(offset.encode(2048, 2), None);
        assert_eq!(offset.encode(-2048, 2), None);
        assert_eq!(offset.encode(-2047, 2), Some(0x0FFF));
    }

    #[test]
    fn a_register_encodes_in_the_servo_word_order() {
        let position = reg("goal_position", 2, Encoding::SignMagnitude { sign_bit: 15 });
        assert_eq!(
            position.encode(WordOrder::Little, -100).unwrap(),
            [0x64, 0x80]
        );
        assert_eq!(position.decode(WordOrder::Little, &[0x64, 0x80]), Ok(-100));

        let position = reg("goal_position", 2, Encoding::Unsigned);
        assert_eq!(
            position.encode(WordOrder::Big, 0x1234).unwrap(),
            [0x12, 0x34]
        );
        assert_eq!(position.decode(WordOrder::Big, &[0x12, 0x34]), Ok(0x1234));

        assert_eq!(
            reg("torque_enable", 1, Encoding::Unsigned).encode(WordOrder::Little, 256),
            Err(RegisterError::OutOfRange {
                name: "torque_enable",
                value: 256
            })
        );
        assert_eq!(
            reg("pid", 12, Encoding::Unsigned).decode(WordOrder::Little, &[0; 12]),
            Err(RegisterError::NotAnInteger {
                name: "pid",
                size: 12
            })
        );
    }

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
