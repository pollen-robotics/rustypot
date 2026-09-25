pub mod conversion;
pub mod info;

pub mod dynamixel;
pub mod feetech;
pub mod orbita;
pub(crate) mod servo_macro;

pub use info::{encoding_for, Encoding, RegisterType, ServoInfo, WordOrder};

/// Where a register exists in a servo's control table.
///
/// Each servo module exposes its full table as `REGISTERS`, which lets callers work with
/// registers chosen at runtime (building an indirect address map, or a config tool that
/// takes register names) without hardcoding addresses.
///
/// In Python the same table is reached through the static `registers()` and
/// `register(name)` of each controller class, with no serial port involved.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
#[cfg_attr(
    feature = "python",
    pyo3_stub_gen::derive::gen_stub_pyclass,
    pyo3::pyclass(frozen, eq, hash, skip_from_py_object)
)]
pub struct RegisterInfo {
    /// Register name, matching the generated accessor (`present_position` -> `read_present_position`).
    pub name: &'static str,
    /// Address in the control table.
    pub addr: u8,
    /// Size in bytes.
    pub size: u8,
    /// Whether the register can be read, written, or both.
    pub access: RegisterAccess,
    /// How the raw bytes carry a sign, for callers reading the register through the raw
    /// address API.
    pub encoding: Encoding,
}

#[cfg(feature = "python")]
#[pyo3_stub_gen::derive::gen_stub_pymethods]
#[pyo3::pymethods]
impl RegisterInfo {
    /// Register name, as spelled in the servo definition (`present_position`).
    #[getter]
    fn name(&self) -> &'static str {
        self.name
    }

    /// Address in the control table.
    #[getter]
    fn addr(&self) -> u8 {
        self.addr
    }

    /// Size in bytes.
    #[getter]
    fn size(&self) -> u8 {
        self.size
    }

    /// Whether the register can be read, written, or both.
    #[getter]
    fn access(&self) -> RegisterAccess {
        self.access
    }

    /// How the raw bytes carry a sign: "unsigned", "twos_complement" or "sign_magnitude".
    #[getter]
    fn encoding(&self) -> &'static str {
        self.encoding.as_str()
    }

    /// The sign bit of a sign-magnitude register, `None` for the other encodings.
    #[getter]
    fn sign_bit(&self) -> Option<u8> {
        self.encoding.sign_bit()
    }

    fn __repr__(&self) -> String {
        let sign = match self.encoding.sign_bit() {
            Some(bit) => format!(", sign_bit={bit}"),
            None => String::new(),
        };
        format!(
            "RegisterInfo(name='{}', addr={}, size={}, access=RegisterAccess.{:?}, encoding='{}'{})",
            self.name,
            self.addr,
            self.size,
            self.access,
            self.encoding.as_str(),
            sign
        )
    }
}

/// How a register can be accessed, as declared in its servo definition.
///
/// Worth checking before a generic tool writes a register it was given by name: nothing else
/// at runtime says whether a write will be refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(
    feature = "python",
    pyo3_stub_gen::derive::gen_stub_pyclass_enum,
    pyo3::pyclass(frozen, eq, eq_int, hash, skip_from_py_object)
)]
pub enum RegisterAccess {
    /// Read only (`r`).
    Read,
    /// Write only (`w`).
    Write,
    /// Readable and writable (`rw`).
    ReadWrite,
}

crate::register_servo!(
    servo: (dynamixel, AX,
        (AX12, 12), // All AX12, except the W are equivalent.
        (AX12W, 300),
        (AX18A, 18)
    ),
    servo: (dynamixel, MX,
        (MX28, 29),
        (MX64, 310),
        (MX106, 320)
    ),
    servo: (dynamixel, XL320,
        (XL320, 35)
    ),
    servo: (dynamixel, XL330,
        (XL330M077, 1190),
        (XL330M288, 1200),
        (XC330T181, 1210), // Same control table as the XL330.
        (XC330T288, 1220)
    ),
    servo: (dynamixel, XL430,
        (XL430W250, 1060),
        (XL430W2502, 1090),
        (XC430W150, 1070),
        (XM430W350, 1020), // The XL430 definition is the XM430 control table, current registers included.
        (XM540W270, 1120),
        (XH540W150, 1110)
    ),
    servo: (feetech, STS3215,
        (STS3215, 777), // Bytes 9, 3 at address 3, read little-endian like the scan does.
        (STS3250, 2825),
        (SM8512BL, 11272) // Same control table as the STS3215.
    ),
    servo: (feetech, SCS0009,
        (SCS0009, 1280)
    ),
    servo: (feetech, SCS0043,
        (SCS0043, 1290)
    ),
    servo: (orbita, orbita2d_poulpe,
        (orbita2d_poulpe, 10020)
    ),
    servo: (orbita, orbita2d_foc,
        (orbita2d_foc, 10021)
    ),
    servo: (orbita, orbita3d_poulpe,
        (orbita3d_poulpe, 10030)
    ),
    servo: (orbita, orbita3d_foc,
        (orbita3d_foc, 10031)
    )
);

#[cfg(test)]
mod tests {
    use super::{
        dynamixel::{mx, xl330, xl430},
        feetech::{scs0009, sts3215},
        Encoding, ServoKind, WordOrder,
    };

    #[test]
    fn definitions_state_their_facts() {
        assert_eq!(sts3215::INFO.resolution, Some(4096));
        assert_eq!(sts3215::INFO.word_order, WordOrder::Little);
        assert!(sts3215::INFO.supports_sync_read);
        assert!(sts3215::INFO.baudrates.contains(&(1_000_000, 0)));

        assert_eq!(scs0009::INFO.resolution, Some(1024));
        assert_eq!(scs0009::INFO.word_order, WordOrder::Big);
        assert!(!scs0009::INFO.supports_sync_read);

        assert_eq!(mx::INFO.resolution, Some(4096));
        assert!(mx::INFO.baudrates.is_empty());
    }

    #[test]
    fn encodings_come_from_the_override_or_the_type() {
        let encoding = |reg: Option<super::RegisterInfo>| reg.unwrap().encoding;
        assert_eq!(
            encoding(sts3215::register("present_position")),
            Encoding::SignMagnitude { sign_bit: 15 }
        );
        assert_eq!(
            encoding(sts3215::register("present_load")),
            Encoding::SignMagnitude { sign_bit: 10 }
        );
        assert_eq!(
            encoding(sts3215::register("min_position_limit")),
            Encoding::Unsigned
        );
        assert_eq!(encoding(sts3215::register("id")), Encoding::Unsigned);
        assert_eq!(
            encoding(scs0009::register("goal_position")),
            Encoding::Unsigned
        );
        // Declared i32, nothing to override.
        assert_eq!(
            encoding(xl330::register("goal_position")),
            Encoding::TwosComplement
        );
        // Declared u16, overridden.
        assert_eq!(
            encoding(xl330::register("goal_pwm")),
            Encoding::TwosComplement
        );
        assert_eq!(
            encoding(xl430::register("present_position")),
            Encoding::TwosComplement
        );
        assert_eq!(
            encoding(xl430::register("torque_enable")),
            Encoding::Unsigned
        );
    }

    #[test]
    fn model_numbers_resolve_to_their_definition() {
        assert!(matches!(
            ServoKind::try_from(777),
            Ok(ServoKind::feetech_STS3215)
        ));
        assert!(matches!(
            ServoKind::try_from(2825),
            Ok(ServoKind::feetech_STS3250)
        ));
        assert!(matches!(
            ServoKind::try_from(11272),
            Ok(ServoKind::feetech_SM8512BL)
        ));
        assert!(matches!(
            ServoKind::try_from(1220),
            Ok(ServoKind::dynamixel_XC330T288)
        ));
        assert!(matches!(
            ServoKind::try_from(1020),
            Ok(ServoKind::dynamixel_XM430W350)
        ));
        assert!(ServoKind::try_from(2307).is_err());
    }
}
