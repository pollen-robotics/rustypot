pub mod conversion;
pub mod info;

pub mod dynamixel;
pub mod feetech;
pub mod orbita;
pub(crate) mod servo_macro;

pub use info::{encoding_for, Encoding, RegisterError, RegisterType, ServoInfo, WordOrder};

/// The read timeout of an ID sweep at `baudrate`.
///
/// Every absent id costs one timeout, so a sweep at a port's usual timeout takes
/// minutes. A Model Number read and its answer are under 320 bits on the wire; the
/// floor leaves room for the motor's return delay and for USB scheduling.
pub fn scan_timeout(baudrate: u32) -> std::time::Duration {
    std::time::Duration::from_micros(u64::from(320_000_000 / baudrate).max(5_000))
}

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
    fn port_settings_reach_the_serial_port() {
        use crate::fake_port::FakePort;
        use std::time::Duration;

        let port = FakePort::new(vec![]);
        let settings = port.settings();
        let mut c = sts3215::Sts3215Controller::new()
            .with_serial_port(Box::new(port))
            .with_protocol_v1();

        c.set_baudrate(57_600).unwrap();
        c.set_timeout(Duration::from_millis(20)).unwrap();

        let settings = settings.lock().unwrap();
        assert_eq!(settings.baud_rate, 57_600);
        assert_eq!(settings.timeouts.last(), Some(&Duration::from_millis(20)));
    }

    #[test]
    fn registers_are_read_and_written_by_name() {
        use crate::fake_port::FakePort;

        // The answers: homing_offset of motor 1 as 0x0AC5, then the status of the write.
        let port = FakePort::new(vec![
            vec![0xFF, 0xFF, 0x01, 0x04, 0x00, 0xC5, 0x0A, 0x2B],
            vec![0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC],
        ]);
        let written = port.written();
        let mut c = sts3215::Sts3215Controller::new()
            .with_serial_port(Box::new(port))
            .with_protocol_v1();

        // Sign-magnitude on bit 11: 0x0AC5 is -709.
        assert_eq!(c.read_register(1, "homing_offset").unwrap(), -709);

        // Sign-magnitude on bit 15: -100 is 0x8064, little-endian at address 42.
        c.write_register(1, "goal_position", -100).unwrap();
        assert_eq!(
            written.lock().unwrap()[1],
            [0xFF, 0xFF, 0x01, 0x05, 0x03, 0x2A, 0x64, 0x80, 0xE8]
        );
    }

    #[test]
    fn big_endian_servos_write_each_word_high_byte_first() {
        use crate::fake_port::FakePort;

        let port = FakePort::new(vec![vec![0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC]]);
        let written = port.written();
        let mut c = scs0009::Scs0009Controller::new()
            .with_serial_port(Box::new(port))
            .with_protocol_v1();

        c.write_register(1, "goal_position", 0x1234).unwrap();
        assert_eq!(
            written.lock().unwrap()[0],
            [0xFF, 0xFF, 0x01, 0x05, 0x03, 0x2A, 0x12, 0x34, 0x86]
        );
    }

    #[test]
    fn a_bad_name_or_value_never_reaches_the_bus() {
        use crate::fake_port::FakePort;

        let port = FakePort::new(vec![]);
        let written = port.written();
        let mut c = sts3215::Sts3215Controller::new()
            .with_serial_port(Box::new(port))
            .with_protocol_v1();

        assert_eq!(
            c.write_register(1, "torque_enable", 256)
                .unwrap_err()
                .to_string(),
            "256 does not fit register 'torque_enable'"
        );
        assert_eq!(
            c.read_register(1, "current_limit").unwrap_err().to_string(),
            "no register named 'current_limit'"
        );
        assert!(written.lock().unwrap().is_empty());
    }

    #[test]
    fn a_scan_reports_the_ids_that_answer_and_puts_the_timeout_back() {
        use crate::fake_port::FakePort;
        use std::time::Duration;

        // Only motor 2 answers, with model number 777: bytes 9, 3 at address 3.
        let port = FakePort::new(vec![
            vec![],
            vec![0xFF, 0xFF, 0x02, 0x04, 0x00, 0x09, 0x03, 0xED],
            vec![],
        ]);
        let settings = port.settings();
        let mut c = sts3215::Sts3215Controller::new()
            .with_serial_port(Box::new(port))
            .with_protocol_v1();

        let found = c.scan(&[1, 2, 3]).unwrap();

        assert_eq!(found, std::collections::BTreeMap::from([(2, 777)]));
        assert_eq!(
            settings.lock().unwrap().timeouts,
            [
                Duration::from_millis(10),
                super::scan_timeout(1_000_000),
                Duration::from_millis(10)
            ]
        );
    }

    #[test]
    fn the_scan_timeout_follows_the_baud_rate_down_to_a_floor() {
        use std::time::Duration;

        assert_eq!(super::scan_timeout(1_000_000), Duration::from_millis(5));
        assert_eq!(super::scan_timeout(115_200), Duration::from_millis(5));
        assert_eq!(super::scan_timeout(57_600), Duration::from_micros(5_555));
        assert_eq!(super::scan_timeout(9_600), Duration::from_micros(33_333));
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
