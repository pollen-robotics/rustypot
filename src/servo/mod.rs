pub mod conversion;

pub mod dynamixel;
pub mod feetech;
pub mod orbita;
pub(crate) mod servo_macro;

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

    fn __repr__(&self) -> String {
        format!(
            "RegisterInfo(name='{}', addr={}, size={}, access=RegisterAccess.{:?})",
            self.name, self.addr, self.size, self.access
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
        (XL330M288, 1200)
    ),
    servo: (dynamixel, XL430,
        (XL430W250, 1060),
        (XL430W2502, 1090)
    ),
    servo: (feetech, STS3215,
        (STS3215, 2307)
    ),
    servo: (feetech, SCS0009,
        (SCS0009, 1284) // Bytes 5, 4 at address 3, read big-endian as the SCS series stores words.
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
    use super::ServoKind;

    #[test]
    fn scs0009_model_number_resolves_to_its_definition() {
        assert!(matches!(
            ServoKind::try_from(1284),
            Ok(ServoKind::feetech_SCS0009)
        ));
        assert!(ServoKind::try_from(1280).is_err());
    }
}
