pub mod conversion;

/// Where a register exists in a servo's control table.
///
/// Each servo module exposes its full table as `REGISTERS`, which lets callers work with
/// registers chosen at runtime (building an indirect address map, or a config tool that
/// takes register names) without hardcoding addresses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RegisterInfo {
    /// Register name, matching the generated accessor (`present_position` -> `read_present_position`).
    pub name: &'static str,
    /// Address in the control table.
    pub addr: u8,
    /// Size in bytes.
    pub size: u8,
}

pub mod dynamixel;
pub mod feetech;
pub mod orbita;
pub(crate) mod servo_macro;

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
