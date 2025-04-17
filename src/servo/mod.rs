pub mod dynamixel;
pub mod feetech;
pub mod orbita;
pub(crate) mod servo_macro;

crate::register_servo!(
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
    )
);
