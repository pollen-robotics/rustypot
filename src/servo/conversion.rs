pub trait Conversion {
    type RegisterType;
    type UsiType;
    fn from_raw(raw: Self::RegisterType) -> Self::UsiType;
    fn to_raw(value: Self::UsiType) -> Self::RegisterType;
}

impl Conversion for bool {
    type RegisterType = u8;
    type UsiType = bool;

    fn from_raw(raw: u8) -> bool {
        raw != 0
    }

    fn to_raw(value: bool) -> u8 {
        if value {
            1
        } else {
            0
        }
    }
}
