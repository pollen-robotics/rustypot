pub trait Serializable {
    fn from_bytes(bytes: Vec<u8>) -> Option<Self>
    where
        Self: Sized;
    fn to_bytes(&self) -> Vec<u8>;
}

impl Serializable for bool {
    fn from_bytes(bytes: Vec<u8>) -> Option<Self> {
        match u8::from_bytes(bytes) {
            Some(0) => Some(false),
            Some(1) => Some(true),
            _ => None,
        }
    }
    fn to_bytes(&self) -> Vec<u8> {
        u8::to_bytes(&(*self as u8))
    }
}

impl Serializable for u8 {
    fn from_bytes(bytes: Vec<u8>) -> Option<Self> {
        match bytes.len() {
            1 => Some(bytes[0]),
            _ => None,
        }
    }
    fn to_bytes(&self) -> Vec<u8> {
        vec![*self]
    }
}

impl Serializable for u16 {
    fn from_bytes(bytes: Vec<u8>) -> Option<Self>
    where
        Self: Sized,
    {
        match bytes.len() {
            2 => Some(u16::from_le_bytes(bytes[0..2].try_into().unwrap())),
            _ => None,
        }
    }

    fn to_bytes(&self) -> Vec<u8> {
        self.to_le_bytes().to_vec()
    }
}

impl Serializable for i16 {
    fn from_bytes(bytes: Vec<u8>) -> Option<Self>
    where
        Self: Sized,
    {
        match bytes.len() {
            2 => Some(i16::from_le_bytes(bytes[0..2].try_into().unwrap())),
            _ => None,
        }
    }

    fn to_bytes(&self) -> Vec<u8> {
        self.to_le_bytes().to_vec()
    }
}

impl Serializable for f32 {
    fn from_bytes(bytes: Vec<u8>) -> Option<Self>
    where
        Self: Sized,
    {
        match bytes.len() {
            4 => Some(f32::from_le_bytes(bytes[0..4].try_into().unwrap())),
            _ => None,
        }
    }

    fn to_bytes(&self) -> Vec<u8> {
        self.to_le_bytes().to_vec()
    }
}

impl Serializable for i32 {
    fn from_bytes(bytes: Vec<u8>) -> Option<Self>
    where
        Self: Sized,
    {
        match bytes.len() {
            4 => Some(i32::from_le_bytes(bytes[0..4].try_into().unwrap())),
            _ => None,
        }
    }

    fn to_bytes(&self) -> Vec<u8> {
        self.to_le_bytes().to_vec()
    }
}

impl Serializable for (bool, bool, bool) {
    fn from_bytes(bytes: Vec<u8>) -> Option<Self>
    where
        Self: Sized,
    {
        match bytes.len() {
            3 => Some((
                bool::from_bytes(vec![bytes[0]]).unwrap(),
                bool::from_bytes(vec![bytes[1]]).unwrap(),
                bool::from_bytes(vec![bytes[2]]).unwrap(),
            )),
            _ => None,
        }
    }

    fn to_bytes(&self) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend(self.0.to_bytes());
        v.extend(self.1.to_bytes());
        v.extend(self.2.to_bytes());
        v
    }
}

impl Serializable for (i32, i32, i32) {
    fn from_bytes(bytes: Vec<u8>) -> Option<Self>
    where
        Self: Sized,
    {
        match bytes.len() {
            12 => Some((
                i32::from_bytes(bytes[0..4].to_vec()).unwrap(),
                i32::from_bytes(bytes[0..4].to_vec()).unwrap(),
                i32::from_bytes(bytes[0..4].to_vec()).unwrap(),
            )),
            _ => None,
        }
    }

    fn to_bytes(&self) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend(self.0.to_bytes());
        v.extend(self.1.to_bytes());
        v.extend(self.2.to_bytes());
        v
    }
}

impl Serializable for (f32, f32, f32) {
    fn from_bytes(bytes: Vec<u8>) -> Option<Self>
    where
        Self: Sized,
    {
        match bytes.len() {
            12 => Some((
                f32::from_bytes(bytes[0..4].to_vec()).unwrap(),
                f32::from_bytes(bytes[0..4].to_vec()).unwrap(),
                f32::from_bytes(bytes[0..4].to_vec()).unwrap(),
            )),
            _ => None,
        }
    }

    fn to_bytes(&self) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend(self.0.to_bytes());
        v.extend(self.1.to_bytes());
        v.extend(self.2.to_bytes());
        v
    }
}
