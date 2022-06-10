use super::{FromBytes, ToBytes, ErrorKind};

const HEADER_SIZE: usize = 4;

#[derive(Debug)]
pub struct InstructionPacket {
    id: u8,
    instr: Instruction,
    payload: Vec<u8>,
}

impl InstructionPacket {
    pub fn ping_packet(id: u8) -> Self {
        InstructionPacket { id: id, instr: Instruction::Ping, payload: vec![] }
    }
    pub fn read_packet(id: u8, reg: u8, length: u8) -> Self {
        InstructionPacket { id: id, instr: Instruction::Read, payload: vec![reg, length] }
    }
    pub fn write_packet(id: u8, reg: u8, value: Vec<u8>) -> Self {
        let mut payload = vec![reg];
        payload.extend(value);

        InstructionPacket { id: id, instr: Instruction::Write, payload: payload }
    }
}

#[derive(Debug)]
pub struct StatusPacket {
    pub id: u8,
    pub error: Vec<DynamixelErrorKind>,
    pub payload: Vec<u8>,
}

#[derive(Debug)]
pub enum DynamixelErrorKind {
    InstructionError,
    OverloadError,
    ChecksumError,
    RangeError,
    OverheatingError,
    AngleLimitError,
    InputVoltageError,
}
impl DynamixelErrorKind {
    fn from_byte(error: u8) -> Vec<Self> {
        (0..7).into_iter()
            .filter(|i| error & (1 << i) != 0)
            .map(|i| DynamixelErrorKind::from_bit(i).unwrap())
            .collect()
    }
    fn from_bit(b: u8) -> Option<Self> {
        match b {
            6 => Some(DynamixelErrorKind::InstructionError),
            5 => Some(DynamixelErrorKind::OverloadError),
            4 => Some(DynamixelErrorKind::ChecksumError),
            3 => Some(DynamixelErrorKind::RangeError),
            2 => Some(DynamixelErrorKind::OverheatingError),
            1 => Some(DynamixelErrorKind::AngleLimitError),
            0 => Some(DynamixelErrorKind::InputVoltageError),
            _ => None,
        }
    }
}


impl FromBytes for StatusPacket {
    fn from_bytes(bytes: Vec<u8>) -> Result<Self, ErrorKind> {
        if bytes.len() < 6 {
            return Err(ErrorKind::ParsingError);
        }

        let read_crc = *bytes.last().unwrap();
        let computed_crc = crc(&bytes[2..bytes.len() - 1]);
        if read_crc != computed_crc {
            return Err(ErrorKind::ChecksumError);
        }

        if bytes[0] != 255 || bytes[1] != 255 {
            return Err(ErrorKind::ParsingError);
        }

        let id = bytes[2];
        let payload_length = bytes[3] as usize;
        let error = DynamixelErrorKind::from_byte(bytes[4]);

        if payload_length != bytes.len() - HEADER_SIZE || payload_length < 2 {
            return Err(ErrorKind::ParsingError);
        }

        let payload = bytes[5..3 + payload_length].to_vec();

        Ok(StatusPacket {
            id: id,
            error: error,
            payload: payload,
        })

    }
}


#[derive(Debug)]
enum Instruction {
    Ping,
    Read,
    Write,
}

impl Instruction {
    fn value(&self) -> u8 {
        match self {
            Instruction::Ping => 0x01,
            Instruction::Read => 0x02,
            Instruction::Write => 0x03,
        }
    }
}

impl ToBytes for InstructionPacket {
    fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::new();

        let payload_length: u8 = (self.payload.len() + 2).try_into().unwrap();

        bytes.extend(vec![255, 255, self.id, payload_length].iter());
        bytes.push(self.instr.value());
        bytes.extend(self.payload.iter());
        bytes.push(crc(&bytes[2..]));

        bytes
    }
}

fn crc(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for b in data {
        crc = crc.wrapping_add(*b);
    }
    !crc
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn create_ping_packet() {
        let p = InstructionPacket::ping_packet(1);
        let bytes = p.to_bytes();
        assert_eq!(bytes, [0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB]);
    }

    #[test]
    fn create_read_packet() {
        let p = InstructionPacket::read_packet(1, 0x2B, 1);
        let bytes = p.to_bytes();
        assert_eq!(bytes, [0xFF, 0xFF, 0x01, 0x04, 0x02, 0x2B, 0x01, 0xCC]);
    }

    #[test]
    fn create_write_packet() {
        let p = InstructionPacket::write_packet(10, 24, vec![1]);
        let bytes = p.to_bytes();
        assert_eq!(bytes, [255, 255, 10, 4, 3, 24, 1, 213]);

        let p = InstructionPacket::write_packet(0xFE, 0x03, vec![1]);
        let bytes = p.to_bytes();
        assert_eq!(bytes, [0xFF, 0xFF, 0xFE, 0x04, 0x03, 0x03, 0x01, 0xF6]);
    }

    #[test]
    fn parse_status_packet() {
        let bytes = vec![0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC];
        let sp = StatusPacket::from_bytes(bytes).unwrap();
        assert_eq!(sp.id, 1);
        assert_eq!(sp.error.len(), 0);
        assert_eq!(sp.payload.len(), 0);

        let bytes = vec![0xFF, 0xFF, 0x01, 0x03, 0x00, 0x20, 0xDB];
        let sp = StatusPacket::from_bytes(bytes).unwrap();
        assert_eq!(sp.id, 1);
        assert_eq!(sp.error.len(), 0);
        assert_eq!(sp.payload.len(), 1);
        assert_eq!(sp.payload[0], 0x20);
    }
}