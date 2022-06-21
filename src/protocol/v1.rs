use super::{DynamixelErrorKind, FromBytes, ToBytes};
use crate::CommunicationErrorKind;

const HEADER_SIZE: usize = 4;

#[derive(Debug)]
pub struct InstructionPacket {
    pub id: u8,
    pub instr: Instruction,
    pub payload: Vec<u8>,
}

impl InstructionPacket {
    pub fn ping_packet(id: u8) -> Self {
        InstructionPacket {
            id,
            instr: Instruction::Ping,
            payload: vec![],
        }
    }
    pub fn read_packet(id: u8, reg: u8, length: u8) -> Self {
        InstructionPacket {
            id,
            instr: Instruction::Read,
            payload: vec![reg, length],
        }
    }
    pub fn write_packet(id: u8, reg: u8, value: Vec<u8>) -> Self {
        let mut payload = vec![reg];
        payload.extend(value);

        InstructionPacket {
            id,
            instr: Instruction::Write,
            payload,
        }
    }
}

#[derive(Debug)]
pub struct StatusPacket {
    pub id: u8,
    pub error: Vec<DynamixelErrorKind>,
    pub payload: Vec<u8>,
}

impl FromBytes for StatusPacket {
    fn from_bytes(sender_id: u8, bytes: Vec<u8>) -> Result<Self, CommunicationErrorKind> {
        if bytes.len() < 6 {
            return Err(CommunicationErrorKind::ParsingError);
        }

        let read_crc = *bytes.last().unwrap();
        let computed_crc = crc(&bytes[2..bytes.len() - 1]);
        if read_crc != computed_crc {
            return Err(CommunicationErrorKind::ChecksumError);
        }

        if bytes[0] != 255 || bytes[1] != 255 {
            return Err(CommunicationErrorKind::ParsingError);
        }

        let id = bytes[2];
        if id != sender_id {
            return Err(CommunicationErrorKind::ParsingError);
        }

        let payload_length = bytes[3] as usize;
        let error = DynamixelErrorKind::from_byte(bytes[4]);

        if payload_length != bytes.len() - HEADER_SIZE || payload_length < 2 {
            return Err(CommunicationErrorKind::ParsingError);
        }

        let payload = bytes[5..3 + payload_length].to_vec();

        Ok(StatusPacket { id, error, payload })
    }
}

#[derive(Debug)]
pub enum Instruction {
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
