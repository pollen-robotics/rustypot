use crate::{
    packet::{InstructionPacket, StatusPacket},
    Protocol, Result,
};

use super::{CommunicationErrorKind, Packet};

const BROADCAST_ID: u8 = 254;

#[derive(Debug)]
pub struct PacketFeetech;
impl Packet for PacketFeetech {
    const HEADER_SIZE: usize = 4;

    type ErrorKind = DynamixelErrorFeetech;
    type InstructionKind = InstructionKindFeetech;

    fn ping_packet(id: u8) -> Box<dyn InstructionPacket<Self>> {
        Box::new(InstructionPacketFeetech {
            id,
            instruction: InstructionKindFeetech::Ping,
            params: vec![],
        })
    }

    fn read_packet(id: u8, addr: u8, length: u8) -> Box<dyn InstructionPacket<Self>> {
        Box::new(InstructionPacketFeetech {
            id,
            instruction: InstructionKindFeetech::Read,
            params: vec![addr, length],
        })
    }

    fn write_packet(id: u8, addr: u8, data: &[u8]) -> Box<dyn InstructionPacket<Self>> {
        Box::new(InstructionPacketFeetech {
            id,
            instruction: InstructionKindFeetech::Write,
            params: {
                let mut params = vec![addr];
                params.extend(data);
                params
            },
        })
    }

    fn sync_read_packet(ids: &[u8], addr: u8, length: u8) -> Box<dyn InstructionPacket<Self>> {
        Box::new(InstructionPacketFeetech {
            id: BROADCAST_ID,
            instruction: InstructionKindFeetech::SyncRead,
            params: {
                let mut params = vec![addr, length];
                params.extend(ids);
                params
            },
        })
    }

    fn sync_write_packet(
        ids: &[u8],
        addr: u8,
        data: &[Vec<u8>],
    ) -> Box<dyn InstructionPacket<Self>> {
        Box::new(InstructionPacketFeetech {
            id: BROADCAST_ID,
            instruction: InstructionKindFeetech::SyncWrite,
            params: {
                let mut params = vec![addr];
                let values: Vec<u8> = ids
                    .iter()
                    .zip(data.iter())
                    .flat_map(|(&id, val)| {
                        let mut v = vec![id];
                        v.extend(val);
                        v
                    })
                    .collect();
                params.push((values.len() / ids.len() - 1).try_into().unwrap());
                params.extend(values);
                params
            },
        })
    }

    fn get_payload_size(header: &[u8]) -> Result<usize> {
        if header.len() == 4 && header[0] == 255 && header[1] == 255 {
            Ok(header[3].into())
        } else {
            Err(Box::new(CommunicationErrorKind::ParsingError))
        }
    }

    fn status_packet(data: &[u8], sender_id: u8) -> Result<Box<dyn StatusPacket<Self>>> {
        Ok(Box::new(StatusPacketFeetech::from_bytes(data, sender_id)?))
    }
}

#[derive(Debug)]
struct InstructionPacketFeetech {
    id: u8,
    instruction: InstructionKindFeetech,
    params: Vec<u8>,
}
impl InstructionPacket<PacketFeetech> for InstructionPacketFeetech {
    fn id(&self) -> u8 {
        self.id
    }

    fn instruction(&self) -> <PacketFeetech as Packet>::InstructionKind {
        self.instruction
    }

    fn params(&self) -> &Vec<u8> {
        &self.params
    }

    fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::new();

        let payload_length: u8 = (self.params.len() + 2).try_into().unwrap();

        bytes.extend(vec![255, 255, self.id, payload_length].iter());
        bytes.push(self.instruction.value());
        bytes.extend(self.params.iter());
        bytes.push(crc(&bytes[2..]));

        bytes
    }
}

#[derive(Debug)]
struct StatusPacketFeetech {
    id: u8,
    #[allow(dead_code)]
    errors: Vec<DynamixelErrorFeetech>,
    params: Vec<u8>,
}

impl StatusPacket<PacketFeetech> for StatusPacketFeetech {
    fn from_bytes(data: &[u8], sender_id: u8) -> Result<Self>
    where
        Self: Sized,
    {
        if data.len() < PacketFeetech::HEADER_SIZE + 2 {
            return Err(Box::new(CommunicationErrorKind::ParsingError));
        }

        let read_crc = *data.last().unwrap();
        let computed_crc = crc(&data[2..data.len() - 1]);
        if read_crc != computed_crc {
            println!(
                "read crc: {}, computed crc: {} data: {:?}",
                read_crc, computed_crc, data
            );

            return Err(Box::new(CommunicationErrorKind::ChecksumError));
        }

        // This should already have been catched when parsing the header
        assert_eq!(data[0], 255);
        assert_eq!(data[1], 255);

        let id = data[2];
        if id != sender_id {
            return Err(Box::new(CommunicationErrorKind::ParsingError));
        }

        let params_length = data[3] as usize;
        let errors = DynamixelErrorFeetech::from_byte(data[4]);

        if params_length != data.len() - PacketFeetech::HEADER_SIZE || params_length < 2 {
            return Err(Box::new(CommunicationErrorKind::ParsingError));
        }

        let params = data[5..3 + params_length].to_vec();

        Ok(StatusPacketFeetech { id, errors, params })
    }

    fn id(&self) -> u8 {
        self.id
    }

    fn errors(&self) -> &Vec<<PacketFeetech as Packet>::ErrorKind> {
        &self.errors
    }

    fn params(&self) -> &Vec<u8> {
        &self.params
    }
}

#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub enum DynamixelErrorFeetech {
    Instruction,
    Overload,
    Checksum,
    Range,
    Overheating,
    AngleLimit,
    InputVoltage,
}
impl DynamixelErrorFeetech {
    fn from_byte(error: u8) -> Vec<Self> {
        (0..7)
            .filter(|i| error & (1 << i) != 0)
            .map(|i| DynamixelErrorFeetech::from_bit(i).unwrap())
            .collect()
    }
    fn from_bit(b: u8) -> Option<Self> {
        match b {
            6 => Some(DynamixelErrorFeetech::Instruction),
            5 => Some(DynamixelErrorFeetech::Overload),
            4 => Some(DynamixelErrorFeetech::Checksum),
            3 => Some(DynamixelErrorFeetech::Range),
            2 => Some(DynamixelErrorFeetech::Overheating),
            1 => Some(DynamixelErrorFeetech::AngleLimit),
            0 => Some(DynamixelErrorFeetech::InputVoltage),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum InstructionKindFeetech {
    Ping,
    Read,
    Write,
    SyncWrite,
    SyncRead,
}

impl InstructionKindFeetech {
    fn value(&self) -> u8 {
        match self {
            InstructionKindFeetech::Ping => 0x01,
            InstructionKindFeetech::Read => 0x02,
            InstructionKindFeetech::Write => 0x03,
            InstructionKindFeetech::SyncWrite => 0x83,
            InstructionKindFeetech::SyncRead => 0x84,
        }
    }
}

#[derive(Debug)]
pub struct Feetech;
impl Protocol<PacketFeetech> for Feetech {
    fn new() -> Self {
        Feetech
    }
    fn sync_read(
        &self,
        port: &mut dyn serialport::SerialPort,
        ids: &[u8],
        addr: u8,
        length: u8,
    ) -> Result<Vec<Vec<u8>>> {
        let instruction_packet = PacketFeetech::sync_read_packet(ids, addr, length);
        self.send_instruction_packet(port, instruction_packet.as_ref())?;

        let mut result = Vec::new();

        for &id in ids {
            let status_packet = self.read_status_packet(port, id)?;
            result.push(status_packet.params().clone());
        }

        Ok(result)
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
        let p = PacketFeetech::ping_packet(1);
        let bytes = p.to_bytes();
        assert_eq!(bytes, [0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB]);
    }

    #[test]
    fn create_read_packet() {
        let p = PacketFeetech::read_packet(1, 0x2B, 1);
        let bytes = p.to_bytes();
        assert_eq!(bytes, [0xFF, 0xFF, 0x01, 0x04, 0x02, 0x2B, 0x01, 0xCC]);
    }

    #[test]
    fn create_write_packet() {
        let p = PacketFeetech::write_packet(10, 24, &[1]);
        let bytes = p.to_bytes();
        assert_eq!(bytes, [255, 255, 10, 4, 3, 24, 1, 213]);

        let p = PacketFeetech::write_packet(0xFE, 0x03, &[1]);
        let bytes = p.to_bytes();
        assert_eq!(bytes, [0xFF, 0xFF, 0xFE, 0x04, 0x03, 0x03, 0x01, 0xF6]);
    }

    #[test]
    fn create_sync_read_packet() {
        let p = PacketFeetech::sync_read_packet(&[11, 12], 30, 2);
        let bytes = p.to_bytes();
        assert_eq!(
            bytes,
            [0xFF, 0xFF, 0xFE, 0x6, 0x84, 0x1e, 0x2, 0xb, 0xc, 0x40]
        );
    }

    #[test]
    fn create_sync_write_packet() {
        let p = PacketFeetech::sync_write_packet(&[11, 12], 30, &[vec![0x0, 0x0], vec![0xA, 0x14]]);
        let bytes = p.to_bytes();
        assert_eq!(
            bytes,
            [0xFF, 0xFF, 0xFE, 0xA, 0x83, 0x1E, 0x2, 0xB, 0x0, 0x0, 0xC, 0xA, 0x14, 0x1F]
        );
    }

    #[test]
    fn parse_status_packet() {
        let bytes = vec![0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC];
        let sp = StatusPacketFeetech::from_bytes(&bytes, 0x01).unwrap();
        assert_eq!(sp.id, 1);
        assert_eq!(sp.errors.len(), 0);
        assert_eq!(sp.params.len(), 0);

        let bytes = vec![0xFF, 0xFF, 0x01, 0x03, 0x00, 0x20, 0xDB];
        let sp = StatusPacketFeetech::from_bytes(&bytes, 0x01).unwrap();
        assert_eq!(sp.id, 1);
        assert_eq!(sp.errors.len(), 0);
        assert_eq!(sp.params.len(), 1);
        assert_eq!(sp.params[0], 0x20);
    }
    #[test]
    fn check_error_on_wrong_id() {
        let bytes = vec![0xFF, 0xFF, 0x01, 0x03, 0x00, 0x20, 0xDB];
        let sp = StatusPacketFeetech::from_bytes(&bytes, 0x02);
        assert!(sp.is_err());
    }
}
