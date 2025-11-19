use crate::Result;

use super::{
    packet::{InstructionPacket, Packet, StatusPacket},
    CommunicationErrorKind, Protocol,
};

const BROADCAST_ID: u8 = 254;

#[derive(Debug)]
pub(crate) struct PacketV1;
impl Packet for PacketV1 {
    const HEADER_SIZE: usize = 4;

    type ErrorKind = DynamixelErrorV1;
    type InstructionKind = InstructionKindV1;

    fn ping_packet(id: u8) -> Box<dyn InstructionPacket<Self>> {
        Box::new(InstructionPacketV1 {
            id,
            instruction: InstructionKindV1::Ping,
            params: vec![],
        })
    }

    fn reboot_packet(id: u8) -> Box<dyn InstructionPacket<Self>> {
        Box::new(InstructionPacketV1 {
            id,
            instruction: InstructionKindV1::Reboot,
            params: vec![],
        })
    }

    fn factory_reset_packet(
        id: u8,
        _conserve_id_only: bool,
        _conserve_id_and_baudrate: bool,
    ) -> Box<dyn InstructionPacket<Self>> {
        Box::new(InstructionPacketV1 {
            id,
            instruction: InstructionKindV1::FactoryReset,
            params: vec![],
        })
    }

    fn read_packet(id: u8, addr: u8, length: u8) -> Box<dyn InstructionPacket<Self>> {
        Box::new(InstructionPacketV1 {
            id,
            instruction: InstructionKindV1::Read,
            params: vec![addr, length],
        })
    }

    fn write_packet(id: u8, addr: u8, data: &[u8]) -> Box<dyn InstructionPacket<Self>> {
        Box::new(InstructionPacketV1 {
            id,
            instruction: InstructionKindV1::Write,
            params: {
                let mut params = vec![addr];
                params.extend(data);
                params
            },
        })
    }

    fn sync_read_packet(ids: &[u8], addr: u8, length: u8) -> Box<dyn InstructionPacket<Self>> {
        Box::new(InstructionPacketV1 {
            id: BROADCAST_ID,
            instruction: InstructionKindV1::SyncRead,
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
        Box::new(InstructionPacketV1 {
            id: BROADCAST_ID,
            instruction: InstructionKindV1::SyncWrite,
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
        Ok(Box::new(StatusPacketV1::from_bytes(data, sender_id)?))
    }
}

#[derive(Debug)]
struct InstructionPacketV1 {
    id: u8,
    instruction: InstructionKindV1,
    params: Vec<u8>,
}
impl InstructionPacket<PacketV1> for InstructionPacketV1 {
    fn id(&self) -> u8 {
        self.id
    }

    fn instruction(&self) -> <PacketV1 as Packet>::InstructionKind {
        self.instruction
    }

    fn params(&self) -> &Vec<u8> {
        &self.params
    }

    fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::new();

        let payload_length: u8 = (self.params.len() + 2).try_into().unwrap();

        bytes.extend([255, 255, self.id, payload_length].iter());
        bytes.push(self.instruction.value());
        bytes.extend(self.params.iter());
        bytes.push(crc(&bytes[2..]));

        bytes
    }
}

#[derive(Debug)]
struct StatusPacketV1 {
    id: u8,
    #[allow(dead_code)]
    errors: Vec<DynamixelErrorV1>,
    params: Vec<u8>,
}

impl StatusPacket<PacketV1> for StatusPacketV1 {
    fn from_bytes(data: &[u8], sender_id: u8) -> Result<Self>
    where
        Self: Sized,
    {
        if data.len() < PacketV1::HEADER_SIZE + 2 {
            return Err(Box::new(CommunicationErrorKind::ParsingError));
        }

        let read_crc = *data.last().unwrap();
        let computed_crc = crc(&data[2..data.len() - 1]);
        if read_crc != computed_crc {
            println!("read crc: {read_crc}, computed crc: {computed_crc} data: {data:?}");

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
        let errors = DynamixelErrorV1::from_byte(data[4]);

        if params_length != data.len() - PacketV1::HEADER_SIZE || params_length < 2 {
            return Err(Box::new(CommunicationErrorKind::ParsingError));
        }

        let params = data[5..3 + params_length].to_vec();

        Ok(StatusPacketV1 { id, errors, params })
    }

    fn id(&self) -> u8 {
        self.id
    }

    fn errors(&self) -> &Vec<<PacketV1 as Packet>::ErrorKind> {
        &self.errors
    }

    fn params(&self) -> &Vec<u8> {
        &self.params
    }
}

#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub(crate) enum DynamixelErrorV1 {
    Instruction,
    Overload,
    Checksum,
    Range,
    Overheating,
    AngleLimit,
    InputVoltage,
}
impl DynamixelErrorV1 {
    fn from_byte(error: u8) -> Vec<Self> {
        (0..7)
            .filter(|i| error & (1 << i) != 0)
            .map(|i| DynamixelErrorV1::from_bit(i).unwrap())
            .collect()
    }
    fn from_bit(b: u8) -> Option<Self> {
        match b {
            6 => Some(DynamixelErrorV1::Instruction),
            5 => Some(DynamixelErrorV1::Overload),
            4 => Some(DynamixelErrorV1::Checksum),
            3 => Some(DynamixelErrorV1::Range),
            2 => Some(DynamixelErrorV1::Overheating),
            1 => Some(DynamixelErrorV1::AngleLimit),
            0 => Some(DynamixelErrorV1::InputVoltage),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub(crate) enum InstructionKindV1 {
    Ping,
    Read,
    Write,
    FactoryReset,
    Reboot,
    SyncWrite,
    SyncRead,
}

impl InstructionKindV1 {
    fn value(&self) -> u8 {
        match self {
            InstructionKindV1::Ping => 0x01,
            InstructionKindV1::Read => 0x02,
            InstructionKindV1::Write => 0x03,
            InstructionKindV1::FactoryReset => 0x06,
            InstructionKindV1::Reboot => 0x08,
            InstructionKindV1::SyncRead => 0x82,
            InstructionKindV1::SyncWrite => 0x83,
        }
    }
}

#[derive(Debug)]
pub(crate) struct V1;
impl Protocol<PacketV1> for V1 {}

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
        let p = PacketV1::ping_packet(1);
        let bytes = p.to_bytes();
        assert_eq!(bytes, [0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB]);
    }

    #[test]
    fn create_reboot_packet() {
        let p = PacketV1::reboot_packet(2);
        let bytes = p.to_bytes();
        assert_eq!(bytes, [0xFF, 0xFF, 0x02, 0x02, 0x08, 0xF3]);
    }

    #[test]
    fn create_read_packet() {
        let p = PacketV1::read_packet(1, 0x2B, 1);
        let bytes = p.to_bytes();
        assert_eq!(bytes, [0xFF, 0xFF, 0x01, 0x04, 0x02, 0x2B, 0x01, 0xCC]);
    }

    #[test]
    fn create_write_packet() {
        let p = PacketV1::write_packet(10, 24, &[1]);
        let bytes = p.to_bytes();
        assert_eq!(bytes, [255, 255, 10, 4, 3, 24, 1, 213]);

        let p = PacketV1::write_packet(0xFE, 0x03, &[1]);
        let bytes = p.to_bytes();
        assert_eq!(bytes, [0xFF, 0xFF, 0xFE, 0x04, 0x03, 0x03, 0x01, 0xF6]);
    }

    #[test]
    fn create_sync_read_packet() {
        let p = PacketV1::sync_read_packet(&[11, 12], 30, 2);
        let bytes = p.to_bytes();
        assert_eq!(
            bytes,
            [0xFF, 0xFF, 0xFE, 0x6, 0x82, 0x1e, 0x2, 0xb, 0xc, 0x42]
        );
    }

    #[test]
    fn create_sync_write_packet() {
        let p = PacketV1::sync_write_packet(&[11, 12], 30, &[vec![0x0, 0x0], vec![0xA, 0x14]]);
        let bytes = p.to_bytes();
        assert_eq!(
            bytes,
            [0xFF, 0xFF, 0xFE, 0xA, 0x83, 0x1E, 0x2, 0xB, 0x0, 0x0, 0xC, 0xA, 0x14, 0x1F]
        );
    }

    #[test]
    fn parse_status_packet() {
        let bytes = vec![0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC];
        let sp = StatusPacketV1::from_bytes(&bytes, 0x01).unwrap();
        assert_eq!(sp.id, 1);
        assert_eq!(sp.errors.len(), 0);
        assert_eq!(sp.params.len(), 0);

        let bytes = vec![0xFF, 0xFF, 0x01, 0x03, 0x00, 0x20, 0xDB];
        let sp = StatusPacketV1::from_bytes(&bytes, 0x01).unwrap();
        assert_eq!(sp.id, 1);
        assert_eq!(sp.errors.len(), 0);
        assert_eq!(sp.params.len(), 1);
        assert_eq!(sp.params[0], 0x20);
    }
    #[test]
    fn check_error_on_wrong_id() {
        let bytes = vec![0xFF, 0xFF, 0x01, 0x03, 0x00, 0x20, 0xDB];
        let sp = StatusPacketV1::from_bytes(&bytes, 0x02);
        assert!(sp.is_err());
    }
}
