use crate::Result;

use super::{
    packet::{InstructionPacket, Packet, StatusPacket},
    CommunicationErrorKind, Protocol,
};

#[derive(Debug)]
pub(crate) struct V2;
impl Protocol<PacketV2> for V2 {}

#[derive(Debug)]
pub(crate) struct PacketV2;
impl Packet for PacketV2 {
    const HEADER_SIZE: usize = 7;

    type ErrorKind = DynamixelErrorV2;
    type InstructionKind = InstructionKindV2;

    fn get_payload_size(header: &[u8]) -> Result<usize> {
        assert_eq!(header.len(), Self::HEADER_SIZE);

        if (header[0] != 0xFF) || (header[1] != 0xFF) || (header[2] != 0xFD) || (header[3] != 0x00)
        {
            return Err(Box::new(CommunicationErrorKind::ParsingError));
        }

        let payload_size: [u8; 2] = header[5..7].try_into().unwrap();
        let payload_size = u16::from_le_bytes(payload_size);

        Ok(payload_size as usize)
    }

    fn ping_packet(id: u8) -> Box<dyn InstructionPacket<Self>> {
        Box::new(InstructionPacketV2 {
            id,
            instruction: InstructionKindV2::Ping,
            params: vec![],
        })
    }

    fn reboot_packet(id: u8) -> Box<dyn InstructionPacket<Self>> {
        Box::new(InstructionPacketV2 {
            id,
            instruction: InstructionKindV2::Reboot,
            params: vec![],
        })
    }

    fn factory_reset_packet(
        id: u8,
        conserve_id_only: bool,
        conserve_id_and_baudrate: bool,
    ) -> Box<dyn InstructionPacket<Self>> {
        // See https://emanual.robotis.com/docs/en/dxl/protocol2/
        let param = match (conserve_id_only, conserve_id_and_baudrate) {
            (false, false) => 0xFF,
            (true, false) => 0x01,
            (true, true) => 0x02,
            (false, true) => 0x02, // Same as (true, true)
        };

        Box::new(InstructionPacketV2 {
            id,
            instruction: InstructionKindV2::FactoryReset,
            params: vec![param],
        })
    }

    fn read_packet(id: u8, addr: u8, length: u8) -> Box<dyn InstructionPacket<Self>> {
        Box::new(InstructionPacketV2 {
            id,
            instruction: InstructionKindV2::Read,
            params: {
                let mut params = Vec::new();
                params.extend((addr as u16).to_le_bytes());
                params.extend((length as u16).to_le_bytes());
                params
            },
        })
    }

    fn write_packet(id: u8, addr: u8, data: &[u8]) -> Box<dyn InstructionPacket<Self>> {
        Box::new(InstructionPacketV2 {
            id,
            instruction: InstructionKindV2::Write,
            params: {
                let mut params = Vec::new();
                params.extend((addr as u16).to_le_bytes());
                params.extend(data);
                params
            },
        })
    }

    fn sync_read_packet(ids: &[u8], addr: u8, length: u8) -> Box<dyn InstructionPacket<Self>> {
        Box::new(InstructionPacketV2 {
            id: BROADCAST_ID,
            instruction: InstructionKindV2::SyncRead,
            params: {
                let mut params = Vec::new();
                params.extend((addr as u16).to_le_bytes());
                params.extend((length as u16).to_le_bytes());
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
        Box::new(InstructionPacketV2 {
            id: BROADCAST_ID,
            instruction: InstructionKindV2::SyncWrite,
            params: {
                let mut params = Vec::new();
                params.extend((addr as u16).to_le_bytes());
                params.extend((data[0].len() as u16).to_le_bytes());

                for (&id, value) in ids.iter().zip(data) {
                    params.push(id);
                    params.extend(value);
                }

                params
            },
        })
    }

    fn status_packet(data: &[u8], sender_id: u8) -> Result<Box<dyn StatusPacket<Self>>> {
        Ok(Box::new(StatusPacketV2::from_bytes(data, sender_id)?))
    }
}

#[derive(Debug)]
struct InstructionPacketV2 {
    id: u8,
    instruction: InstructionKindV2,
    params: Vec<u8>,
}
impl InstructionPacket<PacketV2> for InstructionPacketV2 {
    fn id(&self) -> u8 {
        self.id
    }

    fn instruction(&self) -> <PacketV2 as Packet>::InstructionKind {
        self.instruction
    }

    fn params(&self) -> &Vec<u8> {
        &self.params
    }

    fn to_bytes(&self) -> Vec<u8> {
        // 0xFF	0xFF 0xFD 0x00 ID Len_L Len_H Instruction Param 1 … Param N CRC_L CRC_H
        let mut bytes = vec![0xFF, 0xFF, 0xFD, 0x00];

        bytes.push(self.id());

        let nb_params = self.params.len() as u16 + 3;
        bytes.extend(nb_params.to_le_bytes());

        bytes.push(self.instruction().value());

        bytes.extend(self.params());

        bytes.extend(crc(&bytes).to_le_bytes());

        bytes
    }
}

#[derive(Debug)]
struct StatusPacketV2 {
    id: u8,
    errors: Vec<DynamixelErrorV2>,
    params: Vec<u8>,
}

impl StatusPacket<PacketV2> for StatusPacketV2 {
    fn from_bytes(data: &[u8], sender_id: u8) -> Result<Self>
    where
        Self: Sized,
    {
        let msg_length = data.len();

        if msg_length < PacketV2::HEADER_SIZE + 3 {
            return Err(Box::new(CommunicationErrorKind::ParsingError));
        }

        let read_crc = u16::from_le_bytes(data[msg_length - 2..].try_into().unwrap());
        let computed_crc = crc(&data[..data.len() - 2]);
        if read_crc != computed_crc {
            return Err(Box::new(CommunicationErrorKind::ChecksumError));
        }

        // This should already have been catched when parsing the header
        assert_eq!(data[0], 0xFF);
        assert_eq!(data[1], 0xFF);
        assert_eq!(data[2], 0xFD);
        assert_eq!(data[3], 0x00);

        let id = data[4];
        if id != sender_id {
            return Err(Box::new(CommunicationErrorKind::IncorrectId(id, sender_id)));
        }

        let payload_length = u16::from_le_bytes(data[5..7].try_into().unwrap()) as usize;
        if data[7] != 0x55 {
            return Err(Box::new(CommunicationErrorKind::ParsingError));
        }
        let errors = DynamixelErrorV2::from_byte(data[8]);

        if payload_length != data.len() - PacketV2::HEADER_SIZE || payload_length < 4 {
            return Err(Box::new(CommunicationErrorKind::ParsingError));
        }

        let params = data[9..msg_length - 2].to_vec();
        assert_eq!(params.len(), payload_length - 4);

        Ok(StatusPacketV2 { id, errors, params })
    }

    fn id(&self) -> u8 {
        self.id
    }

    fn errors(&self) -> &Vec<<PacketV2 as Packet>::ErrorKind> {
        &self.errors
    }

    fn params(&self) -> &Vec<u8> {
        &self.params
    }
}

#[derive(Debug, Clone, Copy)]
pub(crate) enum InstructionKindV2 {
    Ping,
    Read,
    Write,
    FactoryReset,
    Reboot,
    SyncRead,
    SyncWrite,
}

impl InstructionKindV2 {
    fn value(&self) -> u8 {
        match self {
            InstructionKindV2::Ping => 0x01,
            InstructionKindV2::Read => 0x02,
            InstructionKindV2::Write => 0x03,
            InstructionKindV2::FactoryReset => 0x06,
            InstructionKindV2::Reboot => 0x08,
            InstructionKindV2::SyncRead => 0x82,
            InstructionKindV2::SyncWrite => 0x83,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub(crate) enum DynamixelErrorV2 {
    ResultFail,
    Instruction,
    Checksum,
    Range,
    Length,
    Limit,
    Access,
}

impl DynamixelErrorV2 {
    fn from_byte(error: u8) -> Vec<Self> {
        (1..7)
            .filter(|i| error & (1 << i) != 0)
            .map(|i| DynamixelErrorV2::from_bit(i).unwrap())
            .collect()
    }
    fn from_bit(b: u8) -> Option<Self> {
        match b {
            1 => Some(DynamixelErrorV2::Access),
            2 => Some(DynamixelErrorV2::Limit),
            3 => Some(DynamixelErrorV2::Length),
            4 => Some(DynamixelErrorV2::Range),
            5 => Some(DynamixelErrorV2::Checksum),
            6 => Some(DynamixelErrorV2::Instruction),
            7 => Some(DynamixelErrorV2::ResultFail),
            _ => None,
        }
    }
}

fn crc(data: &[u8]) -> u16 {
    let mut crc_accum: u16 = 0;

    for byte in data {
        let i: u8 = (crc_accum >> 8) as u8 ^ byte;
        crc_accum = (crc_accum << 8) ^ CRC_TABLE[i as usize];
    }

    crc_accum
}

const BROADCAST_ID: u8 = 0xFE;
const CRC_TABLE: [u16; 256] = [
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011, 0x8033, 0x0036, 0x003C, 0x8039,
    0x0028, 0x802D, 0x8027, 0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
    0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093, 0x0096, 0x009C, 0x8099,
    0x0088, 0x808D, 0x8087, 0x0082, 0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA,
    0x81FB, 0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176, 0x017C, 0x8179,
    0x0168, 0x816D, 0x8167, 0x0162, 0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101, 0x8303, 0x0306, 0x030C, 0x8309,
    0x0318, 0x831D, 0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C, 0x8359,
    0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2, 0x83A3, 0x03A6, 0x03AC, 0x83A9,
    0x03B8, 0x83BD, 0x83B7, 0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
    0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243, 0x0246, 0x024C, 0x8249,
    0x0258, 0x825D, 0x8257, 0x0252, 0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219,
    0x0208, 0x820D, 0x8207, 0x0202,
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc() {
        let data = vec![0xFF, 0xFF, 0xFD, 0x00, 0x2a, 0x3, 0x0, 0x1];
        let crc = crc(&data);

        assert_eq!(crc.to_le_bytes(), [0x16, 0xd2]);
    }

    #[test]
    fn create_ping_packet() {
        let p = PacketV2::ping_packet(2);
        let bytes = p.to_bytes();
        assert_eq!(
            bytes,
            [0xff, 0xff, 0xfd, 0x0, 0x2, 0x3, 0x0, 0x1, 0x19, 0x72]
        );
    }

    #[test]
    fn create_reboot_packet() {
        let p = PacketV2::reboot_packet(2);
        let bytes = p.to_bytes();
        assert_eq!(
            bytes,
            [0xff, 0xff, 0xfd, 0x0, 0x2, 0x3, 0x0, 0x8, 0x2f, 0x72]
        );
    }

    #[test]
    fn create_read_packet() {
        let p = PacketV2::read_packet(1, 0x2B, 2);
        let bytes = p.to_bytes();
        assert_eq!(
            bytes,
            [0xff, 0xff, 0xfd, 0x0, 0x1, 0x7, 0x0, 0x2, 0x2b, 0x0, 0x2, 0x0, 0x2e, 0xcd]
        );
    }

    #[test]
    fn create_write_packet() {
        let p = PacketV2::write_packet(1, 116, &512_u32.to_le_bytes());
        let bytes = p.to_bytes();
        assert_eq!(
            bytes,
            [
                0xFF, 0xFF, 0xFD, 0x0, 0x1, 0x9, 0x0, 0x03, 0x74, 0x00, 0x00, 0x02, 0x00, 0x00,
                0xCA, 0x89
            ]
        );
    }

    #[test]
    fn create_sync_read_packet() {
        let p = PacketV2::sync_read_packet(&[1, 2], 132, 4);
        let bytes = p.to_bytes();
        assert_eq!(
            bytes,
            [
                0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x09, 0x00, 0x82, 0x84, 0x00, 0x04, 0x00, 0x01, 0x02,
                0xCE, 0xFA
            ]
        );
    }

    #[test]
    fn create_sync_write_packet() {
        let p = PacketV2::sync_write_packet(
            &[1, 2],
            116,
            &[
                150_u32.to_le_bytes().to_vec(),
                170_u32.to_le_bytes().to_vec(),
            ],
        );
        let bytes = p.to_bytes();
        assert_eq!(
            bytes,
            [
                0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x11, 0x00, 0x83, 0x74, 0x00, 0x04, 0x00, 0x01, 0x96,
                0x00, 0x00, 0x00, 0x02, 0xAA, 0x00, 0x00, 0x00, 0x82, 0x87
            ]
        );
    }

    #[test]
    fn parse_status_packet() {
        let bytes = vec![
            0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x08, 0x00, 0x55, 0x00, 0xA6, 0x00, 0x00, 0x00, 0x8C,
            0xC0,
        ];

        let sp = StatusPacketV2::from_bytes(&bytes, 0x01).unwrap();
        assert_eq!(sp.id, 1);
        assert_eq!(sp.errors.len(), 0);
        assert_eq!(sp.params.len(), 4);
        assert_eq!(sp.params, [0xA6, 0x00, 0x00, 0x00])
    }
}
