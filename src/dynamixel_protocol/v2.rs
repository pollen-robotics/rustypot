use serialport::SerialPort;

use crate::Result;

use super::{
    packet::{InstructionPacket, Packet, StatusPacket},
    CommunicationErrorKind, Protocol,
};

#[derive(Debug)]
pub(crate) struct V2;
impl Protocol<PacketV2> for V2 {}

impl V2 {
    /// Fast Sync Read (instruction 0x8A).
    ///
    /// The instruction packet is the same as Sync Read's, but instead of each motor
    /// answering with its own status packet, every motor appends its answer to a single
    /// status packet sent from the broadcast id. That saves one packet header plus one
    /// bus turnaround (and its return delay time) per motor.
    ///
    /// Only supported by firmware new enough to implement it (XL330: v46+); older
    /// firmware does not answer and the read times out.
    pub(crate) fn fast_sync_read(
        &self,
        port: &mut dyn SerialPort,
        ids: &[u8],
        addr: u8,
        length: u8,
    ) -> Result<Vec<Vec<u8>>> {
        self.send_instruction_packet(
            port,
            PacketV2::fast_sync_read_packet(ids, addr, length).as_ref(),
        )?;
        let data = self.read_status_packet_bytes(port)?;
        parse_fast_sync_read_status(&data, ids, length)
    }
}

impl PacketV2 {
    /// Same parameters as [`PacketV2::sync_read_packet`], only the instruction differs.
    fn fast_sync_read_packet(
        ids: &[u8],
        addr: u8,
        length: u8,
    ) -> Box<dyn InstructionPacket<PacketV2>> {
        Box::new(InstructionPacketV2 {
            id: BROADCAST_ID,
            instruction: InstructionKindV2::FastSyncRead,
            params: {
                let mut params = Vec::new();
                params.extend((addr as u16).to_le_bytes());
                params.extend((length as u16).to_le_bytes());
                params.extend(ids);
                params
            },
        })
    }
}

/// Split the single status packet answering a Fast Sync Read into one data slice per id.
///
/// After the usual `FF FF FD 00 FE LEN_L LEN_H 0x55` header, the body is one fixed size
/// block per requested id, in the order they were requested:
///
/// ```text
/// [ERROR ID DATA(length) CRC_L CRC_H] x nb_ids
/// ```
///
/// The CRC a motor appends is the CRC accumulated over the whole packet up to and
/// including its own block, so the last one is also the CRC of the complete packet.
/// Checking every block's CRC therefore validates the packet *and* tells us the blocks
/// sit where we think they do.
///
/// Note: unlike a regular status packet, the body is not de-stuffed. The official SDK
/// reads these packets with stuffing removal explicitly skipped
/// (`GroupFastSyncRead::rxPacket` calls `rxPacket(.., skip_stuffing = true)`) and walks
/// the body with a fixed stride, i.e. it assumes motors do not insert stuffing bytes
/// here. We do the same, but the per block CRC check above means a stuffed byte would
/// be reported as a checksum error rather than silently shifting the data.
fn parse_fast_sync_read_status(data: &[u8], ids: &[u8], length: u8) -> Result<Vec<Vec<u8>>> {
    // Header + the 0x55 marking a status packet
    const BODY_START: usize = PacketV2::HEADER_SIZE + 1;
    // ERROR + ID + DATA + CRC16
    let block_size = length as usize + 4;

    if data.len() != BODY_START + ids.len() * block_size {
        return Err(Box::new(CommunicationErrorKind::ParsingError));
    }
    if data[4] != BROADCAST_ID || data[7] != 0x55 {
        return Err(Box::new(CommunicationErrorKind::ParsingError));
    }
    let payload_length = u16::from_le_bytes(data[5..7].try_into().unwrap()) as usize;
    if payload_length != data.len() - PacketV2::HEADER_SIZE {
        return Err(Box::new(CommunicationErrorKind::ParsingError));
    }

    let mut values = Vec::with_capacity(ids.len());
    for (i, &id) in ids.iter().enumerate() {
        let block = BODY_START + i * block_size;
        let crc_at = block + 2 + length as usize;

        let read_crc = u16::from_le_bytes(data[crc_at..crc_at + 2].try_into().unwrap());
        if read_crc != crc(&data[..crc_at]) {
            return Err(Box::new(CommunicationErrorKind::ChecksumError));
        }
        if data[block + 1] != id {
            return Err(Box::new(CommunicationErrorKind::IncorrectId(
                id,
                data[block + 1],
            )));
        }

        values.push(data[block + 2..crc_at].to_vec());
    }

    Ok(values)
}

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

/// Insert protocol 2.0 byte stuffing: whenever the pattern 0xFF 0xFF 0xFD
/// appears in the packet body, an extra 0xFD is added right after it so the
/// data can never be mistaken for a packet header on the wire. The length
/// field must count the inserted bytes. Mirrors the official DynamixelSDK
/// `addStuffing` (the pattern scan runs over the original data only, so an
/// inserted 0xFD never seeds a new match).
/// See https://emanual.robotis.com/docs/en/dxl/protocol2/#packet-processing
fn add_stuffing(data: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(data.len());
    // Length of the FF FF FD prefix matched so far
    let mut run = 0u8;
    for &b in data {
        out.push(b);
        run = match (run, b) {
            (0, 0xFF) | (1, 0xFF) => run + 1,
            (2, 0xFF) => 2, // FF FF FF keeps an FF FF suffix alive
            (2, 0xFD) => {
                out.push(0xFD); // stuffing byte
                0
            }
            (_, 0xFF) => 1,
            _ => 0,
        };
    }
    out
}

/// Remove protocol 2.0 byte stuffing: drop the 0xFD the device inserted after
/// each 0xFF 0xFF 0xFD in the packet body. The received length field (and the
/// CRC) covers the stuffed bytes, so this runs after CRC validation, on the
/// body only. Mirrors the official DynamixelSDK `removeStuffing` (the pattern
/// scan restarts after a removed byte, so FF FF FD FD FD → FF FF FD FD).
fn remove_stuffing(data: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(data.len());
    // Length of the FF FF FD prefix matched so far
    let mut run = 0u8;
    for &b in data {
        if run == 3 {
            run = 0;
            if b == 0xFD {
                continue; // stuffing byte inserted by the sender — drop it
            }
        }
        run = match (run, b) {
            (0, 0xFF) | (1, 0xFF) => run + 1,
            (2, 0xFF) => 2, // FF FF FF keeps an FF FF suffix alive
            (2, 0xFD) => 3,
            (_, 0xFF) => 1,
            _ => 0,
        };
        out.push(b);
    }
    out
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

        // Params containing FF FF FD must be byte-stuffed, and the length
        // field counts the stuffed bytes. (No instruction value is 0xFF, so a
        // pattern can never straddle the instruction/params boundary.)
        let params = add_stuffing(&self.params);

        let nb_params = params.len() as u16 + 3;
        bytes.extend(nb_params.to_le_bytes());

        bytes.push(self.instruction().value());

        bytes.extend(&params);

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

        if payload_length != data.len() - PacketV2::HEADER_SIZE || payload_length < 4 {
            return Err(Box::new(CommunicationErrorKind::ParsingError));
        }

        // The body (error byte + params, i.e. everything between the
        // instruction byte and the CRC) arrives byte-stuffed: the device
        // inserts 0xFD after any FF FF FD, and the length field and CRC cover
        // the stuffed form — so de-stuff only now, after those checks.
        let body = remove_stuffing(&data[8..msg_length - 2]);
        let errors = DynamixelErrorV2::from_byte(body[0]);
        let params = body[1..].to_vec();

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
    FastSyncRead,
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
            InstructionKindV2::FastSyncRead => 0x8A,
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
    fn stuffing_roundtrip() {
        // No pattern → untouched
        assert_eq!(add_stuffing(&[1, 2, 0xFF, 0xFD, 3]), [1, 2, 0xFF, 0xFD, 3]);
        assert_eq!(
            remove_stuffing(&[1, 2, 0xFF, 0xFD, 3]),
            [1, 2, 0xFF, 0xFD, 3]
        );

        // FF FF FD gets an extra FD, and back
        assert_eq!(
            add_stuffing(&[0xFF, 0xFF, 0xFD, 7]),
            [0xFF, 0xFF, 0xFD, 0xFD, 7]
        );
        assert_eq!(
            remove_stuffing(&[0xFF, 0xFF, 0xFD, 0xFD, 7]),
            [0xFF, 0xFF, 0xFD, 7]
        );

        // The scan restarts after a stuffed byte: FF FF FD FD → FF FF FD FD* FD
        assert_eq!(
            add_stuffing(&[0xFF, 0xFF, 0xFD, 0xFD]),
            [0xFF, 0xFF, 0xFD, 0xFD, 0xFD]
        );
        assert_eq!(
            remove_stuffing(&[0xFF, 0xFF, 0xFD, 0xFD, 0xFD]),
            [0xFF, 0xFF, 0xFD, 0xFD]
        );

        // FF FF FF FD: the FF FF suffix stays alive across extra FFs
        assert_eq!(
            add_stuffing(&[0xFF, 0xFF, 0xFF, 0xFD]),
            [0xFF, 0xFF, 0xFF, 0xFD, 0xFD]
        );
        assert_eq!(
            remove_stuffing(&[0xFF, 0xFF, 0xFF, 0xFD, 0xFD]),
            [0xFF, 0xFF, 0xFF, 0xFD]
        );

        // Multiple patterns, and full round-trip
        let data = [0x10, 0xFF, 0xFF, 0xFD, 0x00, 0xFF, 0xFF, 0xFD, 0x20];
        assert_eq!(remove_stuffing(&add_stuffing(&data)), data);
    }

    #[test]
    fn create_write_packet_with_stuffing() {
        // Data containing FF FF FD must be stuffed on the wire and the length
        // field must count the extra byte (7 params -> nb_params = 10).
        let p = PacketV2::write_packet(1, 116, &[0xFF, 0xFF, 0xFD, 0x00]);
        let bytes = p.to_bytes();
        assert_eq!(bytes[5..7], [0x0A, 0x00]);
        assert_eq!(
            bytes[8..14],
            [0x74, 0x00, 0xFF, 0xFF, 0xFD, 0xFD],
            "stuffing byte missing after FF FF FD"
        );
    }

    #[test]
    fn parse_status_packet_with_stuffing() {
        // A 4-byte read whose data contains FF FF FD arrives as 5 wire bytes
        // (stuffed), with the length field and CRC covering the stuffed form.
        // This happens in practice e.g. when present current = -1 (FF FF) is
        // followed by a velocity byte of 0xFD in a bulk read.
        let mut bytes = vec![
            0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x09, 0x00, 0x55, 0x00, 0xFF, 0xFF, 0xFD, 0xFD, 0xA6,
        ];
        bytes.extend(crc(&bytes).to_le_bytes());

        let sp = StatusPacketV2::from_bytes(&bytes, 0x01).unwrap();
        assert_eq!(sp.id, 1);
        assert_eq!(sp.errors.len(), 0);
        assert_eq!(sp.params, [0xFF, 0xFF, 0xFD, 0xA6]);
    }

    #[test]
    fn create_fast_sync_read_packet() {
        // Same bytes as a sync read, with instruction 0x82 -> 0x8A.
        let p = PacketV2::fast_sync_read_packet(&[1, 2], 132, 4);
        let bytes = p.to_bytes();
        assert_eq!(bytes[7], 0x8A);
        assert_eq!(bytes[..7], [0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x09, 0x00]);
        assert_eq!(bytes[8..12], [0x84, 0x00, 0x04, 0x00]);
        assert_eq!(bytes[12..14], [0x01, 0x02]);
        assert_eq!(crc(&bytes[..bytes.len() - 2]).to_le_bytes(), bytes[14..]);
    }

    /// Example status packet from the protocol 2.0 e-manual: ids 3, 7 and 4 answering a
    /// fast sync read of present position (addr 132, 4 bytes).
    /// <https://docs.robotis.com/docs/dxl/protocol/protocol2/#fast-sync-read-0x8a>
    const FAST_SYNC_READ_STATUS: [u8; 32] = [
        0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x19, 0x00, 0x55, //
        0x00, 0x03, 0xA6, 0x00, 0x00, 0x00, 0x84, 0x08, //
        0x00, 0x07, 0x1F, 0x08, 0x00, 0x00, 0x16, 0xCA, //
        0x00, 0x04, 0xFF, 0x03, 0x00, 0x00, 0xD1, 0x9E,
    ];

    #[test]
    fn parse_fast_sync_read_status_packet() {
        let values = parse_fast_sync_read_status(&FAST_SYNC_READ_STATUS, &[3, 7, 4], 4).unwrap();

        assert_eq!(values.len(), 3);
        assert_eq!(values[0], [0xA6, 0x00, 0x00, 0x00]); // id 3: 166
        assert_eq!(values[1], [0x1F, 0x08, 0x00, 0x00]); // id 7: 2079
        assert_eq!(values[2], [0xFF, 0x03, 0x00, 0x00]); // id 4: 1023
    }

    #[test]
    fn fast_sync_read_blocks_carry_a_running_crc() {
        // Every block ends with the CRC of the packet up to that point, so the last one
        // is the CRC of the whole packet. This is what lets us check block alignment.
        for (block_end, expected) in [(14, [0x84, 0x08]), (22, [0x16, 0xCA]), (30, [0xD1, 0x9E])] {
            assert_eq!(
                crc(&FAST_SYNC_READ_STATUS[..block_end]).to_le_bytes(),
                expected
            );
        }
    }

    #[test]
    fn reject_corrupted_fast_sync_read_status_packet() {
        // A wrong number of ids for that packet size
        assert!(parse_fast_sync_read_status(&FAST_SYNC_READ_STATUS, &[3, 7], 4).is_err());
        // Ids answering in an order we did not ask for
        assert!(parse_fast_sync_read_status(&FAST_SYNC_READ_STATUS, &[3, 4, 7], 4).is_err());

        // A flipped data byte breaks that block's CRC (and every one after it)
        let mut corrupted = FAST_SYNC_READ_STATUS;
        corrupted[10] ^= 0x01;
        assert!(parse_fast_sync_read_status(&corrupted, &[3, 7, 4], 4).is_err());

        // A missing motor: the packet is one block short of what we asked for
        assert!(parse_fast_sync_read_status(&FAST_SYNC_READ_STATUS[..24], &[3, 7, 4], 4).is_err());
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
