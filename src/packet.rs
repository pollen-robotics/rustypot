use crate::Result;

pub trait Packet {
    const HEADER_SIZE: usize;
    type ErrorKind;
    type InstructionKind;

    fn get_payload_size(header: &[u8]) -> Result<usize>;

    fn ping_packet(id: u8) -> Box<dyn InstructionPacket<Self>>;

    fn read_packet(id: u8, addr: u8, length: u8) -> Box<dyn InstructionPacket<Self>>;
    fn write_packet(id: u8, addr: u8, data: &[u8]) -> Box<dyn InstructionPacket<Self>>;
    fn sync_read_packet(ids: &[u8], addr: u8, length: u8) -> Box<dyn InstructionPacket<Self>>;
    fn sync_write_packet(
        ids: &[u8],
        addr: u8,
        data: &[Vec<u8>],
    ) -> Box<dyn InstructionPacket<Self>>;

    fn status_packet(data: &[u8], sender_id: u8) -> Result<Box<dyn StatusPacket<Self>>>;
}

pub trait InstructionPacket<P: Packet> {
    fn id(&self) -> u8;
    fn instruction(&self) -> P::InstructionKind;
    fn params(&self) -> &Vec<u8>;

    fn to_bytes(&self) -> Vec<u8>;
}
pub trait StatusPacket<P: Packet> {
    fn from_bytes(data: &[u8], sender_id: u8) -> Result<Self>
    where
        Self: Sized;

    fn id(&self) -> u8;
    fn errors(&self) -> Vec<P::ErrorKind>;
    fn params(&self) -> &Vec<u8>;
}
