use std::fmt::Debug;

use crate::Result;

pub trait Packet {
    const HEADER_SIZE: usize;
    type ErrorKind: Debug;
    type InstructionKind: Debug;

    fn get_payload_size(header: &[u8]) -> Result<usize>;

    fn ping_packet(id: u8) -> Box<dyn InstructionPacket<Self>>;
    fn reboot_packet(id: u8) -> Box<dyn InstructionPacket<Self>>;
    fn factory_reset_packet(
        id: u8,
        conserve_id_only: bool,
        conserve_id_and_baudrate: bool,
    ) -> Box<dyn InstructionPacket<Self>>;

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

impl<P: Packet> Debug for dyn InstructionPacket<P> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "InstructionPacket {{ id: {}, instruction: {:?}, params: {:?} }}",
            self.id(),
            self.instruction(),
            self.params()
        )
    }
}

pub trait StatusPacket<P: Packet> {
    fn from_bytes(data: &[u8], sender_id: u8) -> Result<Self>
    where
        Self: Sized;

    fn id(&self) -> u8;
    fn errors(&self) -> &Vec<P::ErrorKind>;
    fn params(&self) -> &Vec<u8>;
}

impl<P: Packet> Debug for dyn StatusPacket<P> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "StatusPacket {{ id: {}, errors: {:?}, params: {:?} }}",
            self.id(),
            self.errors(),
            self.params()
        )
    }
}
