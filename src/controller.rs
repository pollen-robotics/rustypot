use std::collections::HashSet;
use std::mem::size_of;

use crate::protocol::{DynamixelErrorKind, FromBytes, ToBytes};
use crate::serialize::Serializable;

use super::protocol::v1::{InstructionPacket, StatusPacket};
use super::{CommunicationErrorKind, DynamixelLikeIO};

pub struct Controller {
    io: Box<dyn DynamixelLikeIO + Send>,
    errors: HashSet<DynamixelErrorKind>,
}

pub trait Register {
    fn address(&self) -> u8;
}

impl Controller {
    pub fn new(io: Box<dyn DynamixelLikeIO + Send>) -> Self {
        Controller {
            errors: HashSet::new(),
            io,
        }
    }

    pub fn get_register<T: Serializable>(
        &mut self,
        id: u8,
        reg: &dyn Register,
    ) -> Result<T, CommunicationErrorKind> {
        let instruction_packet =
            InstructionPacket::read_packet(id, reg.address(), size_of::<T>().try_into().unwrap());
        let status_packet = self.request(instruction_packet)?;
        T::from_bytes(status_packet.payload).ok_or(CommunicationErrorKind::ParsingError)
    }

    pub fn set_register<T: Serializable>(
        &mut self,
        id: u8,
        reg: &dyn Register,
        value: T,
    ) -> Result<(), CommunicationErrorKind> {
        let instruction_packet =
            InstructionPacket::write_packet(id, reg.address(), value.to_bytes());
        self.request(instruction_packet)?;
        Ok(())
    }

    pub fn get_errors(&mut self) -> Vec<DynamixelErrorKind> {
        self.errors.iter().copied().collect()
    }
    pub fn reset_errors(&mut self) {
        self.errors.clear()
    }

    fn request(
        &mut self,
        instruction_packet: InstructionPacket,
    ) -> Result<StatusPacket, CommunicationErrorKind> {
        self.io.send_packet(instruction_packet.to_bytes());

        let data = self.io.read_packet()?;
        let status_packet = StatusPacket::from_bytes(instruction_packet.id, data)?;

        for e in status_packet.error.iter().copied() {
            self.errors.insert(e);
        }

        Ok(status_packet)
    }
}
