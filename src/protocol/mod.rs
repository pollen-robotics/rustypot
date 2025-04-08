use serialport::SerialPort;

mod v1;
#[allow(unused_imports)]
pub use v1::{PacketV1, V1};

mod v2;
#[allow(unused_imports)]
pub use v2::{PacketV2, V2};

use crate::{
    packet::{InstructionPacket, StatusPacket},
    Packet, Result,
};

pub trait Protocol<P: Packet> {
    fn ping(&self, port: &mut dyn SerialPort, id: u8) -> Result<bool> {
        self.send_instruction_packet(port, P::ping_packet(id).as_ref())?;

        Ok(self.read_status_packet(port, id).is_ok())
    }

    fn read(&self, port: &mut dyn SerialPort, id: u8, addr: u8, length: u8) -> Result<Vec<u8>> {
        self.send_instruction_packet(port, P::read_packet(id, addr, length).as_ref())?;
        self.read_status_packet(port, id)
            .map(|sp| sp.params().to_vec())
    }
    fn write(&self, port: &mut dyn SerialPort, id: u8, addr: u8, data: &[u8]) -> Result<()> {
        self.send_instruction_packet(port, P::write_packet(id, addr, data).as_ref())?;
        self.read_status_packet(port, id).map(|_| ())
    }

    fn write_fb(
        &self,
        port: &mut dyn SerialPort,
        id: u8,
        addr: u8,
        data: &[u8],
    ) -> Result<Vec<u8>> {
        self.send_instruction_packet(port, P::write_packet(id, addr, data).as_ref())?;
        self.read_status_packet(port, id)
            .map(|sp| sp.params().to_vec())
    }

    fn sync_read(
        &self,
        port: &mut dyn SerialPort,
        ids: &[u8],
        addr: u8,
        length: u8,
    ) -> Result<Vec<Vec<u8>>> {
        self.send_instruction_packet(port, P::sync_read_packet(ids, addr, length).as_ref())?;
        let mut result = Vec::new();
        for id in ids {
            let sp = self.read_status_packet(port, *id)?;
            result.push(sp.params().to_vec());
        }
        Ok(result)
    }
    fn sync_write(
        &self,
        port: &mut dyn SerialPort,
        ids: &[u8],
        addr: u8,
        data: &[Vec<u8>],
    ) -> Result<()> {
        self.send_instruction_packet(port, P::sync_write_packet(ids, addr, data).as_ref())?;
        Ok(())
    }

    fn send_instruction_packet(
        &self,
        port: &mut dyn SerialPort,
        packet: &dyn InstructionPacket<P>,
    ) -> Result<()> {
        // Before we send an instruction
        // The input buffer should always be empty
        // (if not, it means that an old corrupted message need to be flushed)
        if !self.is_input_buffer_empty(port)? {
            self.flush(port)?;
        }
        assert!(self.is_input_buffer_empty(port)?);

        log::debug!(">>> {:?}", packet.to_bytes());

        match port.write_all(&packet.to_bytes()) {
            Ok(_) => Ok(()),
            Err(_) => Err(Box::new(CommunicationErrorKind::TimeoutError)),
        }
    }
    fn read_status_packet(
        &self,
        port: &mut dyn SerialPort,
        sender_id: u8,
    ) -> Result<Box<dyn StatusPacket<P>>> {
        let mut header = vec![0u8; P::HEADER_SIZE];
        port.read_exact(&mut header)?;

        let payload_size = P::get_payload_size(&header)?;
        let mut payload = vec![0u8; payload_size];
        port.read_exact(&mut payload)?;

        let mut data = Vec::new();
        data.extend(header);
        data.extend(payload);

        log::debug!("<<< {:?}", data);

        P::status_packet(&data, sender_id)
    }

    fn is_input_buffer_empty(&self, port: &mut dyn SerialPort) -> Result<bool> {
        let n = port.bytes_to_read()? as usize;
        Ok(n == 0)
    }

    fn flush(&self, port: &mut dyn SerialPort) -> Result<()> {
        let n = port.bytes_to_read()? as usize;
        if n > 0 {
            log::info!("Needed to flush serial port ({} bytes)...", n);
            let mut buff = vec![0u8; n];
            port.read_exact(&mut buff)?;
        }

        Ok(())
    }
}

use std::fmt;

/// Dynamixel Communication Error
#[derive(Debug, Clone, Copy)]
pub enum CommunicationErrorKind {
    /// Incorrect checksum
    ChecksumError,
    /// Could not parse incoherent message
    ParsingError,
    /// Timeout
    TimeoutError,
    /// Incorrect response id - different from sender (sender id, response id)
    IncorrectId(u8, u8),

    /// Operation not supported
    Unsupported,
}
impl fmt::Display for CommunicationErrorKind {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            CommunicationErrorKind::ChecksumError => write!(f, "Checksum error"),
            CommunicationErrorKind::ParsingError => write!(f, "Parsing error"),
            CommunicationErrorKind::TimeoutError => write!(f, "Timeout error"),
            CommunicationErrorKind::IncorrectId(sender_id, resp_id) => {
                write!(f, "Incorrect id ({} instead of {})", resp_id, sender_id)
            }
            CommunicationErrorKind::Unsupported => write!(f, "Operation not supported"),
        }
    }
}
impl std::error::Error for CommunicationErrorKind {}
