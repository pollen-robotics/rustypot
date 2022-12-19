pub mod protocol;
use protocol::{Protocol, V1, V2};

mod packet;
use packet::Packet;

pub mod device;

pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

pub enum Protocols {
    V1(V1),
    V2(V2),
}

pub struct DynamixelSerialIO {
    protocol: Protocols,
}

impl DynamixelSerialIO {
    pub fn v1() -> Self {
        DynamixelSerialIO {
            protocol: Protocols::V1(V1),
        }
    }
    pub fn v2() -> Self {
        DynamixelSerialIO {
            protocol: Protocols::V2(V2),
        }
    }

    pub fn ping(&self, serial_port: &mut dyn serialport::SerialPort, id: u8) -> Result<bool> {
        match &self.protocol {
            Protocols::V1(p) => p.ping(serial_port, id),
            Protocols::V2(p) => p.ping(serial_port, id),
        }
    }

    pub fn read(
        &self,
        serial_port: &mut dyn serialport::SerialPort,
        id: u8,
        addr: u8,
        length: u8,
    ) -> Result<Vec<u8>> {
        match &self.protocol {
            Protocols::V1(p) => p.read(serial_port, id, addr, length),
            Protocols::V2(p) => p.read(serial_port, id, addr, length),
        }
    }

    pub fn write(
        &self,
        serial_port: &mut dyn serialport::SerialPort,
        id: u8,
        addr: u8,
        data: &[u8],
    ) -> Result<()> {
        match &self.protocol {
            Protocols::V1(p) => p.write(serial_port, id, addr, data),
            Protocols::V2(p) => p.write(serial_port, id, addr, data),
        }
    }

    pub fn sync_read(
        &self,
        serial_port: &mut dyn serialport::SerialPort,
        ids: &[u8],
        addr: u8,
        length: u8,
    ) -> Result<Vec<Vec<u8>>> {
        match &self.protocol {
            Protocols::V1(p) => p.sync_read(serial_port, ids, addr, length),
            Protocols::V2(p) => p.sync_read(serial_port, ids, addr, length),
        }
    }

    pub fn sync_write(
        &self,
        serial_port: &mut dyn serialport::SerialPort,
        ids: &[u8],
        addr: u8,
        data: &[Vec<u8>],
    ) -> Result<()> {
        match &self.protocol {
            Protocols::V1(p) => p.sync_write(serial_port, ids, addr, data),
            Protocols::V2(p) => p.sync_write(serial_port, ids, addr, data),
        }
    }
}
