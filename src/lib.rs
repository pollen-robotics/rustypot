pub mod protocol;
use protocol::Protocol;

mod packet;
use packet::Packet;

pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

pub struct DynamixelSerialIO<P: Packet> {
    inner: Box<dyn Protocol<P>>,
}
impl<P: Packet> DynamixelSerialIO<P> {
    pub fn new<D: Protocol<P> + 'static>() -> Self {
        DynamixelSerialIO {
            inner: Box::new(D::new()),
        }
    }

    pub fn ping(&self, serial_port: &mut dyn serialport::SerialPort, id: u8) -> Result<bool> {
        self.inner.ping(serial_port, id)
    }

    pub fn read(
        &self,
        serial_port: &mut dyn serialport::SerialPort,
        id: u8,
        addr: u8,
        length: u8,
    ) -> Result<Vec<u8>> {
        self.inner.read(serial_port, id, addr, length)
    }

    pub fn write(
        &self,
        serial_port: &mut dyn serialport::SerialPort,
        id: u8,
        addr: u8,
        data: &[u8],
    ) -> Result<()> {
        self.inner.write(serial_port, id, addr, data)
    }

    pub fn sync_read(
        &self,
        serial_port: &mut dyn serialport::SerialPort,
        ids: &[u8],
        addr: u8,
        length: u8,
    ) -> Result<Vec<Vec<u8>>> {
        self.inner.sync_read(serial_port, ids, addr, length)
    }

    pub fn sync_write(
        &self,
        serial_port: &mut dyn serialport::SerialPort,
        ids: &[u8],
        addr: u8,
        data: &[&[u8]],
    ) -> Result<()> {
        self.inner.sync_write(serial_port, ids, addr, data)
    }
}
