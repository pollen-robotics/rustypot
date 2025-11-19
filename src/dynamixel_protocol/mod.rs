use serialport::SerialPort;

mod packet;
use packet::{InstructionPacket, Packet, StatusPacket};

mod v1;
use v1::V1;

mod v2;
use v2::V2;

use crate::Result;

#[derive(Debug)]
enum ProtocolKind {
    V1(V1),
    V2(V2),
}

#[derive(Debug)]
/// Raw dynamixel communication messages controller (protocol v1 or v2)
pub struct DynamixelProtocolHandler {
    protocol: ProtocolKind,
    post_delay: Option<Duration>,
}

impl DynamixelProtocolHandler {
    /// Creates a protocol v1 communication IO.
    ///
    /// For more information on protocol v1, please refer to <https://emanual.robotis.com/docs/en/dxl/protocol1/>
    ///
    /// # Examples
    /// ```no_run
    /// use rustypot::{DynamixelProtocolHandler, servo::dynamixel::mx};
    /// use std::time::Duration;
    ///
    /// let mut serial_port = serialport::new("/dev/ttyACM0", 1_000_000)
    ///     .timeout(Duration::from_millis(10))
    ///     .open()
    ///     .expect("Failed to open port");
    ///
    /// let dph = DynamixelProtocolHandler::v1();
    ///
    /// let pos =
    ///     mx::read_present_position(&dph, serial_port.as_mut(), 11).expect("Communication error");
    /// println!("Motor MX ID: 11 present position: {:?}", pos);
    /// ```
    pub fn v1() -> Self {
        DynamixelProtocolHandler {
            protocol: ProtocolKind::V1(V1),
            post_delay: None,
        }
    }
    /// Creates a protocol v2 communication IO.
    ///
    /// For more information on protocol v2, please refer to <https://emanual.robotis.com/docs/en/dxl/protocol2/>
    ///
    /// # Examples
    /// ```no_run
    /// use rustypot::{DynamixelProtocolHandler, servo::dynamixel::xl320};
    /// use std::time::Duration;
    ///
    /// let mut serial_port = serialport::new("/dev/ttyACM0", 1_000_000)
    ///     .timeout(Duration::from_millis(10))
    ///     .open()
    ///     .expect("Failed to open port");
    ///
    /// let dph = DynamixelProtocolHandler::v2();
    ///
    /// let pos =
    ///     xl320::read_present_position(&dph, serial_port.as_mut(), 11).expect("Communication error");
    /// println!("Motor XL-320 ID: 11 present position: {:?}", pos);
    /// ```
    pub fn v2() -> Self {
        DynamixelProtocolHandler {
            protocol: ProtocolKind::V2(V2),
            post_delay: None,
        }
    }

    /// Set a delay after each communication.
    pub fn with_post_delay(self, delay: Duration) -> Self {
        DynamixelProtocolHandler {
            post_delay: Some(delay),
            ..self
        }
    }

    /// Send a ping instruction.
    ///
    /// Ping the motor with specified `id`.
    /// Returns an [CommunicationErrorKind] if the communication fails.
    ///
    /// # Examples
    /// ```no_run
    /// use rustypot::DynamixelProtocolHandler;
    /// use std::time::Duration;
    ///
    /// let mut serial_port = serialport::new("/dev/ttyACM0", 1_000_000)
    ///     .timeout(Duration::from_millis(10))
    ///     .open()
    ///     .expect("Failed to open port");
    ///
    /// let dph = DynamixelProtocolHandler::v1();
    ///
    /// match dph
    ///     .ping(serial_port.as_mut(), 42)
    ///     .expect("Communication error")
    /// {
    ///     true => println!("Motor 42 found!"),
    ///     false => println!("Motor 42 did not respond"),
    /// }
    /// ```
    pub fn ping(&self, serial_port: &mut dyn serialport::SerialPort, id: u8) -> Result<bool> {
        match &self.protocol {
            ProtocolKind::V1(p) => p.ping(serial_port, id),
            ProtocolKind::V2(p) => p.ping(serial_port, id),
        }
    }

    /// Send a reboot instruction.
    ///
    /// Reboot the motor with specified `id`.
    /// Returns an [CommunicationErrorKind] if the communication fails.
    pub fn reboot(&self, serial_port: &mut dyn serialport::SerialPort, id: u8) -> Result<bool> {
        match &self.protocol {
            ProtocolKind::V1(p) => p.reboot(serial_port, id),
            ProtocolKind::V2(p) => p.reboot(serial_port, id),
        }
    }

    /// Factory reset instruction.
    ///
    /// Reset the Control Table of DYNAMIXEL to the factory default values.
    /// Please note that conserving ID and/or Baudrate is only supported on protocol v2.
    pub fn factory_reset(
        &self,
        serial_port: &mut dyn serialport::SerialPort,
        id: u8,
        conserve_id_only: bool,
        conserve_id_and_baudrate: bool,
    ) -> Result<()> {
        match &self.protocol {
            ProtocolKind::V1(p) => {
                if conserve_id_only || conserve_id_and_baudrate {
                    return Err(Box::new(CommunicationErrorKind::Unsupported));
                }
                p.factory_reset(serial_port, id, conserve_id_only, conserve_id_and_baudrate)
            }
            ProtocolKind::V2(p) => {
                p.factory_reset(serial_port, id, conserve_id_only, conserve_id_and_baudrate)
            }
        }
    }

    /// Reads raw register bytes.
    ///
    /// Sends a read instruction to the motor and wait for the status packet in response.
    /// Returns raw bytes without interpretation.
    /// For higher level methods, check the [device] implementation.
    ///
    /// # Arguments
    ///
    /// * `serial_port` - the serial port to use for communication
    /// * `id` - id of the motor
    /// * `addr` - register address
    /// * `length` - number of bytes to read
    ///
    /// # Examples
    /// ```no_run
    /// use rustypot::DynamixelProtocolHandler;
    /// use std::time::Duration;
    ///
    /// let mut serial_port = serialport::new("/dev/ttyACM0", 1_000_000)
    ///     .timeout(Duration::from_millis(10))
    ///     .open()
    ///     .expect("Failed to open port");
    ///
    /// let dph = DynamixelProtocolHandler::v1();
    ///
    /// // Read 2 bytes from register address 36 of motor 10
    /// let bytes = dph
    ///     .read(serial_port.as_mut(), 10, 36, 2)
    ///     .expect("Communication error");
    /// assert_eq!(bytes.len(), 2);
    /// ```
    pub fn read(
        &self,
        serial_port: &mut dyn serialport::SerialPort,
        id: u8,
        addr: u8,
        length: u8,
    ) -> Result<Vec<u8>> {
        let res = match &self.protocol {
            ProtocolKind::V1(p) => p.read(serial_port, id, addr, length),
            ProtocolKind::V2(p) => p.read(serial_port, id, addr, length),
        };
        if let Some(delay) = self.post_delay {
            std::thread::sleep(delay);
        }
        res
    }

    /// Writes raw bytes to register.
    ///
    /// Sends a write instruction with the raw bytes as parameter to the motor.
    /// Wait for the status packet in response.
    /// For higher level methods, check the [device] implementation.
    ///
    /// # Arguments
    ///
    /// * `serial_port` - the serial port to use for communication
    /// * `id` - id of the motor
    /// * `addr` - register address
    /// * `data` - raw bytes to write
    ///
    /// # Examples
    /// ```no_run
    /// use rustypot::DynamixelProtocolHandler;
    /// use std::time::Duration;
    ///
    /// let mut serial_port = serialport::new("/dev/ttyACM0", 1_000_000)
    ///     .timeout(Duration::from_millis(10))
    ///     .open()
    ///     .expect("Failed to open port");
    ///
    /// let dph = DynamixelProtocolHandler::v1();
    ///
    /// // Write a single byte to register address 24 of motor 33
    /// dph.write(serial_port.as_mut(), 33, 24, &vec![0])
    ///    .expect("Communication error");
    /// ```
    pub fn write(
        &self,
        serial_port: &mut dyn serialport::SerialPort,
        id: u8,
        addr: u8,
        data: &[u8],
    ) -> Result<()> {
        match &self.protocol {
            ProtocolKind::V1(p) => p.write(serial_port, id, addr, data),
            ProtocolKind::V2(p) => p.write(serial_port, id, addr, data),
        }?;
        if let Some(delay) = self.post_delay {
            std::thread::sleep(delay);
        }
        Ok(())
    }

    pub fn write_fb(
        &self,
        serial_port: &mut dyn serialport::SerialPort,
        id: u8,
        addr: u8,
        data: &[u8],
    ) -> Result<Vec<u8>> {
        match &self.protocol {
            ProtocolKind::V1(p) => {
                let res = p.write_fb(serial_port, id, addr, data);
                if let Some(delay) = self.post_delay {
                    std::thread::sleep(delay);
                }
                res
            }
            ProtocolKind::V2(_) => Err(Box::new(CommunicationErrorKind::Unsupported)),
        }
    }

    /// Reads raw register bytes from multiple ids at once.
    ///
    /// Sends a sync read instruction to the specified motors and wait for the status packet in response.
    /// Returns raw bytes without interpretation.
    /// For higher level methods, check the [device] implementation.
    ///
    /// *Note: sync read support on protocol v1 depends on usb to serial hardware used!*
    ///
    /// # Arguments
    ///
    /// * `serial_port` - the serial port to use for communication
    /// * `ids` - specfied motors id
    /// * `addr` - register address
    /// * `length` - number of bytes to read
    ///
    /// # Examples
    /// ```no_run
    /// use rustypot::DynamixelProtocolHandler;
    /// use std::time::Duration;
    ///
    /// let mut serial_port = serialport::new("/dev/ttyACM0", 1_000_000)
    ///     .timeout(Duration::from_millis(10))
    ///     .open()
    ///     .expect("Failed to open port");
    ///
    /// let dph = DynamixelProtocolHandler::v1();
    ///
    /// // Read a single byte from motor 10, 11 and 12 (addr 43)
    /// let resp = dph
    ///     .sync_read(serial_port.as_mut(), &[10, 11, 12], 43, 1)
    ///     .expect("Communication error");
    ///
    /// assert_eq!(resp.len(), 3);
    /// for bytes in resp {
    ///     assert_eq!(bytes.len(), 1);
    /// }
    /// ```
    pub fn sync_read(
        &self,
        serial_port: &mut dyn serialport::SerialPort,
        ids: &[u8],
        addr: u8,
        length: u8,
    ) -> Result<Vec<Vec<u8>>> {
        match &self.protocol {
            ProtocolKind::V1(p) => p.sync_read(serial_port, ids, addr, length),
            ProtocolKind::V2(p) => p.sync_read(serial_port, ids, addr, length),
        }
    }

    /// Write raw bytes to multiple ids at once.
    ///
    /// Sends a sync write instruction to the specified motors.
    /// No status response is sent back.
    /// For higher level methods, check the [device] implementation.
    ///
    /// # Arguments
    ///
    /// * `serial_port` - the serial port to use for communication
    /// * `ids` - specfied motors id
    /// * `addr` - register address
    /// * `data` - bytes to write to each motor
    ///
    /// # Examples
    /// ```no_run
    /// use rustypot::DynamixelProtocolHandler;
    /// use std::time::Duration;
    ///
    /// let mut serial_port = serialport::new("/dev/ttyACM0", 1_000_000)
    ///     .timeout(Duration::from_millis(10))
    ///     .open()
    ///     .expect("Failed to open port");
    ///
    /// let dph = DynamixelProtocolHandler::v1();
    ///
    /// // In a single message
    /// //  * writes 0 to register 25 of motor 40
    /// //  * writes 1 to register 25 of motor 41
    /// dph.sync_write(serial_port.as_mut(), &[40, 41], 25, &[vec![0], vec![1]])
    ///    .expect("Communication error");
    /// ```
    pub fn sync_write(
        &self,
        serial_port: &mut dyn serialport::SerialPort,
        ids: &[u8],
        addr: u8,
        data: &[Vec<u8>],
    ) -> Result<()> {
        match &self.protocol {
            ProtocolKind::V1(p) => p.sync_write(serial_port, ids, addr, data),
            ProtocolKind::V2(p) => p.sync_write(serial_port, ids, addr, data),
        }
    }
}

trait Protocol<P: Packet> {
    fn ping(&self, port: &mut dyn SerialPort, id: u8) -> Result<bool> {
        self.send_instruction_packet(port, P::ping_packet(id).as_ref())?;

        Ok(self.read_status_packet(port, id).is_ok())
    }

    fn reboot(&self, port: &mut dyn SerialPort, id: u8) -> Result<bool> {
        self.send_instruction_packet(port, P::reboot_packet(id).as_ref())?;

        Ok(self.read_status_packet(port, id).is_ok())
    }

    fn factory_reset(
        &self,
        port: &mut dyn SerialPort,
        id: u8,
        conserve_id_only: bool,
        conserve_id_and_baudrate: bool,
    ) -> Result<()> {
        self.send_instruction_packet(
            port,
            P::factory_reset_packet(id, conserve_id_only, conserve_id_and_baudrate).as_ref(),
        )?;
        self.read_status_packet(port, id).map(|_| ())
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

        log::debug!("<<< {data:?}");

        P::status_packet(&data, sender_id)
    }

    fn is_input_buffer_empty(&self, port: &mut dyn SerialPort) -> Result<bool> {
        let n = port.bytes_to_read()? as usize;
        Ok(n == 0)
    }

    fn flush(&self, port: &mut dyn SerialPort) -> Result<()> {
        let n = port.bytes_to_read()? as usize;
        if n > 0 {
            log::info!("Needed to flush serial port ({n} bytes)...");
            let mut buff = vec![0u8; n];
            port.read_exact(&mut buff)?;
        }

        Ok(())
    }
}

use std::{fmt, time::Duration};

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
                write!(f, "Incorrect id ({resp_id} instead of {sender_id})")
            }
            CommunicationErrorKind::Unsupported => write!(f, "Operation not supported"),
        }
    }
}
impl std::error::Error for CommunicationErrorKind {}
