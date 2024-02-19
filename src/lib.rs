//! Yet another communication library for robotis Dynamixel motors.
//!
//! ## Feature Overview
//!
//! * Relies on [serialport] for serial communication
//! * Support for dynamixel protocol v1 and v2 (can also use both on the same bus)
//! * Support for sync read and sync write operations
//! * Easy support for new type of motors (register definition through macros)
//! * Pure Rust
//!
//! *Note: this version use std and Vec extensively.*
//!
//! ## Examples
//! ```no_run
//! use rustypot::{DynamixelSerialIO, device::mx};
//! use std::time::Duration;
//!
//! let mut serial_port = serialport::new("/dev/ttyACM0", 1_000_000)
//!     .timeout(Duration::from_millis(10))
//!     .open()
//!     .expect("Failed to open port");
//!
//! let io = DynamixelSerialIO::v1();
//!
//! let pos =
//!     mx::read_present_position(&io, serial_port.as_mut(), 11).expect("Communication error");
//! println!("Motor 11 present position: {:?}", pos);
//! ```

mod protocol;
use std::time::Duration;

pub use protocol::CommunicationErrorKind;
use protocol::{Protocol, V1, V2};

mod packet;
use packet::Packet;

pub mod device;

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

#[derive(Debug)]
enum Protocols {
    V1(V1),
    V2(V2),
}

#[derive(Debug)]
/// Raw dynamixel communication messages controller (protocol v1 or v2)
pub struct DynamixelSerialIO {
    protocol: Protocols,
    post_delay: Option<Duration>,
}

impl DynamixelSerialIO {
    /// Creates a protocol v1 communication IO.
    ///
    /// For more information on protocol v1, please refer to <https://emanual.robotis.com/docs/en/dxl/protocol1/>
    ///
    /// # Examples
    /// ```no_run
    /// use rustypot::{DynamixelSerialIO, device::mx};
    /// use std::time::Duration;
    ///
    /// let mut serial_port = serialport::new("/dev/ttyACM0", 1_000_000)
    ///     .timeout(Duration::from_millis(10))
    ///     .open()
    ///     .expect("Failed to open port");
    ///
    /// let io = DynamixelSerialIO::v1();
    ///
    /// let pos =
    ///     mx::read_present_position(&io, serial_port.as_mut(), 11).expect("Communication error");
    /// println!("Motor MX ID: 11 present position: {:?}", pos);
    /// ```
    pub fn v1() -> Self {
        DynamixelSerialIO {
            protocol: Protocols::V1(V1),
            post_delay: None,
        }
    }
    /// Creates a protocol v2 communication IO.
    ///
    /// For more information on protocol v2, please refer to <https://emanual.robotis.com/docs/en/dxl/protocol2/>
    ///
    /// # Examples
    /// ```no_run
    /// use rustypot::{DynamixelSerialIO, device::xl320};
    /// use std::time::Duration;
    ///
    /// let mut serial_port = serialport::new("/dev/ttyACM0", 1_000_000)
    ///     .timeout(Duration::from_millis(10))
    ///     .open()
    ///     .expect("Failed to open port");
    ///
    /// let io = DynamixelSerialIO::v2();
    ///
    /// let pos =
    ///     xl320::read_present_position(&io, serial_port.as_mut(), 11).expect("Communication error");
    /// println!("Motor XL-320 ID: 11 present position: {:?}", pos);
    /// ```
    pub fn v2() -> Self {
        DynamixelSerialIO {
            protocol: Protocols::V2(V2),
            post_delay: None,
        }
    }

    /// Set a delay after each communication.
    pub fn with_post_delay(self, delay: Duration) -> Self {
        DynamixelSerialIO {
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
    /// use rustypot::DynamixelSerialIO;
    /// use std::time::Duration;
    ///
    /// let mut serial_port = serialport::new("/dev/ttyACM0", 1_000_000)
    ///     .timeout(Duration::from_millis(10))
    ///     .open()
    ///     .expect("Failed to open port");
    ///
    /// let io = DynamixelSerialIO::v1();
    ///
    /// match io
    ///     .ping(serial_port.as_mut(), 42)
    ///     .expect("Communication error")
    /// {
    ///     true => println!("Motor 42 found!"),
    ///     false => println!("Motor 42 did not respond"),
    /// }
    /// ```
    pub fn ping(&self, serial_port: &mut dyn serialport::SerialPort, id: u8) -> Result<bool> {
        match &self.protocol {
            Protocols::V1(p) => p.ping(serial_port, id),
            Protocols::V2(p) => p.ping(serial_port, id),
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
    /// use rustypot::DynamixelSerialIO;
    /// use std::time::Duration;
    ///
    /// let mut serial_port = serialport::new("/dev/ttyACM0", 1_000_000)
    ///     .timeout(Duration::from_millis(10))
    ///     .open()
    ///     .expect("Failed to open port");
    ///
    /// let io = DynamixelSerialIO::v1();
    ///
    /// // Read 2 bytes from register address 36 of motor 10
    /// let bytes = io
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
            Protocols::V1(p) => p.read(serial_port, id, addr, length),
            Protocols::V2(p) => p.read(serial_port, id, addr, length),
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
    /// use rustypot::DynamixelSerialIO;
    /// use std::time::Duration;
    ///
    /// let mut serial_port = serialport::new("/dev/ttyACM0", 1_000_000)
    ///     .timeout(Duration::from_millis(10))
    ///     .open()
    ///     .expect("Failed to open port");
    ///
    /// let io = DynamixelSerialIO::v1();
    ///
    /// // Write a single byte to register address 24 of motor 33
    /// io.write(serial_port.as_mut(), 33, 24, &vec![0])
    ///     .expect("Communication error");
    /// ```
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
        }?;
        if let Some(delay) = self.post_delay {
            std::thread::sleep(delay);
        }
        Ok(())
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
    /// use rustypot::DynamixelSerialIO;
    /// use std::time::Duration;
    ///
    /// let mut serial_port = serialport::new("/dev/ttyACM0", 1_000_000)
    ///     .timeout(Duration::from_millis(10))
    ///     .open()
    ///     .expect("Failed to open port");
    ///
    /// let io = DynamixelSerialIO::v1();
    ///
    /// // Read a single byte from motor 10, 11 and 12 (addr 43)
    /// let resp = io
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
            Protocols::V1(p) => p.sync_read(serial_port, ids, addr, length),
            Protocols::V2(p) => p.sync_read(serial_port, ids, addr, length),
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
    /// use rustypot::DynamixelSerialIO;
    /// use std::time::Duration;
    ///
    /// let mut serial_port = serialport::new("/dev/ttyACM0", 1_000_000)
    ///     .timeout(Duration::from_millis(10))
    ///     .open()
    ///     .expect("Failed to open port");
    ///
    /// let io = DynamixelSerialIO::v1();
    ///
    /// // In a single message
    /// //  * writes 0 to register 25 of motor 40
    /// //  * writes 1 to register 25 of motor 41
    /// io.sync_write(serial_port.as_mut(), &[40, 41], 25, &[vec![0], vec![1]])
    ///     .expect("Communication error");
    /// ```
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
