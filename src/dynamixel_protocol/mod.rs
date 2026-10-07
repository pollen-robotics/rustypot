use serialport::SerialPort;

mod packet;
use packet::{InstructionPacket, Packet, StatusPacket};

mod v1;
pub use v1::DynamixelErrorV1;
use v1::V1;

mod v2;
use v2::V2;

use crate::Result;

#[derive(Debug)]
enum ProtocolKind {
    V1(V1),
    /// The bool routes sync reads through fast sync read (protocol v2 only)
    V2(V2, bool),
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
            protocol: ProtocolKind::V2(V2, false),
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

    /// Sleep the configured post delay, if there is one.
    ///
    /// Called after every bus transaction, whether or not it succeeded: the delay is
    /// there to leave a gap on the wire, and a transaction that timed out has used the
    /// bus just the same -- an immediate retry is exactly the case it guards against.
    fn sleep_post_delay(&self) {
        if let Some(delay) = self.post_delay {
            std::thread::sleep(delay);
        }
    }

    /// Make [DynamixelProtocolHandler::sync_read] use Fast Sync Read.
    ///
    /// Protocol v2 only, and needs firmware accepting instruction 0x8A (XL330: v46+).
    /// Ignored on protocol v1.
    ///
    /// # Examples
    /// ```no_run
    /// use rustypot::DynamixelProtocolHandler;
    ///
    /// let dph = DynamixelProtocolHandler::v2().with_fast_sync_read();
    /// ```
    pub fn with_fast_sync_read(mut self) -> Self {
        self.set_fast_sync_read(true);
        self
    }

    /// Turn the Fast Sync Read routing of [DynamixelProtocolHandler::sync_read] on or off.
    ///
    /// Has no effect on protocol v1, which has no such instruction.
    pub fn set_fast_sync_read(&mut self, enabled: bool) {
        if let ProtocolKind::V2(_, fast) = &mut self.protocol {
            *fast = enabled;
        }
    }

    /// Whether [DynamixelProtocolHandler::sync_read] currently uses Fast Sync Read.
    pub fn fast_sync_read_enabled(&self) -> bool {
        matches!(self.protocol, ProtocolKind::V2(_, true))
    }

    /// The protocol version this handler speaks: 1 or 2.
    pub fn protocol_version(&self) -> u8 {
        match self.protocol {
            ProtocolKind::V1(_) => 1,
            ProtocolKind::V2(..) => 2,
        }
    }

    /// The highest id a motor can take on this protocol: 253 on v1, 252 on v2, where
    /// 253 is reserved. Ids start at 0 on both.
    pub fn max_id(&self) -> u8 {
        match self.protocol {
            ProtocolKind::V1(_) => v1::MAX_ID,
            ProtocolKind::V2(..) => v2::MAX_ID,
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
            ProtocolKind::V2(p, _) => p.ping(serial_port, id),
        }
    }

    /// Ping every motor at once, and return the ids that answer.
    ///
    /// Each motor that hears the broadcast ping answers with its own status packet, in
    /// turn, so finding all of them takes one listening window
    /// ([`broadcast_ping_window`](Self::broadcast_ping_window) at the port's baud rate)
    /// instead of one timeout per absent id. The port's timeout is put back afterwards.
    ///
    /// Only some firmwares answer: protocol v2 Dynamixel and Feetech STS do, protocol v1
    /// Dynamixel and Feetech SCS stay silent, which reads as no motor at all.
    pub fn broadcast_ping(&self, serial_port: &mut dyn serialport::SerialPort) -> Result<Vec<u8>> {
        let window = self.broadcast_ping_window(serial_port.baud_rate()?);
        let timeout = serial_port.timeout();
        let ids = match &self.protocol {
            ProtocolKind::V1(p) => p.broadcast_ping(serial_port, window),
            ProtocolKind::V2(p, _) => p.broadcast_ping(serial_port, window),
        };
        serial_port.set_timeout(timeout)?;
        self.sleep_post_delay();
        ids
    }

    /// How long a broadcast ping listens at `baudrate`: the window the vendor SDKs use.
    ///
    /// The status packets of every id the protocol allows (6 bytes each on v1, 14 on v2),
    /// 3 ms of turn per id, and 16 ms for the latency timer of a USB serial adapter, which
    /// can hold a short answer that long before passing it on.
    pub fn broadcast_ping_window(&self, baudrate: u32) -> Duration {
        let (status_size, max_id) = match self.protocol {
            ProtocolKind::V1(_) => (6, v1::MAX_ID),
            ProtocolKind::V2(..) => (14, v2::MAX_ID),
        };
        let max_id = u64::from(max_id);
        let bits = status_size * max_id * 10;
        Duration::from_micros(bits * 1_000_000 / u64::from(baudrate))
            + Duration::from_millis(3 * max_id + 16)
    }

    /// Send a reboot instruction.
    ///
    /// Reboot the motor with specified `id`.
    /// Returns an [CommunicationErrorKind] if the communication fails.
    pub fn reboot(&self, serial_port: &mut dyn serialport::SerialPort, id: u8) -> Result<bool> {
        match &self.protocol {
            ProtocolKind::V1(p) => p.reboot(serial_port, id),
            ProtocolKind::V2(p, _) => p.reboot(serial_port, id),
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
            ProtocolKind::V2(p, _) => {
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
            ProtocolKind::V2(p, _) => p.read(serial_port, id, addr, length),
        };
        self.sleep_post_delay();
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
        let res = match &self.protocol {
            ProtocolKind::V1(p) => p.write(serial_port, id, addr, data),
            ProtocolKind::V2(p, _) => p.write(serial_port, id, addr, data),
        };
        self.sleep_post_delay();
        res
    }

    /// Same as [DynamixelProtocolHandler::read], and also returns the status packet's
    /// error field. Like `read`, it honours the handler's post delay.
    ///
    /// A motor can answer a read it could serve while still reporting a fault.
    /// [DynamixelProtocolHandler::read] drops that byte; this keeps it, so a caller can
    /// surface the condition instead of driving a motor that is reporting one.
    ///
    /// The byte is returned unparsed, because its layout depends on the protocol and
    /// only the caller knows which motor family it is talking to:
    ///
    /// - **Protocol v1** -- a bitfield of motor conditions: input voltage, angle limit,
    ///   overheating, range, checksum, overload, instruction.
    /// - **Protocol v2** -- not a bitfield. Bits 0-6 are an instruction-error *number*
    ///   (1 Result Fail, 2 Instruction Error, 3 CRC Error, 4 Data Range Error,
    ///   5 Data Length Error, 6 Data Limit Error, 7 Access Error), and bit 7 is Alert,
    ///   which only says that a hardware fault is set -- the condition itself must be
    ///   read from the Hardware Error Status register.
    pub fn read_with_error(
        &self,
        serial_port: &mut dyn serialport::SerialPort,
        id: u8,
        addr: u8,
        length: u8,
    ) -> Result<(Vec<u8>, StatusError)> {
        let res = match &self.protocol {
            ProtocolKind::V1(p) => p.read_with_error(serial_port, id, addr, length),
            ProtocolKind::V2(p, _) => p.read_with_error(serial_port, id, addr, length),
        }
        .map(|(values, e)| (values, StatusError::new(e)));
        self.sleep_post_delay();
        res
    }

    /// Same as [DynamixelProtocolHandler::write], and also returns the status packet's
    /// error field. See [DynamixelProtocolHandler::read_with_error].
    pub fn write_with_error(
        &self,
        serial_port: &mut dyn serialport::SerialPort,
        id: u8,
        addr: u8,
        data: &[u8],
    ) -> Result<StatusError> {
        let res = match &self.protocol {
            ProtocolKind::V1(p) => p.write_with_error(serial_port, id, addr, data),
            ProtocolKind::V2(p, _) => p.write_with_error(serial_port, id, addr, data),
        }
        .map(StatusError::new);
        self.sleep_post_delay();
        res
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
                self.sleep_post_delay();
                res
            }
            ProtocolKind::V2(..) => Err(Box::new(CommunicationErrorKind::Unsupported)),
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
        let res = match &self.protocol {
            ProtocolKind::V1(p) => p.sync_read(serial_port, ids, addr, length),
            ProtocolKind::V2(p, false) => p.sync_read(serial_port, ids, addr, length),
            ProtocolKind::V2(p, true) => p.fast_sync_read(serial_port, ids, addr, length),
        };
        self.sleep_post_delay();
        res
    }

    /// Same as [DynamixelProtocolHandler::sync_read], and also returns each motor's
    /// error field. Like `sync_read`, it routes to fast sync read when that is enabled.
    ///
    /// A control loop polling with `sync_read` is exactly where a motor reporting a fault
    /// goes unnoticed: it keeps answering, so the read succeeds and the caller never sees
    /// the condition. See [DynamixelProtocolHandler::read_with_error] for how to read the
    /// byte on each protocol.
    pub fn sync_read_with_error(
        &self,
        serial_port: &mut dyn serialport::SerialPort,
        ids: &[u8],
        addr: u8,
        length: u8,
    ) -> Result<Vec<(Vec<u8>, StatusError)>> {
        let res = match &self.protocol {
            ProtocolKind::V1(p) => p.sync_read_with_error(serial_port, ids, addr, length),
            ProtocolKind::V2(p, false) => p.sync_read_with_error(serial_port, ids, addr, length),
            ProtocolKind::V2(p, true) => {
                p.fast_sync_read_with_error(serial_port, ids, addr, length)
            }
        };
        self.sleep_post_delay();
        Ok(res?
            .into_iter()
            .map(|(values, e)| (values, StatusError::new(e)))
            .collect())
    }

    /// Reads raw register bytes from multiple ids at once, using a single status packet.
    ///
    /// Same as [DynamixelProtocolHandler::sync_read], but sends a fast sync read
    /// instruction (0x8A): every motor appends its answer to one status packet returned
    /// from the broadcast id, instead of each sending its own. This saves a packet header
    /// and a bus turnaround (plus its return delay time) per motor.
    ///
    /// Protocol v2 only, and only on firmware new enough to implement it (XL330: v46+;
    /// the cutoff differs per model). Older firmware does not answer, which surfaces as a
    /// timeout (update the firmware if this occurs).
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
    /// let mut serial_port = serialport::new("/dev/ttyUSB0", 1_000_000)
    ///     .timeout(Duration::from_millis(10))
    ///     .open()
    ///     .expect("Failed to open port");
    ///
    /// let dph = DynamixelProtocolHandler::v2();
    ///
    /// // Read the present position (addr 132, 4 bytes) of motors 10, 11 and 12
    /// let resp = dph
    ///     .fast_sync_read(serial_port.as_mut(), &[10, 11, 12], 132, 4)
    ///     .expect("Communication error");
    ///
    /// assert_eq!(resp.len(), 3);
    /// ```
    pub fn fast_sync_read(
        &self,
        serial_port: &mut dyn serialport::SerialPort,
        ids: &[u8],
        addr: u8,
        length: u8,
    ) -> Result<Vec<Vec<u8>>> {
        // The v1 arm never reaches the bus, so it has no gap to leave -- same shape as
        // write_fb.
        match &self.protocol {
            ProtocolKind::V1(_) => Err(Box::new(CommunicationErrorKind::Unsupported)),
            ProtocolKind::V2(p, _) => {
                let res = p.fast_sync_read(serial_port, ids, addr, length);
                self.sleep_post_delay();
                res
            }
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
        let res = match &self.protocol {
            ProtocolKind::V1(p) => p.sync_write(serial_port, ids, addr, data),
            ProtocolKind::V2(p, _) => p.sync_write(serial_port, ids, addr, data),
        };
        self.sleep_post_delay();
        res
    }
}

/// The ids of the status packets in `bytes`. Anything that does not frame one (a stray
/// byte, a packet cut short, a bad checksum) is skipped a byte at a time until the next
/// header, so it costs that packet and not the ones after it.
fn status_packet_ids<P: Packet>(bytes: &[u8]) -> Vec<u8> {
    let mut ids = Vec::new();
    let mut start = 0;
    while start + P::HEADER_SIZE <= bytes.len() {
        let packet = P::get_payload_size(&bytes[start..start + P::HEADER_SIZE])
            .ok()
            .map(|payload| &bytes[start..bytes.len().min(start + P::HEADER_SIZE + payload)]);
        match packet {
            Some(packet) if P::status_packet(packet, packet[P::ID_POSITION]).is_ok() => {
                ids.push(packet[P::ID_POSITION]);
                start += packet.len();
            }
            _ => start += 1,
        }
    }
    ids
}

trait Protocol<P: Packet> {
    fn ping(&self, port: &mut dyn SerialPort, id: u8) -> Result<bool> {
        self.send_instruction_packet(port, P::ping_packet(id).as_ref())?;

        Ok(self.read_status_packet(port, id).is_ok())
    }

    /// Ping the broadcast id, collect every byte that arrives until `window` closes, and
    /// return the ids of the status packets among them. Changes the port's timeout; the
    /// caller puts it back.
    fn broadcast_ping(&self, port: &mut dyn SerialPort, window: Duration) -> Result<Vec<u8>> {
        self.send_instruction_packet(port, P::ping_packet(P::BROADCAST_ID).as_ref())?;
        let deadline = Instant::now() + window;
        let mut received = Vec::new();
        let mut chunk = [0u8; 256];
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                break;
            }
            port.set_timeout(left)?;
            match port.read(&mut chunk) {
                Ok(0) => break,
                Ok(n) => received.extend_from_slice(&chunk[..n]),
                Err(e) if e.kind() == io::ErrorKind::TimedOut => break,
                Err(e) => return Err(e.into()),
            }
        }
        Ok(status_packet_ids::<P>(&received))
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

    fn read_with_error(
        &self,
        port: &mut dyn SerialPort,
        id: u8,
        addr: u8,
        length: u8,
    ) -> Result<(Vec<u8>, u8)> {
        self.send_instruction_packet(port, P::read_packet(id, addr, length).as_ref())?;
        self.read_status_packet(port, id)
            .map(|sp| (sp.params().to_vec(), sp.error_byte()))
    }

    fn write_with_error(
        &self,
        port: &mut dyn SerialPort,
        id: u8,
        addr: u8,
        data: &[u8],
    ) -> Result<u8> {
        self.send_instruction_packet(port, P::write_packet(id, addr, data).as_ref())?;
        self.read_status_packet(port, id).map(|sp| sp.error_byte())
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
    fn sync_read_with_error(
        &self,
        port: &mut dyn SerialPort,
        ids: &[u8],
        addr: u8,
        length: u8,
    ) -> Result<Vec<(Vec<u8>, u8)>> {
        self.send_instruction_packet(port, P::sync_read_packet(ids, addr, length).as_ref())?;
        let mut result = Vec::with_capacity(ids.len());
        for id in ids {
            let sp = self.read_status_packet(port, *id)?;
            result.push((sp.params().to_vec(), sp.error_byte()));
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

    const MAX_FLUSH_RETRIES: usize = 3;
    const FLUSH_RETRY_DELAY_MS: u64 = 5;

    fn flush_if_needed(
        &self,
        port: &mut dyn SerialPort,
        max_retries: usize,
        flush_after_delay: Duration,
    ) -> Result<()> {
        for _attempt in 1..=max_retries {
            if self.is_input_buffer_empty(port)? {
                return Ok(());
            }
            // log::warn!(
            //     "Input buffer not empty before sending instruction, flushing... (retry {}/{})",
            //     attempt,
            //     max_retries
            // );
            self.flush(port)?;
            std::thread::sleep(flush_after_delay);
        }
        if self.is_input_buffer_empty(port)? {
            Ok(())
        } else {
            // log::error!("Could not flush input buffer before sending instruction");
            Err(Box::new(CommunicationErrorKind::TimeoutError))
        }
    }

    fn send_instruction_packet(
        &self,
        port: &mut dyn SerialPort,
        packet: &dyn InstructionPacket<P>,
    ) -> Result<()> {
        // Before we send an instruction
        // The input buffer should always be empty
        // (if not, it means that an old corrupted message need to be flushed)
        self.flush_if_needed(
            port,
            Self::MAX_FLUSH_RETRIES,
            Duration::from_millis(Self::FLUSH_RETRY_DELAY_MS),
        )?;

        // log::debug!(">>> {:?}", packet.to_bytes());

        match port.write_all(&packet.to_bytes()) {
            Ok(_) => Ok(()),
            Err(_) => Err(Box::new(CommunicationErrorKind::TimeoutError)),
        }
    }
    /// Read one status packet off the wire, header included, without interpreting it.
    fn read_status_packet_bytes(&self, port: &mut dyn SerialPort) -> Result<Vec<u8>> {
        let mut header = vec![0u8; P::HEADER_SIZE];
        port.read_exact(&mut header)?;

        let payload_size = P::get_payload_size(&header)?;
        let mut payload = vec![0u8; payload_size];
        port.read_exact(&mut payload)?;

        let mut data = Vec::new();
        data.extend(header);
        data.extend(payload);

        // log::debug!("<<< {data:?}");

        Ok(data)
    }

    fn read_status_packet(
        &self,
        port: &mut dyn SerialPort,
        sender_id: u8,
    ) -> Result<Box<dyn StatusPacket<P>>> {
        let data = self.read_status_packet_bytes(port)?;
        P::status_packet(&data, sender_id)
    }

    fn is_input_buffer_empty(&self, port: &mut dyn SerialPort) -> Result<bool> {
        let n = port.bytes_to_read()? as usize;
        Ok(n == 0)
    }

    fn flush(&self, port: &mut dyn SerialPort) -> Result<()> {
        let n = port.bytes_to_read()? as usize;
        if n > 0 {
            // log::info!("Needed to flush serial port ({n} bytes)...");
            let mut buff = vec![0u8; n];
            port.read_exact(&mut buff)?;
        }

        Ok(())
    }
}

use std::{
    fmt, io,
    time::{Duration, Instant},
};

/// The error field of a status packet.
///
/// A motor can answer a request it could serve while still reporting a fault, so this
/// travels beside the data rather than in place of it. The byte is kept raw because its
/// layout depends on the protocol; the accessors below name the two readings.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct StatusError(u8);

impl StatusError {
    /// Wrap a status packet's error byte.
    pub fn new(byte: u8) -> Self {
        StatusError(byte)
    }

    /// The byte as it arrived.
    pub fn byte(self) -> u8 {
        self.0
    }

    /// Whether the motor reported nothing at all.
    pub fn is_ok(self) -> bool {
        self.0 == 0
    }

    /// Protocol v1 reading: the conditions the motor is reporting.
    ///
    /// On v1 the byte is a bitfield of motor conditions, so several can be set at once.
    /// Meaningless on a v2 motor, where the same bits mean something else entirely.
    pub fn v1_conditions(self) -> Vec<DynamixelErrorV1> {
        DynamixelErrorV1::from_byte(self.0)
    }

    /// Protocol v2 reading: the instruction error number, 0 when there is none.
    ///
    /// On v2 bits 0-6 are a number, not a bitfield: 1 Result Fail, 2 Instruction Error,
    /// 3 CRC Error, 4 Data Range Error, 5 Data Length Error, 6 Data Limit Error,
    /// 7 Access Error.
    pub fn v2_instruction_error(self) -> u8 {
        self.0 & 0x7F
    }

    /// Protocol v2 reading: the alert bit.
    ///
    /// Set means a hardware fault is latched, and says nothing about which one -- that
    /// lives in the motor's Hardware Error Status register.
    pub fn v2_alert(self) -> bool {
        self.0 & 0x80 != 0
    }
}

impl fmt::Display for StatusError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        if self.is_ok() {
            write!(f, "no error")
        } else {
            write!(f, "status error 0x{:02X}", self.0)
        }
    }
}

/// A motor answered, and its status packet reported an error: what it was asked to do may
/// not have happened. The status is kept raw, as in [`StatusError`], since its meaning
/// depends on the protocol.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotorError {
    pub id: u8,
    pub status: StatusError,
}

impl MotorError {
    /// `Ok` when `status` reports no error, else the error of motor `id`.
    pub(crate) fn check(id: u8, status: StatusError) -> Result<()> {
        if status.is_ok() {
            Ok(())
        } else {
            Err(Box::new(MotorError { id, status }))
        }
    }
}

impl fmt::Display for MotorError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "motor {} answered with {}", self.id, self.status)
    }
}

impl std::error::Error for MotorError {}

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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fake_port::FakePort;
    use std::time::Instant;

    #[test]
    fn status_error_reads_v1_as_a_bitfield() {
        let e = StatusError::new(0x24);
        assert!(!e.is_ok());
        assert_eq!(e.byte(), 0x24);
        assert_eq!(
            e.v1_conditions(),
            vec![DynamixelErrorV1::Overheating, DynamixelErrorV1::Overload]
        );
    }

    #[test]
    fn status_error_reads_v2_as_a_number_plus_alert() {
        // 0x83 is alert set with instruction error 3, not bits 0, 1 and 7.
        let e = StatusError::new(0x83);
        assert_eq!(e.v2_instruction_error(), 3);
        assert!(e.v2_alert());

        // Alert alone: a hardware fault is latched, no instruction error.
        let e = StatusError::new(0x80);
        assert_eq!(e.v2_instruction_error(), 0);
        assert!(e.v2_alert());
    }

    #[test]
    fn status_error_zero_is_ok() {
        let e = StatusError::default();
        assert!(e.is_ok());
        assert!(e.v1_conditions().is_empty());
        assert_eq!(e.v2_instruction_error(), 0);
        assert!(!e.v2_alert());
    }

    const POST_DELAY: Duration = Duration::from_millis(20);

    /// One v1 status packet per motor, each carrying the single byte 0x20.
    /// Checksum is !(id + length + error + params), e.g. !(0x0A + 0x03 + 0x20) = 0xD2.
    const SYNC_READ_RESPONSE: [u8; 21] = [
        0xFF, 0xFF, 0x0A, 0x03, 0x00, 0x20, 0xD2, // id 10
        0xFF, 0xFF, 0x0B, 0x03, 0x00, 0x20, 0xD1, // id 11
        0xFF, 0xFF, 0x0C, 0x03, 0x00, 0x20, 0xD0, // id 12
    ];

    #[test]
    fn sync_read_honours_the_post_delay() {
        let dph = DynamixelProtocolHandler::v1().with_post_delay(POST_DELAY);
        let mut port = FakePort::new(vec![SYNC_READ_RESPONSE.to_vec()]);

        let start = Instant::now();
        let values = dph.sync_read(&mut port, &[10, 11, 12], 43, 1).unwrap();
        let elapsed = start.elapsed();

        assert_eq!(values, vec![vec![0x20], vec![0x20], vec![0x20]]);
        assert!(
            elapsed >= POST_DELAY,
            "sync_read returned after {elapsed:?}, before the {POST_DELAY:?} post delay"
        );
    }

    #[test]
    fn sync_write_honours_the_post_delay() {
        let dph = DynamixelProtocolHandler::v1().with_post_delay(POST_DELAY);
        let mut port = FakePort::new(vec![]);

        let start = Instant::now();
        dph.sync_write(&mut port, &[40, 41], 25, &[vec![0], vec![1]])
            .unwrap();
        let elapsed = start.elapsed();

        assert!(
            elapsed >= POST_DELAY,
            "sync_write returned after {elapsed:?}, before the {POST_DELAY:?} post delay"
        );
    }

    #[test]
    fn a_failed_transaction_still_honours_the_post_delay() {
        // The port answers nothing, so the read times out. The bus was used all the
        // same, and an immediate retry is what the delay exists to space out.
        let dph = DynamixelProtocolHandler::v1().with_post_delay(POST_DELAY);
        let mut port = FakePort::new(vec![]);

        let start = Instant::now();
        assert!(dph.sync_read(&mut port, &[10], 43, 1).is_err());
        let elapsed = start.elapsed();

        assert!(
            elapsed >= POST_DELAY,
            "sync_read returned after {elapsed:?}, before the {POST_DELAY:?} post delay"
        );
    }

    #[test]
    fn a_broadcast_ping_lists_every_motor_that_answers() {
        // Motors 1 and 3 answer; motor 2's status packet arrives with a broken checksum.
        let answers = [
            [0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC],
            [0xFF, 0xFF, 0x02, 0x02, 0x00, 0x00],
            [0xFF, 0xFF, 0x03, 0x02, 0x00, 0xFA],
        ]
        .concat();
        let mut port = FakePort::new(vec![answers]);
        let written = port.written();
        let settings = port.settings();

        let ids = DynamixelProtocolHandler::v1()
            .broadcast_ping(&mut port)
            .unwrap();

        assert_eq!(ids, [1, 3]);
        // One ping, to the broadcast id.
        assert_eq!(
            *written.lock().unwrap(),
            [vec![0xFF, 0xFF, 0xFE, 0x02, 0x01, 0xFE]]
        );
        // The port's own timeout is back once the window closes.
        let timeouts = &settings.lock().unwrap().timeouts;
        assert_eq!(timeouts.last(), timeouts.first());
    }

    #[test]
    fn a_broadcast_ping_resyncs_past_bytes_that_do_not_frame_a_packet() {
        // Motor 1, a stray byte, a packet cut short after its header, then motor 3.
        let answers = [
            &[0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC][..],
            &[0x00],
            &[0xFF, 0xFF, 0x02, 0x02],
            &[0xFF, 0xFF, 0x03, 0x02, 0x00, 0xFA],
        ]
        .concat();
        let mut port = FakePort::new(vec![answers]);

        let ids = DynamixelProtocolHandler::v1()
            .broadcast_ping(&mut port)
            .unwrap();

        assert_eq!(ids, [1, 3]);
    }

    #[test]
    fn a_broadcast_ping_reads_protocol_v2_status_packets() {
        let answer = vec![
            0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x08, 0x00, 0x55, 0x00, 0xA6, 0x00, 0x00, 0x00, 0x8C,
            0xC0,
        ];
        let mut port = FakePort::new(vec![answer]);

        let ids = DynamixelProtocolHandler::v2()
            .broadcast_ping(&mut port)
            .unwrap();

        assert_eq!(ids, [1]);
    }

    #[test]
    fn the_broadcast_ping_window_leaves_every_id_its_turn() {
        // 253 ids at 3 ms each, the 16 ms of a USB latency timer, and 253 six-byte status
        // packets at 1 Mbps.
        assert_eq!(
            DynamixelProtocolHandler::v1().broadcast_ping_window(1_000_000),
            Duration::from_micros(253 * 3_000 + 16_000 + 253 * 6 * 10)
        );
    }

    #[test]
    fn no_post_delay_configured_means_no_sleep() {
        let dph = DynamixelProtocolHandler::v1();
        let mut port = FakePort::new(vec![SYNC_READ_RESPONSE.to_vec()]);

        let start = Instant::now();
        dph.sync_read(&mut port, &[10, 11, 12], 43, 1).unwrap();

        assert!(start.elapsed() < POST_DELAY);
    }
}
