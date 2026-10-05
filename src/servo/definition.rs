//! Register access by name through a servo definition, for any handler and port.
//!
//! A controller owns a port and speaks one definition. A bus can mix definitions, and
//! even protocols: Reachy Mini has STS3215 (v1) and XL330 (v2) motors on one port. The
//! functions here take the protocol handler and the port for each call, like the typed
//! functions of each servo module, so the caller picks the definition and the handler
//! motor by motor. [`crate::bus::Bus`] does that for a whole bus.

use std::collections::BTreeMap;

use serialport::SerialPort;

use crate::servo::{scan_timeout, RegisterError, RegisterInfo, ServoInfo, WordOrder};
use crate::{DynamixelProtocolHandler, Result, StatusError};

/// A servo definition as a value: its name, the protocol it speaks, what it states
/// about itself and its registers.
///
/// Every servo module has its own as `DEFINITION`; on Python, the static `definition()`
/// of each controller class returns it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(
    feature = "python",
    pyo3_stub_gen::derive::gen_stub_pyclass,
    pyo3::pyclass(frozen, eq, hash, from_py_object)
)]
pub struct ServoDefinition {
    pub name: &'static str,
    /// The Dynamixel protocol version the servo speaks: 1 or 2.
    pub protocol: u8,
    pub info: ServoInfo,
    pub registers: &'static [RegisterInfo],
    /// The servo models this definition covers, as (name, model number).
    pub models: &'static [(&'static str, u16)],
}

impl ServoDefinition {
    /// Look up a register by name, as spelled in `registers`.
    pub fn register(&self, name: &str) -> Option<RegisterInfo> {
        self.registers.iter().copied().find(|r| r.name == name)
    }

    fn named(&self, name: &str) -> Result<RegisterInfo> {
        Ok(self
            .register(name)
            .ok_or_else(|| RegisterError::Unknown(name.to_string()))?)
    }

    fn check_protocol(&self, dph: &DynamixelProtocolHandler) -> Result<()> {
        if dph.protocol_version() != self.protocol {
            return Err(RegisterError::Protocol {
                servo: self.name,
                servo_protocol: self.protocol,
                handler_protocol: dph.protocol_version(),
            }
            .into());
        }
        Ok(())
    }

    /// Read register `name` of motor `id` as an integer, decoded from this definition's
    /// word order and the register's sign encoding.
    pub fn read_register(
        &self,
        dph: &DynamixelProtocolHandler,
        port: &mut dyn SerialPort,
        id: u8,
        name: &str,
    ) -> Result<i64> {
        self.check_protocol(dph)?;
        let reg = self.named(name)?;
        let bytes = dph.read(port, id, reg.addr, reg.size)?;
        Ok(reg.decode(self.info.word_order, &bytes)?)
    }

    /// Same as [`read_register`](Self::read_register), plus the status packet's error
    /// field.
    pub fn read_register_with_error(
        &self,
        dph: &DynamixelProtocolHandler,
        port: &mut dyn SerialPort,
        id: u8,
        name: &str,
    ) -> Result<(i64, StatusError)> {
        self.check_protocol(dph)?;
        let reg = self.named(name)?;
        let (bytes, error) = dph.read_with_error(port, id, reg.addr, reg.size)?;
        Ok((reg.decode(self.info.word_order, &bytes)?, error))
    }

    /// Write `value` to register `name` of motor `id`, laid out in this definition's word
    /// order and the register's sign encoding. A value that does not fit the register
    /// fails before anything reaches the bus.
    pub fn write_register(
        &self,
        dph: &DynamixelProtocolHandler,
        port: &mut dyn SerialPort,
        id: u8,
        name: &str,
        value: i64,
    ) -> Result<()> {
        self.check_protocol(dph)?;
        let reg = self.named(name)?;
        dph.write(
            port,
            id,
            reg.addr,
            &reg.encode(self.info.word_order, value)?,
        )
    }

    /// Same as [`write_register`](Self::write_register), plus the status packet's error
    /// field.
    pub fn write_register_with_error(
        &self,
        dph: &DynamixelProtocolHandler,
        port: &mut dyn SerialPort,
        id: u8,
        name: &str,
        value: i64,
    ) -> Result<StatusError> {
        self.check_protocol(dph)?;
        let reg = self.named(name)?;
        dph.write_with_error(
            port,
            id,
            reg.addr,
            &reg.encode(self.info.word_order, value)?,
        )
    }

    /// Sync read register `name` from `ids`, each value decoded like
    /// [`read_register`](Self::read_register).
    ///
    /// A servo whose firmware has no Sync Read (`supports_sync_read` false: the Feetech
    /// SCS series) is read one id at a time instead, in the order asked. The values come
    /// back the same way, but from one transaction per id rather than one for the bus.
    pub fn sync_read_register(
        &self,
        dph: &DynamixelProtocolHandler,
        port: &mut dyn SerialPort,
        ids: &[u8],
        name: &str,
    ) -> Result<Vec<i64>> {
        sync_read_register(dph, port, &self.motors(ids), name)
    }

    /// Same as [`sync_read_register`](Self::sync_read_register), plus each motor's error
    /// field.
    pub fn sync_read_register_with_error(
        &self,
        dph: &DynamixelProtocolHandler,
        port: &mut dyn SerialPort,
        ids: &[u8],
        name: &str,
    ) -> Result<Vec<(i64, StatusError)>> {
        sync_read_register_with_error(dph, port, &self.motors(ids), name)
    }

    /// Sync write `values` to register `name` of `ids`, one value per id, each encoded
    /// like [`write_register`](Self::write_register).
    pub fn sync_write_register(
        &self,
        dph: &DynamixelProtocolHandler,
        port: &mut dyn SerialPort,
        ids: &[u8],
        name: &str,
        values: &[i64],
    ) -> Result<()> {
        sync_write_register(dph, port, &self.motors(ids), name, values)
    }

    /// Which of `ids` answer, with their model number read as this definition lays it
    /// out.
    ///
    /// One Model Number read per id, so presence and identity cost a single round trip.
    /// An absent id costs a timeout, so the sweep runs under one sized to the baud rate,
    /// see [`scan_timeout`], and puts the port's timeout back afterwards. An id that
    /// answers with anything the protocol cannot parse counts as absent.
    pub fn scan(
        &self,
        dph: &DynamixelProtocolHandler,
        port: &mut dyn SerialPort,
        ids: &[u8],
    ) -> Result<BTreeMap<u8, u16>> {
        self.check_protocol(dph)?;
        let reg = self.named("model_number")?;
        let timeout = port.timeout();
        port.set_timeout(scan_timeout(port.baud_rate()?))?;
        let found = ids
            .iter()
            .filter_map(|&id| self.model_number(dph, port, reg, id))
            .collect();
        port.set_timeout(timeout)?;
        Ok(found)
    }

    /// Which of `ids` answer, with their model number read as this definition lays it
    /// out: one broadcast ping, then a Model Number read of each id asked that answered,
    /// under the port's own timeout.
    ///
    /// The ping listens for its whole window however few ids are asked
    /// ([`broadcast_ping_window`](DynamixelProtocolHandler::broadcast_ping_window), about
    /// 0.8 s at 1 Mbps), so this pays off over many ids, where [`scan`](Self::scan) costs a
    /// timeout per absent id, and behind a USB adapter whose latency timer outlasts the
    /// short timeout `scan` uses. A servo that does not answer a broadcast ping
    /// (`supports_broadcast_ping`) is refused before anything is sent.
    pub fn broadcast_scan(
        &self,
        dph: &DynamixelProtocolHandler,
        port: &mut dyn SerialPort,
        ids: &[u8],
    ) -> Result<BTreeMap<u8, u16>> {
        self.check_protocol(dph)?;
        let reg = self.named("model_number")?;
        if !self.info.supports_broadcast_ping {
            return Err(RegisterError::BroadcastPing(self.name).into());
        }
        let answered = dph.broadcast_ping(port)?;
        Ok(ids
            .iter()
            .filter(|id| answered.contains(id))
            .filter_map(|&id| self.model_number(dph, port, reg, id))
            .collect())
    }

    /// The model number of motor `id`, or `None` when it does not answer with one.
    fn model_number(
        &self,
        dph: &DynamixelProtocolHandler,
        port: &mut dyn SerialPort,
        reg: RegisterInfo,
        id: u8,
    ) -> Option<(u8, u16)> {
        let bytes = dph.read(port, id, reg.addr, reg.size).ok()?;
        let model = reg.decode(self.info.word_order, &bytes).ok()?;
        Some((id, model as u16))
    }

    fn motors(&self, ids: &[u8]) -> Vec<(u8, ServoDefinition)> {
        ids.iter().map(|&id| (id, *self)).collect()
    }
}

/// Register `name` of each motor, which one instruction can only reach if it sits at the
/// same address and size on all of them, through a handler of their protocol.
fn resolve(
    dph: &DynamixelProtocolHandler,
    motors: &[(u8, ServoDefinition)],
    name: &str,
) -> Result<Vec<(RegisterInfo, WordOrder)>> {
    let mut regs = Vec::with_capacity(motors.len());
    for (_, definition) in motors {
        definition.check_protocol(dph)?;
        regs.push((definition.named(name)?, definition.info.word_order));
    }
    if regs
        .windows(2)
        .any(|pair| (pair[0].0.addr, pair[0].0.size) != (pair[1].0.addr, pair[1].0.size))
    {
        return Err(RegisterError::Layout(name.to_string()).into());
    }
    Ok(regs)
}

fn answer_sync_read(motors: &[(u8, ServoDefinition)]) -> bool {
    motors
        .iter()
        .all(|(_, definition)| definition.info.supports_sync_read)
}

fn ids(motors: &[(u8, ServoDefinition)]) -> Vec<u8> {
    motors.iter().map(|&(id, _)| id).collect()
}

/// Sync read register `name` from `motors`, each an id with its own definition, in one
/// instruction: the XL430 and XL330 of a Koch arm, say.
///
/// Every definition must speak the handler's protocol and have the register at the same
/// address and size (`RegisterError::Layout` otherwise); each value is decoded through its
/// motor's definition. When one of them has no Sync Read, the motors are read one at a
/// time instead, in the order given.
pub fn sync_read_register(
    dph: &DynamixelProtocolHandler,
    port: &mut dyn SerialPort,
    motors: &[(u8, ServoDefinition)],
    name: &str,
) -> Result<Vec<i64>> {
    if !answer_sync_read(motors) {
        return motors
            .iter()
            .map(|&(id, definition)| definition.read_register(dph, port, id, name))
            .collect();
    }
    let regs = resolve(dph, motors, name)?;
    let Some(&(first, _)) = regs.first() else {
        return Ok(Vec::new());
    };
    let answers = dph.sync_read(port, &ids(motors), first.addr, first.size)?;
    regs.iter()
        .zip(answers)
        .map(|(&(reg, order), bytes)| Ok(reg.decode(order, &bytes)?))
        .collect()
}

/// Same as [`sync_read_register`], plus each motor's error field.
pub fn sync_read_register_with_error(
    dph: &DynamixelProtocolHandler,
    port: &mut dyn SerialPort,
    motors: &[(u8, ServoDefinition)],
    name: &str,
) -> Result<Vec<(i64, StatusError)>> {
    if !answer_sync_read(motors) {
        return motors
            .iter()
            .map(|&(id, definition)| definition.read_register_with_error(dph, port, id, name))
            .collect();
    }
    let regs = resolve(dph, motors, name)?;
    let Some(&(first, _)) = regs.first() else {
        return Ok(Vec::new());
    };
    let answers = dph.sync_read_with_error(port, &ids(motors), first.addr, first.size)?;
    regs.iter()
        .zip(answers)
        .map(|(&(reg, order), (bytes, error))| Ok((reg.decode(order, &bytes)?, error)))
        .collect()
}

/// Sync write `values` to register `name` of `motors`, one value per motor, in one
/// instruction; each value is laid out through its motor's definition. The same rules
/// as [`sync_read_register`] apply to the definitions.
pub fn sync_write_register(
    dph: &DynamixelProtocolHandler,
    port: &mut dyn SerialPort,
    motors: &[(u8, ServoDefinition)],
    name: &str,
    values: &[i64],
) -> Result<()> {
    match SyncWrite::encode(dph, motors, name, values)? {
        Some(write) => write.send(dph, port),
        None => Ok(()),
    }
}

/// A Sync Write ready to go: every check done and every value encoded, so sending it
/// can only fail on the bus.
pub(crate) struct SyncWrite {
    ids: Vec<u8>,
    addr: u8,
    data: Vec<Vec<u8>>,
}

impl SyncWrite {
    /// The Sync Write that puts `values` in register `name` of `motors`, or `None` when
    /// there is no motor. Fails with a [`RegisterError`], and sends nothing.
    pub(crate) fn encode(
        dph: &DynamixelProtocolHandler,
        motors: &[(u8, ServoDefinition)],
        name: &str,
        values: &[i64],
    ) -> Result<Option<SyncWrite>> {
        if values.len() != motors.len() {
            return Err(RegisterError::ValueCount {
                ids: motors.len(),
                values: values.len(),
            }
            .into());
        }
        let regs = resolve(dph, motors, name)?;
        let Some(&(first, _)) = regs.first() else {
            return Ok(None);
        };
        let data = regs
            .iter()
            .zip(values)
            .map(|(&(reg, order), &value)| reg.encode(order, value))
            .collect::<std::result::Result<Vec<_>, _>>()?;
        Ok(Some(SyncWrite {
            ids: ids(motors),
            addr: first.addr,
            data,
        }))
    }

    pub(crate) fn send(
        &self,
        dph: &DynamixelProtocolHandler,
        port: &mut dyn SerialPort,
    ) -> Result<()> {
        dph.sync_write(port, &self.ids, self.addr, &self.data)
    }
}

/// Run `op`, and run it again up to `retries` more times while it fails on the bus.
///
/// A timeout or a corrupted status packet is worth another try, and each attempt starts
/// with the pre-send flush. A [`RegisterError`] is not: it is found before anything
/// reaches the bus, and fails the same way every time.
pub(crate) fn retrying<T>(retries: u32, mut op: impl FnMut() -> Result<T>) -> Result<T> {
    let mut attempt = 0;
    loop {
        match op() {
            Err(e) if attempt < retries && !e.is::<RegisterError>() => attempt += 1,
            result => return result,
        }
    }
}

#[cfg(feature = "python")]
#[pyo3_stub_gen::derive::gen_stub_pymethods]
#[pyo3::pymethods]
impl ServoDefinition {
    /// The servo's name, as its controller class spells it (`XL330`).
    #[getter]
    fn name(&self) -> &'static str {
        self.name
    }

    /// The Dynamixel protocol version the servo speaks: 1 or 2.
    #[getter]
    fn protocol(&self) -> u8 {
        self.protocol
    }

    /// Model numbers of the servos this definition covers, as {name: number}, by name.
    #[getter]
    fn models(&self) -> BTreeMap<&'static str, u16> {
        self.models.iter().copied().collect()
    }

    /// Encoder steps per turn, or `None` when the servo does not count steps.
    #[getter]
    fn resolution(&self) -> Option<u32> {
        self.info.resolution
    }

    /// Byte order of multi-byte registers on the wire: "little" or "big".
    #[getter]
    fn word_order(&self) -> &'static str {
        self.info.word_order.as_str()
    }

    /// Whether the firmware answers the Sync Read instruction.
    #[getter]
    fn supports_sync_read(&self) -> bool {
        self.info.supports_sync_read
    }

    /// Whether the firmware answers a ping sent to the broadcast id, which
    /// `broadcast_scan` relies on.
    #[getter]
    fn supports_broadcast_ping(&self) -> bool {
        self.info.supports_broadcast_ping
    }

    /// Serial rates the servo can be set to, as {baud rate: register value}, slowest
    /// first.
    #[getter]
    fn baudrates(&self) -> BTreeMap<u32, u8> {
        self.info.baudrates.iter().copied().collect()
    }

    /// The baud rate a new servo answers at, or `None` when the definition does not say.
    #[getter]
    fn factory_baudrate(&self) -> Option<u32> {
        self.info.factory_baudrate
    }

    /// How the homing offset moves the position the servo reports,
    /// `present = actual + sign * homing_offset`: -1 on Feetech, 1 on Dynamixel, `None`
    /// without a homing offset.
    #[getter]
    fn homing_offset_sign(&self) -> Option<i8> {
        self.info.homing_offset_sign
    }

    /// The values of the operating mode register, as {name: value}. A name means the
    /// same mode on every servo that has it (`position`, `velocity`, `pwm`).
    #[getter]
    fn operating_modes(&self) -> BTreeMap<&'static str, u8> {
        self.info.operating_modes.iter().copied().collect()
    }

    /// Whether the `lock` register guards the EEPROM and opens and closes at will, as on
    /// Feetech servos; `Bus.set_torque` then moves it with the torque.
    #[getter]
    fn eeprom_lock(&self) -> bool {
        self.info.eeprom_lock
    }

    /// Every register of this servo, in declaration order.
    #[pyo3(name = "registers")]
    fn py_registers(&self) -> Vec<RegisterInfo> {
        self.registers.to_vec()
    }

    /// Look up a register by name, as spelled in `registers()`; `None` when this servo
    /// has no such register.
    #[pyo3(name = "register")]
    fn py_register(&self, name: &str) -> Option<RegisterInfo> {
        self.register(name)
    }

    fn __repr__(&self) -> String {
        format!("ServoDefinition('{}')", self.name)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fake_port::FakePort;
    use crate::servo::feetech::{scs0009, sts3215};

    #[test]
    fn a_definition_reads_through_the_handler_and_port_it_is_given() {
        // Present position 2048 from motor 1, little-endian.
        let mut port = FakePort::new(vec![vec![0xFF, 0xFF, 0x01, 0x04, 0x00, 0x00, 0x08, 0xF2]]);
        let written = port.written();

        let value = sts3215::DEFINITION
            .read_register(
                &DynamixelProtocolHandler::v1(),
                &mut port,
                1,
                "present_position",
            )
            .unwrap();
        assert_eq!(value, 2048);

        // A handler of another protocol is refused before anything is sent.
        let err = sts3215::DEFINITION
            .read_register(
                &DynamixelProtocolHandler::v2(),
                &mut port,
                1,
                "present_position",
            )
            .unwrap_err();
        assert!(matches!(
            err.downcast_ref::<RegisterError>(),
            Some(RegisterError::Protocol { .. })
        ));
        assert_eq!(written.lock().unwrap().len(), 1);
    }

    #[test]
    fn one_sync_write_reaches_motors_of_two_definitions_sharing_a_layout() {
        let mut port = FakePort::new(vec![]);
        let written = port.written();
        let v1 = DynamixelProtocolHandler::v1();
        let motors = [(1, sts3215::DEFINITION), (2, scs0009::DEFINITION)];

        // One Sync Write at address 42, two bytes each: -100 sign-magnitude and
        // little-endian for the STS3215, 0x1234 big-endian for the SCS0009.
        sync_write_register(&v1, &mut port, &motors, "goal_position", &[-100, 0x1234]).unwrap();
        assert_eq!(
            written.lock().unwrap()[0][5..13],
            [0x2A, 0x02, 0x01, 0x64, 0x80, 0x02, 0x12, 0x34]
        );

        // The lock register is at 55 on the STS3215 and 48 on the SCS0009: no single
        // instruction reaches both. A value per motor is needed too. Nothing is sent.
        let layout = sync_write_register(&v1, &mut port, &motors, "lock", &[0, 0]).unwrap_err();
        assert!(matches!(
            layout.downcast_ref::<RegisterError>(),
            Some(RegisterError::Layout(_))
        ));
        let count =
            sync_write_register(&v1, &mut port, &motors, "goal_position", &[0]).unwrap_err();
        assert!(matches!(
            count.downcast_ref::<RegisterError>(),
            Some(RegisterError::ValueCount { ids: 2, values: 1 })
        ));
        assert_eq!(written.lock().unwrap().len(), 1);
    }
}
