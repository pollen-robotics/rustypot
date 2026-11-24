//! Motors of several definitions, and of both protocols, on one serial port.
//!
//! A controller speaks one definition over one protocol. A robot's bus often does not:
//! LeRobot's Koch arm has XL430 and XL330 motors on one port, and Reachy Mini has
//! STS3215 (protocol v1) and XL330 (protocol v2) motors on one port. A [`Bus`] holds the
//! port, a handler for each protocol and the definition of each motor, and calls the
//! definition's functions with the handler that matches it.
//!
//! With [`ServoDefinition`], it is rustypot's higher-level, family-agnostic API, designed
//! mostly for the LeRobot library: registers by name, the facts about each servo, finding
//! and setting up motors, the torque of a whole bus. Some of its choices are LeRobot
//! conventions rather than vendor rules, such as the Feetech lock following the torque in
//! [`Bus::set_torque`]. The per-servo controllers stay the API underneath.

use std::collections::BTreeMap;
use std::time::Duration;

use serialport::SerialPort;

use crate::servo::definition::{self, retrying, ServoDefinition, SyncWrite};
use crate::servo::RegisterError;
use crate::{DynamixelProtocolHandler, Result, StatusError, SyncReadError};

/// The motors of `ids` sharing a protocol and the address and size of one register, which
/// is what one Sync Read or Sync Write can reach: each as its position in `ids`, its id
/// and its definition.
type Group = (u8, Vec<(usize, (u8, ServoDefinition))>);

/// A serial port, a protocol handler for each protocol, and the definition of each motor.
pub struct Bus {
    serial_port: Box<dyn SerialPort>,
    /// Indexed by protocol version minus one.
    handlers: [DynamixelProtocolHandler; 2],
    motors: BTreeMap<u8, ServoDefinition>,
}

impl Bus {
    /// A bus over `serial_port` whose motors are `motors`, as id -> definition. Motors of
    /// both protocols can share the port.
    pub fn new(serial_port: Box<dyn SerialPort>, motors: BTreeMap<u8, ServoDefinition>) -> Self {
        Bus {
            serial_port,
            handlers: [
                DynamixelProtocolHandler::v1(),
                DynamixelProtocolHandler::v2(),
            ],
            motors,
        }
    }

    /// The definition of motor `id`.
    pub fn definition(&self, id: u8) -> Result<ServoDefinition> {
        Ok(self
            .motors
            .get(&id)
            .copied()
            .ok_or(RegisterError::UnknownMotor(id))?)
    }

    /// Read register `name` of motor `id`, through its definition.
    pub fn read_register(&mut self, id: u8, name: &str) -> Result<i64> {
        let definition = self.definition(id)?;
        let dph = &self.handlers[definition.protocol as usize - 1];
        definition.read_register(dph, self.serial_port.as_mut(), id, name)
    }

    /// Same as [`read_register`](Self::read_register), plus the status packet's error field.
    pub fn read_register_with_error(&mut self, id: u8, name: &str) -> Result<(i64, StatusError)> {
        let definition = self.definition(id)?;
        let dph = &self.handlers[definition.protocol as usize - 1];
        definition.read_register_with_error(dph, self.serial_port.as_mut(), id, name)
    }

    /// Write `value` to register `name` of motor `id`, through its definition.
    pub fn write_register(&mut self, id: u8, name: &str, value: i64) -> Result<()> {
        let definition = self.definition(id)?;
        let dph = &self.handlers[definition.protocol as usize - 1];
        definition.write_register(dph, self.serial_port.as_mut(), id, name, value)
    }

    /// Same as [`write_register`](Self::write_register), plus the status packet's error field.
    pub fn write_register_with_error(
        &mut self,
        id: u8,
        name: &str,
        value: i64,
    ) -> Result<StatusError> {
        let definition = self.definition(id)?;
        let dph = &self.handlers[definition.protocol as usize - 1];
        definition.write_register_with_error(dph, self.serial_port.as_mut(), id, name, value)
    }

    /// `ids` split into what one instruction can reach: same protocol, and register
    /// `name` at the same address and size. Groups come in the order their first id is
    /// asked, and keep the order of their ids.
    fn groups(&self, ids: &[u8], name: &str) -> Result<Vec<Group>> {
        let mut groups: Vec<((u8, u8, u8), Group)> = Vec::new();
        for (position, &id) in ids.iter().enumerate() {
            let definition = self.definition(id)?;
            let reg = definition
                .register(name)
                .ok_or_else(|| RegisterError::Unknown(name.to_string()))?;
            let key = (definition.protocol, reg.addr, reg.size);
            let member = (position, (id, definition));
            match groups.iter_mut().find(|(k, _)| *k == key) {
                Some((_, (_, members))) => members.push(member),
                None => groups.push((key, (definition.protocol, vec![member]))),
            }
        }
        Ok(groups.into_iter().map(|(_, group)| group).collect())
    }

    /// Sync read register `name` from `ids`, in the order asked.
    ///
    /// One Sync Read for each group of motors sharing a protocol and the register's
    /// address and size, so the XL430 and XL330 of a Koch arm are read together, and the
    /// v1 and v2 motors of Reachy Mini in two instructions. A group with a motor that has
    /// no Sync Read is read one id at a time.
    pub fn sync_read_register(&mut self, ids: &[u8], name: &str) -> Result<Vec<i64>> {
        let mut values = vec![0; ids.len()];
        for (protocol, members) in self.groups(ids, name)? {
            let motors: Vec<_> = members.iter().map(|&(_, motor)| motor).collect();
            let dph = &self.handlers[protocol as usize - 1];
            let read =
                definition::sync_read_register(dph, self.serial_port.as_mut(), &motors, name)?;
            for (&(position, _), value) in members.iter().zip(read) {
                values[position] = value;
            }
        }
        Ok(values)
    }

    /// Same as [`sync_read_register`](Self::sync_read_register), plus each motor's error
    /// field.
    pub fn sync_read_register_with_error(
        &mut self,
        ids: &[u8],
        name: &str,
    ) -> Result<Vec<(i64, StatusError)>> {
        let mut values = vec![(0, StatusError::default()); ids.len()];
        for (protocol, members) in self.groups(ids, name)? {
            let motors: Vec<_> = members.iter().map(|&(_, motor)| motor).collect();
            let dph = &self.handlers[protocol as usize - 1];
            let read = definition::sync_read_register_with_error(
                dph,
                self.serial_port.as_mut(),
                &motors,
                name,
            )?;
            for (&(position, _), value) in members.iter().zip(read) {
                values[position] = value;
            }
        }
        Ok(values)
    }

    /// Read contiguous named registers from each motor, in the order asked.
    /// Registers must be listed in address order with no gaps.
    pub fn sync_read_block(&mut self, ids: &[u8], names: &[&str]) -> Result<Vec<Vec<i64>>> {
        Ok(self
            .sync_read_block_with_error(ids, names)?
            .into_iter()
            .map(|(values, _)| values)
            .collect())
    }

    /// Read contiguous named registers and each motor's status, in the order asked.
    /// Registers must be listed in address order and contiguous for each motor.
    /// Motors with different protocols or ranges are read in separate transactions.
    pub fn sync_read_block_with_error(
        &mut self,
        ids: &[u8],
        names: &[&str],
    ) -> Result<Vec<(Vec<i64>, StatusError)>> {
        if names.is_empty() {
            return Err(RegisterError::EmptyRegisterBlock.into());
        }
        let mut layouts = Vec::new();
        for &id in ids {
            let definition = self.definition(id)?;
            let registers: Vec<_> = names
                .iter()
                .map(|name| {
                    definition
                        .register(name)
                        .ok_or_else(|| RegisterError::Unknown((*name).to_string()))
                })
                .collect::<std::result::Result<_, _>>()?;
            let start = registers[0].addr;
            let mut end = u16::from(start);
            for register in &registers {
                if u16::from(register.addr) != end || register.size == 0 {
                    return Err(RegisterError::Layout(register.name.to_string()).into());
                }
                register.integer_width()?;
                end += u16::from(register.size);
            }
            let length = u8::try_from(end - u16::from(start))
                .map_err(|_| RegisterError::Layout(names[0].to_string()))?;
            layouts.push((definition, registers, start, length));
        }
        let mut groups: Vec<((u8, u8, u8), Vec<usize>)> = Vec::new();
        for (index, (definition, _, start, length)) in layouts.iter().enumerate() {
            let key = (definition.protocol, *start, *length);
            match groups.iter_mut().find(|(existing, _)| *existing == key) {
                Some((_, indices)) => indices.push(index),
                None => groups.push((key, vec![index])),
            }
        }
        let mut values = vec![(Vec::new(), StatusError::default()); ids.len()];
        for ((_, start, length), indices) in groups {
            let group_ids: Vec<_> = indices.iter().map(|&index| ids[index]).collect();
            let replies = self.sync_read_raw_with_error(&group_ids, start, length)?;
            for (index, (bytes, status)) in indices.into_iter().zip(replies) {
                if bytes.len() != usize::from(length) {
                    return Err(SyncReadError::boxed(
                        ids[index],
                        std::io::Error::new(
                            std::io::ErrorKind::InvalidData,
                            format!("expected {length} register bytes, got {}", bytes.len()),
                        )
                        .into(),
                    ));
                }
                let (definition, registers, _, _) = &layouts[index];
                let mut offset = 0;
                let mut decoded = Vec::with_capacity(registers.len());
                for register in registers {
                    let end = offset + usize::from(register.size);
                    decoded.push(register.decode(definition.info.word_order, &bytes[offset..end])?);
                    offset = end;
                }
                values[index] = (decoded, status);
            }
        }
        Ok(values)
    }

    /// Read `length` raw bytes starting at `addr` from each motor in `ids`, returning
    /// the bytes and status packet's error field in the order asked.
    ///
    /// Motors are grouped by protocol, with one Sync Read per protocol. If any motor
    /// in a group does not support Sync Read, that group is read one id at a time.
    /// Unknown ids return [`RegisterError::UnknownMotor`] before any instruction is sent.
    ///
    /// The caller must ensure `addr` and `length` mean the same thing on every motor.
    /// Raw bytes are not decoded, and register layouts are not checked.
    pub fn sync_read_raw_with_error(
        &mut self,
        ids: &[u8],
        addr: u8,
        length: u8,
    ) -> Result<Vec<(Vec<u8>, StatusError)>> {
        let mut groups: Vec<Group> = Vec::new();
        for (position, &id) in ids.iter().enumerate() {
            let definition = self.definition(id)?;
            let member = (position, (id, definition));
            match groups
                .iter_mut()
                .find(|(protocol, _)| *protocol == definition.protocol)
            {
                Some((_, members)) => members.push(member),
                None => groups.push((definition.protocol, vec![member])),
            }
        }
        let mut values = vec![(Vec::new(), StatusError::default()); ids.len()];
        for (protocol, members) in groups {
            let handler = &self.handlers[protocol as usize - 1];
            let replies = if members
                .iter()
                .all(|(_, (_, definition))| definition.info.supports_sync_read)
            {
                let motor_ids: Vec<_> = members.iter().map(|(_, (id, _))| *id).collect();
                handler.sync_read_with_error(self.serial_port.as_mut(), &motor_ids, addr, length)?
            } else {
                members
                    .iter()
                    .map(|(_, (id, _))| {
                        handler.read_with_error(self.serial_port.as_mut(), *id, addr, length)
                    })
                    .collect::<Result<Vec<_>>>()?
            };
            for ((position, _), reply) in members.into_iter().zip(replies) {
                values[position] = reply;
            }
        }
        Ok(values)
    }

    /// Sync write `values` to register `name` of `ids`, one value per id, with one Sync
    /// Write per group as in [`sync_read_register`](Self::sync_read_register).
    ///
    /// Every group is checked and encoded before the first one is sent, so a value that
    /// does not fit its register fails the call with nothing written. A bus failure on a
    /// later group leaves the earlier ones written; `with_retries` sends them all again.
    pub fn sync_write_register(&mut self, ids: &[u8], name: &str, values: &[i64]) -> Result<()> {
        if values.len() != ids.len() {
            return Err(RegisterError::ValueCount {
                ids: ids.len(),
                values: values.len(),
            }
            .into());
        }
        let mut writes = Vec::new();
        for (protocol, members) in self.groups(ids, name)? {
            let motors: Vec<_> = members.iter().map(|&(_, motor)| motor).collect();
            let group_values: Vec<_> = members
                .iter()
                .map(|&(position, _)| values[position])
                .collect();
            let dph = &self.handlers[protocol as usize - 1];
            if let Some(write) = SyncWrite::encode(dph, &motors, name, &group_values)? {
                writes.push((dph, write));
            }
        }
        for (dph, write) in writes {
            write.send(dph, self.serial_port.as_mut())?;
        }
        Ok(())
    }

    /// Which of `ids` answer, with their model number read as `definition` lays it out,
    /// over `definition`'s protocol. See [`ServoDefinition::scan`].
    pub fn scan(&mut self, definition: ServoDefinition, ids: &[u8]) -> Result<BTreeMap<u8, u16>> {
        let dph = &self.handlers[definition.protocol as usize - 1];
        definition.scan(dph, self.serial_port.as_mut(), ids)
    }

    /// [`scan`](Self::scan) every id `definition`'s protocol allows.
    pub fn scan_all(&mut self, definition: ServoDefinition) -> Result<BTreeMap<u8, u16>> {
        let ids: Vec<u8> = (0..=self.handlers[definition.protocol as usize - 1].max_id()).collect();
        self.scan(definition, &ids)
    }

    /// Which of `ids` answer, with their model number read as `definition` lays it out:
    /// one broadcast ping, then a read of each id asked that answered. See
    /// [`ServoDefinition::broadcast_scan`].
    pub fn broadcast_scan(
        &mut self,
        definition: ServoDefinition,
        ids: &[u8],
    ) -> Result<BTreeMap<u8, u16>> {
        let dph = &self.handlers[definition.protocol as usize - 1];
        definition.broadcast_scan(dph, self.serial_port.as_mut(), ids)
    }

    /// [`broadcast_scan`](Self::broadcast_scan) every id `definition`'s protocol allows.
    pub fn broadcast_scan_all(&mut self, definition: ServoDefinition) -> Result<BTreeMap<u8, u16>> {
        let ids: Vec<u8> = (0..=self.handlers[definition.protocol as usize - 1].max_id()).collect();
        self.broadcast_scan(definition, &ids)
    }

    /// Turn the torque of `ids` on or off. Every motor is tried, even after one fails,
    /// and a failure on the bus is tried again up to `retries` more times; the motors
    /// that still failed come back with their error. A motor that refuses a write has
    /// failed, with a [`StatusPacketError`](crate::StatusPacketError), and is not tried
    /// again; an alarm in its answer, as an overload, does not fail it.
    ///
    /// On a servo whose lock guards its EEPROM (`eeprom_lock`: the Feetech ones), the lock
    /// follows the torque, closed when it goes on and open when it goes off, so the EEPROM
    /// takes writes (a calibration, an id) as soon as the torque is off.
    pub fn set_torque(
        &mut self,
        ids: &[u8],
        enabled: bool,
        retries: u32,
    ) -> BTreeMap<u8, Box<dyn std::error::Error>> {
        let value = i64::from(enabled);
        let mut failed = BTreeMap::new();
        for &id in ids {
            let result = retrying(retries, || {
                let definition = self.definition(id)?;
                definition.check_status(
                    id,
                    self.write_register_with_error(id, "torque_enable", value)?,
                )?;
                if definition.info.eeprom_lock {
                    definition
                        .check_status(id, self.write_register_with_error(id, "lock", value)?)?;
                }
                Ok(())
            });
            if let Err(e) = result {
                failed.insert(id, e);
            }
        }
        failed
    }

    /// Zero the turn count of `ids` (Feetech's RESET, 0x0A, or Dynamixel's Clear, 0x10),
    /// every motor tried even after one fails, and return {id: error} for those that
    /// failed (empty when all went through): a servo not known to answer it
    /// (`supports_turn_count_reset`), a motor refusing it, or one not answering. A bus
    /// failure is tried again up to `retries` more times per motor. See
    /// [`ServoDefinition::reset_turn_count`]; send it with the torque off.
    pub fn reset_turn_count(
        &mut self,
        ids: &[u8],
        retries: u32,
    ) -> BTreeMap<u8, Box<dyn std::error::Error>> {
        let mut failed = BTreeMap::new();
        for &id in ids {
            let result = retrying(retries, || {
                let definition = self.definition(id)?;
                let dph = &self.handlers[definition.protocol as usize - 1];
                definition.reset_turn_count(dph, self.serial_port.as_mut(), id)
            });
            if let Err(e) = result {
                failed.insert(id, e);
            }
        }
        failed
    }

    /// Give motor `id` the id `new_id`, through `definition`. See
    /// [`ServoDefinition::change_id`]. Like [`scan`](Self::scan), it reaches motors the
    /// bus does not have: a new motor answers at its factory id.
    pub fn change_id(&mut self, definition: ServoDefinition, id: u8, new_id: u8) -> Result<()> {
        let dph = &self.handlers[definition.protocol as usize - 1];
        definition.change_id(dph, self.serial_port.as_mut(), id, new_id)
    }

    /// Set motor `id` to talk at `baudrate`, through `definition`. See
    /// [`ServoDefinition::change_baudrate`]. The bus's port stays at the old rate: call
    /// [`set_baudrate`](Self::set_baudrate) to keep talking to the motor afterwards.
    pub fn change_baudrate(
        &mut self,
        definition: ServoDefinition,
        id: u8,
        baudrate: u32,
    ) -> Result<()> {
        let dph = &self.handlers[definition.protocol as usize - 1];
        definition.change_baudrate(dph, self.serial_port.as_mut(), id, baudrate)
    }

    /// Switch the open serial port to `baudrate`.
    pub fn set_baudrate(&mut self, baudrate: u32) -> Result<()> {
        Ok(self.serial_port.set_baud_rate(baudrate)?)
    }

    /// Give the open serial port a new read timeout.
    pub fn set_timeout(&mut self, timeout: Duration) -> Result<()> {
        Ok(self.serial_port.set_timeout(timeout)?)
    }

    /// Run `op`, and run it again up to `retries` more times while it fails on the bus.
    /// A [`RegisterError`] is never tried again.
    pub fn with_retries<T>(
        &mut self,
        retries: u32,
        mut op: impl FnMut(&mut Self) -> Result<T>,
    ) -> Result<T> {
        retrying(retries, || op(self))
    }
}

#[cfg(feature = "python")]
mod python {
    use std::collections::BTreeMap;
    use std::sync::Mutex;

    use pyo3::exceptions::{PyIOError, PyRuntimeError, PyValueError};
    use pyo3::prelude::*;
    use pyo3_stub_gen::derive::{gen_stub_pyclass, gen_stub_pymethods};

    use super::Bus;
    use crate::servo::{RegisterError, ServoDefinition};

    /// Motors of several definitions, and of both protocols, on one serial port: the
    /// higher-level, family-agnostic API, designed mostly for the LeRobot library, where
    /// the controller classes are the per-servo one. Some of its choices are LeRobot
    /// conventions, such as the Feetech lock following the torque in `set_torque`.
    ///
    /// ```python
    /// with Bus("/dev/ttyUSB0", 1_000_000, 0.1, {
    ///     1: Xl430PyController.definition(),
    ///     2: Xl330PyController.definition(),
    /// }) as bus:
    ///     bus.sync_read_register([1, 2], "present_position")  # one Sync Read
    /// # the port is released here
    /// ```
    #[gen_stub_pyclass]
    #[pyclass(frozen, name = "Bus")]
    pub struct PyBus(Mutex<Option<Bus>>);

    impl PyBus {
        /// Run an access with the GIL released, telling a bad argument (`ValueError`)
        /// apart from a bus failure (`RuntimeError`).
        fn run<T: Send>(
            &self,
            py: Python,
            op: impl FnOnce(&mut Bus) -> crate::Result<T> + Send,
        ) -> PyResult<T> {
            py.detach(|| {
                let mut guard = self.0.lock().unwrap();
                let bus = guard.as_mut().ok_or((
                    false,
                    "bus is closed: its serial port has been released".to_string(),
                ))?;
                op(bus).map_err(|e| (e.is::<RegisterError>(), e.to_string()))
            })
            .map_err(|(bad_argument, message)| {
                if bad_argument {
                    PyValueError::new_err(message)
                } else {
                    PyRuntimeError::new_err(message)
                }
            })
        }
    }

    #[gen_stub_pymethods]
    #[pymethods]
    impl PyBus {
        /// Open `serial_port` for `motors`, as {id: definition}; the timeout is in seconds.
        #[new]
        pub fn new(
            serial_port: &str,
            baudrate: u32,
            timeout: f32,
            motors: BTreeMap<u8, ServoDefinition>,
        ) -> PyResult<Self> {
            let timeout = std::time::Duration::try_from_secs_f32(timeout)
                .map_err(|e| PyValueError::new_err(e.to_string()))?;
            let port = serialport::new(serial_port, baudrate)
                .timeout(timeout)
                .open()
                .map_err(|e| PyIOError::new_err(e.to_string()))?;
            Ok(Self(Mutex::new(Some(Bus::new(port, motors)))))
        }

        /// Release the serial port. Every later call raises `RuntimeError`.
        pub fn close(&self) {
            *self.0.lock().unwrap() = None;
        }

        /// Whether the bus still holds its serial port: `False` once `close()` has run.
        #[getter]
        pub fn is_open(&self) -> bool {
            self.0.lock().unwrap().is_some()
        }

        fn __enter__(slf: Py<Self>) -> Py<Self> {
            slf
        }

        /// Release the serial port on the way out of a `with` block, whatever happened in
        /// it; an exception raised in the block goes on.
        fn __exit__(
            &self,
            _exc_type: &Bound<'_, PyAny>,
            _exc_value: &Bound<'_, PyAny>,
            _traceback: &Bound<'_, PyAny>,
        ) {
            self.close();
        }

        /// Switch the open serial port to `baudrate`.
        pub fn set_baudrate(&self, py: Python, baudrate: u32) -> PyResult<()> {
            self.run(py, |bus| bus.set_baudrate(baudrate))
        }

        /// Give the open serial port a new read timeout, in seconds.
        pub fn set_timeout(&self, py: Python, timeout: f32) -> PyResult<()> {
            let timeout = std::time::Duration::try_from_secs_f32(timeout)
                .map_err(|e| PyValueError::new_err(e.to_string()))?;
            self.run(py, |bus| bus.set_timeout(timeout))
        }

        /// Read register `name` of motor `id` through its definition. A bus failure is
        /// tried again up to `retries` more times; a bad id or name never is.
        #[pyo3(signature = (id, name, retries = 0))]
        pub fn read_register(
            &self,
            py: Python,
            id: u8,
            name: String,
            retries: u32,
        ) -> PyResult<i64> {
            self.run(py, |bus| {
                bus.with_retries(retries, |bus| bus.read_register(id, &name))
            })
        }

        /// Same as `read_register`, plus the status packet's error field.
        #[pyo3(signature = (id, name, retries = 0))]
        pub fn read_register_with_error(
            &self,
            py: Python,
            id: u8,
            name: String,
            retries: u32,
        ) -> PyResult<(i64, u8)> {
            self.run(py, |bus| {
                bus.with_retries(retries, |bus| bus.read_register_with_error(id, &name))
            })
            .map(|(value, error)| (value, error.byte()))
        }

        /// Write `value` to register `name` of motor `id` through its definition.
        #[pyo3(signature = (id, name, value, retries = 0))]
        pub fn write_register(
            &self,
            py: Python,
            id: u8,
            name: String,
            value: i64,
            retries: u32,
        ) -> PyResult<()> {
            self.run(py, |bus| {
                bus.with_retries(retries, |bus| bus.write_register(id, &name, value))
            })
        }

        /// Same as `write_register`, and return the status packet's error field.
        #[pyo3(signature = (id, name, value, retries = 0))]
        pub fn write_register_with_error(
            &self,
            py: Python,
            id: u8,
            name: String,
            value: i64,
            retries: u32,
        ) -> PyResult<u8> {
            self.run(py, |bus| {
                bus.with_retries(retries, |bus| {
                    bus.write_register_with_error(id, &name, value)
                })
            })
            .map(|error| error.byte())
        }

        /// Sync read register `name` from `ids`, in the order asked: one Sync Read per
        /// group of motors sharing a protocol and the register's address and size.
        #[pyo3(signature = (ids, name, retries = 0))]
        pub fn sync_read_register(
            &self,
            py: Python,
            ids: Vec<u8>,
            name: String,
            retries: u32,
        ) -> PyResult<Vec<i64>> {
            self.run(py, |bus| {
                bus.with_retries(retries, |bus| bus.sync_read_register(&ids, &name))
            })
        }

        /// Same as `sync_read_register`, plus each motor's error field.
        #[pyo3(signature = (ids, name, retries = 0))]
        pub fn sync_read_register_with_error(
            &self,
            py: Python,
            ids: Vec<u8>,
            name: String,
            retries: u32,
        ) -> PyResult<Vec<(i64, u8)>> {
            self.run(py, |bus| {
                bus.with_retries(retries, |bus| {
                    bus.sync_read_register_with_error(&ids, &name)
                })
            })
            .map(|values| values.into_iter().map(|(v, e)| (v, e.byte())).collect())
        }

        /// Read contiguous named registers from each motor, in the order asked.
        /// Registers must be listed in address order with no gaps.
        #[pyo3(signature = (ids, names, retries = 0))]
        pub fn sync_read_block(
            &self,
            py: Python,
            ids: Vec<u8>,
            names: Vec<String>,
            retries: u32,
        ) -> PyResult<Vec<Vec<i64>>> {
            let names: Vec<&str> = names.iter().map(String::as_str).collect();
            self.run(py, |bus| {
                bus.with_retries(retries, |bus| bus.sync_read_block(&ids, &names))
            })
        }

        /// Read contiguous named registers and each motor's status error byte.
        /// Registers must be listed in address order with no gaps.
        #[pyo3(signature = (ids, names, retries = 0))]
        pub fn sync_read_block_with_error(
            &self,
            py: Python,
            ids: Vec<u8>,
            names: Vec<String>,
            retries: u32,
        ) -> PyResult<Vec<(Vec<i64>, u8)>> {
            let names: Vec<&str> = names.iter().map(String::as_str).collect();
            self.run(py, |bus| {
                bus.with_retries(retries, |bus| bus.sync_read_block_with_error(&ids, &names))
            })
            .map(|replies| {
                replies
                    .into_iter()
                    .map(|(values, status)| (values, status.byte()))
                    .collect()
            })
        }

        /// Read `length` raw bytes at `addr` and each motor's status error byte, in
        /// the order of `ids`. One Sync Read per protocol; groups containing a motor
        /// without Sync Read support use individual reads.
        ///
        /// The caller must ensure the range means the same thing on every motor.
        /// No decoding or register layout checks are performed. Unknown ids fail
        /// before any instruction is sent.
        #[pyo3(signature = (ids, addr, length, retries = 0))]
        pub fn sync_read_raw_with_error(
            &self,
            py: Python,
            ids: Vec<u8>,
            addr: u8,
            length: u8,
            retries: u32,
        ) -> PyResult<Vec<(Vec<u8>, u8)>> {
            self.run(py, |bus| {
                bus.with_retries(retries, |bus| {
                    bus.sync_read_raw_with_error(&ids, addr, length)
                })
            })
            .map(|replies| {
                replies
                    .into_iter()
                    .map(|(bytes, status)| (bytes, status.byte()))
                    .collect()
            })
        }

        /// Sync write `values` to register `name` of `ids`, one value per id: one Sync
        /// Write per group as in `sync_read_register`.
        #[pyo3(signature = (ids, name, values, retries = 0))]
        pub fn sync_write_register(
            &self,
            py: Python,
            ids: Vec<u8>,
            name: String,
            values: Vec<i64>,
            retries: u32,
        ) -> PyResult<()> {
            self.run(py, |bus| {
                bus.with_retries(retries, |bus| bus.sync_write_register(&ids, &name, &values))
            })
        }

        /// Turn the torque of `ids` on or off, every motor tried even after one fails, and
        /// return {id: error} for those that failed (empty when all went through), a motor
        /// refusing the write included; an alarm in an answer does not fail it. A bus
        /// failure is tried again up to `retries` more times per motor. On Feetech servos the
        /// lock follows the torque (`ServoDefinition.eeprom_lock`).
        #[pyo3(signature = (ids, enabled, retries = 0))]
        pub fn set_torque(
            &self,
            py: Python,
            ids: Vec<u8>,
            enabled: bool,
            retries: u32,
        ) -> PyResult<BTreeMap<u8, String>> {
            self.run(py, |bus| {
                Ok(bus
                    .set_torque(&ids, enabled, retries)
                    .into_iter()
                    .map(|(id, e)| (id, e.to_string()))
                    .collect())
            })
        }

        /// Zero the turn count of `ids` (Feetech's RESET, 0x0A, or Dynamixel's Clear,
        /// 0x10), every motor tried even after one fails, and return {id: error} for those
        /// that failed (empty when all went through): a servo not known to answer it
        /// (`ServoDefinition.supports_turn_count_reset`), a motor refusing it, or one not
        /// answering. A bus failure is tried again up to `retries` more times per motor.
        /// An STS servo keeps counting turns past its encoder's zero, torque on or off, so
        /// a joint moved by hand through that boundary reports its position a turn away
        /// from its limits and, once the torque is on, drives a whole turn to a goal. The
        /// other registers are kept; a power cycle clears the count as well. Send this
        /// with the torque off.
        #[pyo3(signature = (ids, retries = 0))]
        pub fn reset_turn_count(
            &self,
            py: Python,
            ids: Vec<u8>,
            retries: u32,
        ) -> PyResult<BTreeMap<u8, String>> {
            self.run(py, |bus| {
                Ok(bus
                    .reset_turn_count(&ids, retries)
                    .into_iter()
                    .map(|(id, e)| (id, e.to_string()))
                    .collect())
            })
        }

        /// Give motor `id` the id `new_id`, through `definition`: torque off and, on a
        /// Feetech motor, lock open first. The motor need not be one of the bus's. A motor
        /// that refuses a write raises `RuntimeError`; an alarm in its answer does not.
        pub fn change_id(
            &self,
            py: Python,
            definition: ServoDefinition,
            id: u8,
            new_id: u8,
        ) -> PyResult<()> {
            self.run(py, |bus| bus.change_id(definition, id, new_id))
        }

        /// Set motor `id` to talk at `baudrate`, through `definition`, as `change_id`
        /// does it. A rate the servo cannot take raises `ValueError`. The bus's port stays
        /// at the old rate: call `set_baudrate(baudrate)` to keep talking to the motor
        /// afterwards.
        pub fn change_baudrate(
            &self,
            py: Python,
            definition: ServoDefinition,
            id: u8,
            baudrate: u32,
        ) -> PyResult<()> {
            self.run(py, |bus| bus.change_baudrate(definition, id, baudrate))
        }

        /// Which of `ids` answer, as {id: model number} read as `definition` lays it out,
        /// over its protocol; every id that protocol allows when `ids` is left out.
        #[pyo3(signature = (definition, ids = None))]
        pub fn scan(
            &self,
            py: Python,
            definition: ServoDefinition,
            ids: Option<Vec<u8>>,
        ) -> PyResult<BTreeMap<u8, u16>> {
            self.run(py, |bus| match &ids {
                Some(ids) => bus.scan(definition, ids),
                None => bus.scan_all(definition),
            })
        }

        /// Which of `ids` answer, as {id: model number} read as `definition` lays it out;
        /// every id its protocol allows when `ids` is left out. One broadcast ping, which
        /// listens for its whole window (about 0.8 s at 1 Mbps) however few ids are asked,
        /// then a read of each id asked that answered: faster than `scan` over many ids,
        /// and safe behind a USB adapter with a long latency timer. A servo that does not
        /// answer a broadcast ping (`definition.supports_broadcast_ping`) raises
        /// `ValueError`.
        #[pyo3(signature = (definition, ids = None))]
        pub fn broadcast_scan(
            &self,
            py: Python,
            definition: ServoDefinition,
            ids: Option<Vec<u8>>,
        ) -> PyResult<BTreeMap<u8, u16>> {
            self.run(py, |bus| match &ids {
                Some(ids) => bus.broadcast_scan(definition, ids),
                None => bus.broadcast_scan_all(definition),
            })
        }
    }
}

#[cfg(feature = "python")]
pub use python::PyBus;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fake_port::FakePort;
    use crate::servo::dynamixel::{xl330, xl430};
    use crate::servo::feetech::{scs0009, sts3215};
    use crate::StatusPacketError;

    #[test]
    fn a_bus_reads_each_motor_through_its_definition() {
        // Present position 2048 from the STS3215 (little-endian), 16 from the SCS0009
        // (big-endian), then the same two again for the sync read.
        let sts = vec![0xFF, 0xFF, 0x01, 0x04, 0x00, 0x00, 0x08, 0xF2];
        let scs = vec![0xFF, 0xFF, 0x02, 0x04, 0x00, 0x00, 0x10, 0xE9];
        let port = FakePort::new(vec![sts.clone(), scs.clone(), sts, scs]);
        let written = port.written();
        let mut bus = Bus::new(
            Box::new(port),
            BTreeMap::from([(1, sts3215::DEFINITION), (2, scs0009::DEFINITION)]),
        );

        assert_eq!(bus.read_register(1, "present_position").unwrap(), 2048);
        assert_eq!(bus.read_register(2, "present_position").unwrap(), 16);

        // The SCS0009 has no Sync Read, so its group is read one id at a time.
        assert_eq!(
            bus.sync_read_register(&[1, 2], "present_position").unwrap(),
            [2048, 16]
        );
        assert_eq!(written.lock().unwrap().len(), 4);

        // An id the bus does not have is refused before anything is sent.
        let err = bus.read_register(9, "present_position").unwrap_err();
        assert!(matches!(
            err.downcast_ref::<RegisterError>(),
            Some(RegisterError::UnknownMotor(9))
        ));
        assert_eq!(written.lock().unwrap().len(), 4);
    }

    #[test]
    fn named_block_reads_decode_values_and_preserve_status() {
        let reply = vec![0xFF, 0xFF, 1, 6, 0x20, 0x00, 0x08, 0x64, 0x80, 0xEC];
        let port = FakePort::new(vec![reply]);
        let written = port.written();
        let mut bus = Bus::new(Box::new(port), BTreeMap::from([(1, sts3215::DEFINITION)]));
        let replies = bus
            .sync_read_block_with_error(&[1], &["present_position", "present_speed"])
            .unwrap();
        assert_eq!(replies[0].0, vec![2048, -100]);
        assert_eq!(replies[0].1.byte(), 0x20);
        assert!(bus
            .sync_read_block_with_error(&[1], &["present_position", "present_load"])
            .is_err());
        for ids in [&[1][..], &[]] {
            let error = bus.sync_read_block(ids, &[]).unwrap_err();
            assert_eq!(
                error.downcast_ref::<RegisterError>(),
                Some(&RegisterError::EmptyRegisterBlock)
            );
        }
        assert_eq!(written.lock().unwrap().len(), 1);
    }

    #[test]
    fn named_block_reads_return_mixed_protocols_in_requested_order() {
        let v1_replies = vec![
            0xFF, 0xFF, 1, 4, 0x20, 0x00, 0x08, 0xD2, 0xFF, 0xFF, 3, 4, 0x00, 0x34, 0x12, 0xB2,
        ];
        let v2_reply = vec![
            0xFF, 0xFF, 0xFD, 0x00, 0x02, 0x08, 0x00, 0x55, 0x80, 0xFE, 0xFF, 0xFF, 0xFF, 0x36,
            0x1A,
        ];
        let port = FakePort::new(vec![v1_replies, v2_reply]);
        let mut bus = Bus::new(
            Box::new(port),
            BTreeMap::from([
                (1, sts3215::DEFINITION),
                (2, xl330::DEFINITION),
                (3, sts3215::DEFINITION),
            ]),
        );

        let replies = bus
            .sync_read_block_with_error(&[1, 2, 3], &["present_position"])
            .unwrap();
        let replies: Vec<_> = replies
            .into_iter()
            .map(|(values, status)| (values, status.byte()))
            .collect();
        assert_eq!(
            replies,
            [(vec![2048], 0x20), (vec![-2], 0x80), (vec![4660], 0)]
        );
    }

    #[test]
    fn named_block_reads_reject_a_short_reply() {
        let port = FakePort::new(vec![vec![0xFF, 0xFF, 1, 2, 0x20, 0xDC]]);
        let mut bus = Bus::new(Box::new(port), BTreeMap::from([(1, sts3215::DEFINITION)]));
        let error = bus
            .sync_read_block(&[1], &["present_position", "present_speed"])
            .unwrap_err();
        let error = error.downcast_ref::<crate::SyncReadError>().unwrap();
        assert_eq!(error.id, 1);
        assert_eq!(
            error.cause.downcast_ref::<std::io::Error>().unwrap().kind(),
            std::io::ErrorKind::InvalidData
        );
    }

    #[test]
    fn raw_reads_group_protocols_and_restore_requested_order_with_status() {
        let v1_replies = vec![
            0xFF, 0xFF, 1, 4, 0x20, 0x00, 0x08, 0xD2, 0xFF, 0xFF, 3, 4, 0x00, 0x34, 0x12, 0xB2,
        ];
        let v2_replies = vec![
            0xFF, 0xFF, 0xFD, 0, 2, 6, 0, 0x55, 0x80, 0x12, 0x34, 0x4E, 0xBE, 0xFF, 0xFF, 0xFD, 0,
            4, 6, 0, 0x55, 7, 0x56, 0x78, 0xEC, 0x2B,
        ];
        let port = FakePort::new(vec![v1_replies, v2_replies]);
        let written = port.written();
        let mut bus = Bus::new(
            Box::new(port),
            BTreeMap::from([
                (1, sts3215::DEFINITION),
                (2, xl330::DEFINITION),
                (3, sts3215::DEFINITION),
                (4, xl430::DEFINITION),
            ]),
        );

        // The raw range need not match a named register in any definition.
        let replies = bus.sync_read_raw_with_error(&[1, 2, 3, 4], 56, 2).unwrap();
        let replies: Vec<_> = replies
            .into_iter()
            .map(|(bytes, status)| (bytes, status.byte()))
            .collect();
        assert_eq!(
            replies,
            vec![
                (vec![0x00, 0x08], 0x20),
                (vec![0x12, 0x34], 0x80),
                (vec![0x34, 0x12], 0),
                (vec![0x56, 0x78], 7),
            ]
        );
        let written = written.lock().unwrap();
        assert_eq!(written.len(), 2);
        assert_eq!(&written[0][..9], &[0xFF, 0xFF, 0xFE, 6, 0x82, 56, 2, 1, 3]);
        assert_eq!(
            &written[1][..14],
            &[0xFF, 0xFF, 0xFD, 0, 0xFE, 9, 0, 0x82, 56, 0, 2, 0, 2, 4]
        );
    }

    #[test]
    fn raw_reads_fall_back_for_a_protocol_group_without_sync_read() {
        let port = FakePort::new(vec![
            vec![0xFF, 0xFF, 1, 4, 0x20, 0, 8, 0xD2],
            vec![0xFF, 0xFF, 2, 4, 0, 0x12, 0x34, 0xB3],
        ]);
        let written = port.written();
        let mut bus = Bus::new(
            Box::new(port),
            BTreeMap::from([(1, sts3215::DEFINITION), (2, scs0009::DEFINITION)]),
        );

        let replies = bus.sync_read_raw_with_error(&[1, 2], 56, 2).unwrap();
        assert_eq!(replies[0].0, [0, 8]);
        assert_eq!(replies[0].1.byte(), 0x20);
        assert_eq!(replies[1].0, [0x12, 0x34]);
        assert_eq!(replies[1].1.byte(), 0);
        let written = written.lock().unwrap();
        assert_eq!(written.len(), 2);
        assert_eq!(&written[0][..7], &[0xFF, 0xFF, 1, 4, 2, 56, 2]);
        assert_eq!(&written[1][..7], &[0xFF, 0xFF, 2, 4, 2, 56, 2]);
    }

    #[test]
    fn raw_reads_validate_ids_before_sending_and_accept_no_ids() {
        let port = FakePort::new(vec![]);
        let written = port.written();
        let mut bus = Bus::new(Box::new(port), BTreeMap::from([(1, sts3215::DEFINITION)]));

        assert!(bus.sync_read_raw_with_error(&[], 56, 2).unwrap().is_empty());
        let failure = bus.sync_read_raw_with_error(&[1, 9], 56, 2).unwrap_err();
        assert!(matches!(
            failure.downcast_ref::<RegisterError>(),
            Some(RegisterError::UnknownMotor(9))
        ));
        assert!(written.lock().unwrap().is_empty());
    }

    #[test]
    fn torque_goes_off_on_every_motor_even_when_one_fails() {
        // Motor 1 (STS3215) answers both its writes, motor 2 does not answer, motor 3
        // (XL330) answers; 9 is not on the bus.
        let sts = vec![0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC];
        let xl = vec![
            0xFF, 0xFF, 0xFD, 0x00, 0x03, 0x04, 0x00, 0x55, 0x00, 0x52, 0x8C,
        ];
        let port = FakePort::new(vec![sts.clone(), sts, vec![], xl]);
        let written = port.written();
        let mut bus = Bus::new(
            Box::new(port),
            BTreeMap::from([
                (1, sts3215::DEFINITION),
                (2, sts3215::DEFINITION),
                (3, xl330::DEFINITION),
            ]),
        );

        let failed = bus.set_torque(&[1, 2, 3, 9], false, 0);

        assert_eq!(failed.keys().copied().collect::<Vec<_>>(), [2, 9]);
        // The STS3215 gets its torque (address 40) then its lock (55) opened; motor 2 is
        // tried; the XL330, which has no such lock, gets its torque (64) only.
        let written = written.lock().unwrap();
        assert_eq!(written.len(), 4);
        assert_eq!([written[0][5], written[0][6]], [40, 0]);
        assert_eq!([written[1][5], written[1][6]], [55, 0]);
        assert_eq!(written[2][2], 2);
        assert_eq!([written[3][8], written[3][10]], [64, 0]);
    }

    #[test]
    fn an_alarm_does_not_fail_the_torque_and_the_lock_follows_it() {
        // Motor 1 answers both its writes with 0x20, an overload alarm: they went through.
        let alarm = vec![0xFF, 0xFF, 0x01, 0x02, 0x20, 0xDC];
        let ok = vec![0xFF, 0xFF, 0x02, 0x02, 0x00, 0xFB];
        let port = FakePort::new(vec![alarm.clone(), alarm, ok.clone(), ok]);
        let written = port.written();
        let mut bus = Bus::new(
            Box::new(port),
            BTreeMap::from([(1, sts3215::DEFINITION), (2, sts3215::DEFINITION)]),
        );

        assert!(bus.set_torque(&[1, 2], false, 0).is_empty());

        // The torque (address 40), then the lock (address 55), of each motor.
        let written = written.lock().unwrap();
        let writes: Vec<_> = written
            .iter()
            .map(|packet| (packet[2], packet[5]))
            .collect();
        assert_eq!(writes, [(1, 40), (1, 55), (2, 40), (2, 55)]);
    }

    #[test]
    fn a_motor_refusing_the_torque_write_fails_and_is_not_retried() {
        // XL330 1 refuses its torque write with an access error; XL330 2 takes it.
        let refused = vec![
            0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x04, 0x00, 0x55, 0x07, 0xB0, 0x8C,
        ];
        let ok = vec![
            0xFF, 0xFF, 0xFD, 0x00, 0x02, 0x04, 0x00, 0x55, 0x00, 0x29, 0x0C,
        ];
        let port = FakePort::new(vec![refused, ok]);
        let written = port.written();
        let mut bus = Bus::new(
            Box::new(port),
            BTreeMap::from([(1, xl330::DEFINITION), (2, xl330::DEFINITION)]),
        );

        let failed = bus.set_torque(&[1, 2], false, 2);

        assert_eq!(failed.keys().copied().collect::<Vec<_>>(), [1]);
        assert!(matches!(
            failed[&1].downcast_ref::<StatusPacketError>(),
            Some(StatusPacketError { id: 1, .. })
        ));
        // One torque write to motor 1, not retried, then motor 2's.
        let written = written.lock().unwrap();
        assert_eq!(
            written.iter().map(|packet| packet[4]).collect::<Vec<_>>(),
            [1, 2]
        );
    }

    #[test]
    fn torque_on_closes_a_feetech_lock() {
        let sts = vec![0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC];
        let port = FakePort::new(vec![sts.clone(), sts]);
        let written = port.written();
        let mut bus = Bus::new(Box::new(port), BTreeMap::from([(1, sts3215::DEFINITION)]));

        assert!(bus.set_torque(&[1], true, 0).is_empty());

        let written = written.lock().unwrap();
        assert_eq!([written[0][5], written[0][6]], [40, 1]);
        assert_eq!([written[1][5], written[1][6]], [55, 1]);
    }

    #[test]
    fn a_turn_count_reset_reaches_the_feetech_motors_only() {
        // STS3215 1 answers its reset; STS3215 2 does not answer; the XL430 3 is not
        // known to answer Clear, so nothing is sent to it; 9 is not on the bus.
        let sts = vec![0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC];
        let port = FakePort::new(vec![sts, vec![]]);
        let written = port.written();
        let mut bus = Bus::new(
            Box::new(port),
            BTreeMap::from([
                (1, sts3215::DEFINITION),
                (2, sts3215::DEFINITION),
                (3, xl430::DEFINITION),
            ]),
        );

        let failed = bus.reset_turn_count(&[1, 2, 3, 9], 0);

        assert_eq!(failed.keys().copied().collect::<Vec<_>>(), [2, 3, 9]);
        assert!(matches!(
            failed[&3].downcast_ref::<RegisterError>(),
            Some(RegisterError::TurnCountReset("XL430"))
        ));
        // The RESET instruction (0x0A, no parameter) went to motors 1 and 2 only.
        let written = written.lock().unwrap();
        assert_eq!(
            *written,
            [
                vec![0xFF, 0xFF, 0x01, 0x02, 0x0A, 0xF2],
                vec![0xFF, 0xFF, 0x02, 0x02, 0x0A, 0xF1],
            ]
        );
    }

    #[test]
    fn a_motor_the_bus_does_not_have_can_be_given_its_id() {
        // A new STS3215 answers at id 1, which the bus gives to an XL330.
        let status = vec![0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC];
        let port = FakePort::new(vec![status.clone(), status.clone(), status]);
        let written = port.written();
        let mut bus = Bus::new(Box::new(port), BTreeMap::from([(1, xl330::DEFINITION)]));

        bus.change_id(sts3215::DEFINITION, 1, 11).unwrap();

        // Protocol v1 all the way: three writes, the last one the id.
        let written = written.lock().unwrap();
        assert_eq!(written.len(), 3);
        assert_eq!(written[2][..7], [0xFF, 0xFF, 0x01, 0x04, 0x03, 0x05, 11]);
    }

    #[test]
    fn a_value_that_does_not_fit_leaves_the_whole_bus_unwritten() {
        let port = FakePort::new(vec![]);
        let written = port.written();
        let mut bus = Bus::new(
            Box::new(port),
            BTreeMap::from([
                (1, xl330::DEFINITION),
                (2, xl330::DEFINITION),
                (11, sts3215::DEFINITION),
            ]),
        );

        // The v2 group (1, 2) comes first and its values fit; 70000 does not fit the
        // STS3215's sign-magnitude goal position, in the second group.
        let err = bus
            .sync_write_register(&[1, 2, 11], "goal_position", &[100, 200, 70000])
            .unwrap_err();

        assert!(matches!(
            err.downcast_ref::<RegisterError>(),
            Some(RegisterError::OutOfRange { .. })
        ));
        assert!(written.lock().unwrap().is_empty());
    }

    #[test]
    fn a_bus_sends_one_sync_write_per_protocol_and_layout() {
        let port = FakePort::new(vec![]);
        let written = port.written();
        let mut bus = Bus::new(
            Box::new(port),
            BTreeMap::from([
                (1, sts3215::DEFINITION),
                (2, xl330::DEFINITION),
                (3, xl430::DEFINITION),
            ]),
        );

        // The XL330 and XL430 share protocol v2 and the goal position's address and size:
        // one Sync Write reaches both. The STS3215 speaks v1: a second one.
        bus.sync_write_register(&[2, 1, 3], "goal_position", &[100, 200, 300])
            .unwrap();

        let written = written.lock().unwrap();
        assert_eq!(written.len(), 2);
        // Protocol v2 header, Sync Write (0x83), ids 2 and 3 in the order asked.
        assert_eq!(written[0][..4], [0xFF, 0xFF, 0xFD, 0x00]);
        assert_eq!(written[0][7], 0x83);
        assert_eq!([written[0][12], written[0][17]], [2, 3]);
        // Protocol v1 header, Sync Write (0x83), id 1 with 200 little-endian.
        assert_eq!(written[1][..3], [0xFF, 0xFF, 0xFE]);
        assert_eq!(written[1][4], 0x83);
        assert_eq!(written[1][7..10], [0x01, 0xC8, 0x00]);
    }
}
