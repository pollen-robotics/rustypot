//! Motors of several definitions, and of both protocols, on one serial port.
//!
//! A controller speaks one definition over one protocol. A robot's bus often does not:
//! LeRobot's Koch arm has XL430 and XL330 motors on one port, and Reachy Mini has
//! STS3215 (protocol v1) and XL330 (protocol v2) motors on one port. A [`Bus`] holds the
//! port, a handler for each protocol and the definition of each motor, and calls the
//! definition's functions with the handler that matches it.
//!
//! Together with [`ServoDefinition`], this is rustypot's higher-level API: it works
//! across servo families and reaches registers by name, and it is designed mostly for
//! the [LeRobot](https://github.com/huggingface/lerobot) library. Some of its choices are
//! LeRobot's conventions rather than something the servos require, such as
//! [`Bus::change_id`] and [`Bus::change_baudrate`] leaving the torque off and the Feetech
//! lock open. The controller of each servo module stays the general-purpose API, with
//! one typed accessor per register.

use std::collections::BTreeMap;
use std::time::Duration;

use serialport::SerialPort;

use crate::servo::definition::{self, retrying, ServoDefinition, SyncWrite};
use crate::servo::RegisterError;
use crate::{DynamixelProtocolHandler, Result, StatusError};

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

    /// Give motor `id` the id `new_id`, through `definition`. See
    /// [`ServoDefinition::change_id`]. Like [`scan`](Self::scan), it reaches motors the
    /// bus does not have: a new motor answers at its factory id.
    pub fn change_id(&mut self, definition: ServoDefinition, id: u8, new_id: u8) -> Result<()> {
        let dph = &self.handlers[definition.protocol as usize - 1];
        definition.change_id(dph, self.serial_port.as_mut(), id, new_id)
    }

    /// Set motor `id` to talk at `baudrate`, through `definition`. See
    /// [`ServoDefinition::change_baudrate`].
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

    /// Motors of several definitions, and of both protocols, on one serial port.
    ///
    /// A higher-level API that works across servo families, designed mostly for LeRobot,
    /// and carrying some of its conventions (`change_id` and `change_baudrate` leave the
    /// torque off and the Feetech lock open). The controller classes stay the
    /// general-purpose API.
    ///
    /// ```python
    /// bus = Bus("/dev/ttyUSB0", 1_000_000, 0.1, {
    ///     1: Xl430PyController.definition(),
    ///     2: Xl330PyController.definition(),
    /// })
    /// bus.sync_read_register([1, 2], "present_position")  # one Sync Read
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

        /// Give motor `id` the id `new_id`, through `definition`: torque off and, on a
        /// Feetech motor, lock open first. The motor need not be one of the bus's.
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
        /// does it. A rate the servo cannot take raises `ValueError`.
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
