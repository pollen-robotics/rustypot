#[macro_export]
macro_rules! generate_servo {
    ($servo_name:ident, $protocol:ident,
     $(resolution: $resolution:expr,)?
     $(word_order: $word_order:ident,)?
     $(supports_sync_read: $supports_sync_read:expr,)?
     $(baudrates: [$(($baud:expr, $baud_code:expr)),* $(,)?],)?
     $(encoding: [$(($enc_reg:ident, $enc_kind:ident $(($enc_arg:expr))?)),* $(,)?],)?
     $(reg: ($reg_name:ident, $reg_access:ident, $reg_addr:expr, $reg_type:ty, $conv:ident),)+
    ) => {
        paste::paste! {
            pub struct [<$servo_name:camel Controller>] {
                dph: Option<$crate::DynamixelProtocolHandler>,
                serial_port: Option<Box<dyn serialport::SerialPort>>,
                /// Motors addressed by name through another definition than this one.
                definitions: std::collections::BTreeMap<u8, $crate::servo::ServoDefinition>,
            }

            impl Default for [<$servo_name:camel Controller>] {
                fn default() -> Self {
                    Self::new()
                }
            }

            impl [<$servo_name:camel Controller>] {
                pub fn new() -> Self {
                    Self {
                        dph: None,
                        serial_port: None,
                        definitions: std::collections::BTreeMap::new(),
                    }
                }
                pub fn with_serial_port(self,
                                        serial_port: Box<dyn serialport::SerialPort>,
                ) -> Self {
                    Self {
                        serial_port: Some(serial_port),
                        ..self
                    }
                }

                /// Switch the open serial port to `baudrate`.
                ///
                /// Motors set to another rate stop answering until they are switched
                /// too; this is how a bus is probed at each rate a motor might be at.
                pub fn set_baudrate(&mut self, baudrate: u32) -> $crate::Result<()> {
                    Ok(self.serial_port.as_mut().unwrap().set_baud_rate(baudrate)?)
                }

                /// Give the open serial port a new read timeout.
                ///
                /// This bounds every transaction with a motor that does not answer.
                pub fn set_timeout(&mut self, timeout: std::time::Duration) -> $crate::Result<()> {
                    Ok(self.serial_port.as_mut().unwrap().set_timeout(timeout)?)
                }
            }

            #[cfg(feature = "python")]
            #[gen_stub_pyclass]
            #[pyo3::pyclass(frozen)]
            pub struct [<$servo_name:camel PyController>](
                std::sync::Mutex<Option<[<$servo_name:camel Controller>]>>,
            );

            #[cfg(feature = "python")]
            impl [<$servo_name:camel PyController>] {
                /// Borrow the controller, or report that it has been closed.
                ///
                /// The error is a String, not a PyErr: it is produced inside the closure
                /// that runs with the GIL released, and is converted at the call site.
                fn borrow(
                    guard: &mut Option<[<$servo_name:camel Controller>]>,
                ) -> Result<&mut [<$servo_name:camel Controller>], String> {
                    guard.as_mut().ok_or_else(|| {
                        "controller is closed: its serial port has been released".to_string()
                    })
                }
            }

            #[cfg(feature = "python")]
            #[gen_stub_pymethods]
            #[pymethods]
            impl [<$servo_name:camel PyController>] {
                /// Release the serial port.
                ///
                /// Dropping the controller does this too, but a caller that needs the port
                /// freed at a known point -- to reopen it at another baudrate, or to hand it
                /// to another process -- cannot rely on when that happens. In Python an
                /// exception traceback can keep the object alive long past the last reference
                /// a caller knows about, and the next open then fails with 'port is in use'.
                ///
                /// Every later operation raises `RuntimeError`. Calling it twice is harmless.
                pub fn close(&self) {
                    *self.0.lock().unwrap() = None;
                }

                /// Whether the controller still holds its serial port.
                pub fn is_open(&self) -> bool {
                    self.0.lock().unwrap().is_some()
                }

                /// Switch the open serial port to `baudrate`.
                ///
                /// Motors set to another rate stop answering until they are switched
                /// too; this is how a bus is probed at each rate a motor might be at.
                pub fn set_baudrate(&self, baudrate: u32) -> PyResult<()> {
                    let mut guard = self.0.lock().unwrap();
                    Self::borrow(&mut guard)
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)?
                        .set_baudrate(baudrate)
                        .map_err(|e| pyo3::exceptions::PyIOError::new_err(e.to_string()))
                }

                /// Give the open serial port a new read timeout, in seconds like the
                /// constructor's.
                ///
                /// This bounds every transaction with a motor that does not answer.
                pub fn set_timeout(&self, timeout: f32) -> PyResult<()> {
                    let mut guard = self.0.lock().unwrap();
                    Self::borrow(&mut guard)
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)?
                        .set_timeout(std::time::Duration::from_secs_f32(timeout))
                        .map_err(|e| pyo3::exceptions::PyIOError::new_err(e.to_string()))
                }

                /// Every register of this servo, in declaration order.
                ///
                /// The control table, for code that picks registers at runtime: pair an
                /// entry's `addr` and `size` with `read_raw_data` or `write_raw_data`.
                /// A static method, so it needs no serial port.
                #[staticmethod]
                pub fn registers() -> Vec<$crate::servo::RegisterInfo> {
                    REGISTERS.to_vec()
                }

                /// Look up a register by name, as spelled in `registers()`.
                ///
                /// `None` when this servo has no such register.
                #[staticmethod]
                pub fn register(name: &str) -> Option<$crate::servo::RegisterInfo> {
                    register(name)
                }

                /// Encoder steps per turn, or `None` when the servo does not count steps.
                #[staticmethod]
                pub fn resolution() -> Option<u32> {
                    INFO.resolution
                }

                /// Byte order of multi-byte registers on the wire: "little" or "big".
                #[staticmethod]
                pub fn word_order() -> &'static str {
                    INFO.word_order.as_str()
                }

                /// Whether the firmware answers the Sync Read instruction.
                #[staticmethod]
                pub fn supports_sync_read() -> bool {
                    INFO.supports_sync_read
                }

                /// Serial rates the servo can be set to, as {baud rate: register value}.
                #[staticmethod]
                pub fn baudrates() -> std::collections::HashMap<u32, u8> {
                    INFO.baudrates.iter().copied().collect()
                }

                /// This servo's definition, to hand to `set_definition` on the controller
                /// of a bus that mixes it with servos of another definition.
                #[staticmethod]
                pub fn definition() -> $crate::servo::ServoDefinition {
                    DEFINITION
                }
            }
        }

        #[cfg(feature = "python")]
        use pyo3::prelude::*;
        #[cfg(feature = "python")]
        use pyo3_stub_gen::derive::*;

        /// What this servo states about itself beyond its registers.
        pub const INFO: $crate::servo::ServoInfo = $crate::servo::ServoInfo {
            resolution: $crate::servo_resolution!($($resolution)?),
            word_order: $crate::servo_word_order!($($word_order)?),
            supports_sync_read: $crate::servo_supports_sync_read!($($supports_sync_read)?),
            baudrates: &[$($(($baud, $baud_code)),*)?],
        };

        const ENCODING_OVERRIDES: &[(&str, $crate::servo::Encoding)] = &[
            $($((stringify!($enc_reg), $crate::register_encoding!($enc_kind $(($enc_arg))?))),*)?
        ];

        /// Every register of this servo, in declaration order.
        ///
        /// Useful when the registers are chosen at runtime rather than in source, e.g.
        /// building an indirect address map or a tool that takes register names.
        pub const REGISTERS: &[$crate::servo::RegisterInfo] = &[
            $($crate::servo::RegisterInfo {
                name: stringify!($reg_name),
                addr: $reg_addr,
                // Evaluated at compile time: a register type too large for the protocol's u8
                // lengths fails the build here instead of being silently truncated.
                size: {
                    let size = std::mem::size_of::<$reg_type>();
                    assert!(size <= u8::MAX as usize, "register type is larger than 255 bytes");
                    size as u8
                },
                access: $crate::register_access!($reg_access),
                encoding: $crate::servo::encoding_for(
                    stringify!($reg_name),
                    ENCODING_OVERRIDES,
                    <$reg_type as $crate::servo::RegisterType>::DEFAULT_ENCODING,
                ),
            },)*
        ];

        /// Look up a register by name, as spelled in [`REGISTERS`].
        pub fn register(name: &str) -> Option<$crate::servo::RegisterInfo> {
            REGISTERS.iter().copied().find(|r| r.name == name)
        }

        /// This servo's definition as a value, see [`ServoDefinition`](crate::servo::ServoDefinition).
        pub const DEFINITION: $crate::servo::ServoDefinition = $crate::servo::ServoDefinition {
            name: stringify!($servo_name),
            protocol: $crate::protocol_version!($protocol),
            info: INFO,
            registers: REGISTERS,
        };

        $crate::generate_protocol_constructor!($servo_name, $protocol);
        $crate::generate_special_instructions!($servo_name);
        $crate::generate_addr_read_write!($servo_name);
        $crate::generate_register_access!($servo_name);
        $crate::generate_scan!($servo_name);

        $(
            $crate::generate_reg_access!($servo_name, $reg_name, $reg_access, $reg_addr, $reg_type, $conv);
        )*
    };
}

#[macro_export]
macro_rules! generate_protocol_constructor {
    ($servo_name:ident, v1) => {
        paste::paste! {
            impl [<$servo_name:camel Controller>] {
                pub fn with_protocol_v1(
                    self,
                ) -> Self {
                    Self {
                        dph: Some($crate::DynamixelProtocolHandler::v1()),
                        ..self
                    }
                }
            }
            #[cfg(feature = "python")]
            #[gen_stub_pymethods]
            #[pymethods]
            impl [<$servo_name:camel PyController>] {
                #[new]
                pub fn new(serial_port: &str, baudrate: u32, timeout: f32) -> PyResult<Self> {
                    let serial_port = serialport::new(serial_port, baudrate)
                        .timeout(std::time::Duration::from_secs_f32(timeout))
                        .open()
                        .map_err(|e| pyo3::exceptions::PyIOError::new_err(e.to_string()))?;

                    let c = [<$servo_name:camel Controller>]::new()
                        .with_serial_port(serial_port)
                        .with_protocol_v1();

                    Ok(Self(std::sync::Mutex::new(Some(c))))
                }
            }
        }
    };
    ($servo_name:ident, v2) => {
        paste::paste! {
            impl [<$servo_name:camel Controller>] {
                pub fn with_protocol_v2(
                    self,
                ) -> Self {
                    Self {
                        dph: Some($crate::DynamixelProtocolHandler::v2()),
                        ..self
                    }
                }

                /// Answer every `sync_read_*` with a fast sync read (instruction 0x8A).
                ///
                /// All motors then append their answer to a single status packet instead
                /// of sending one each, which saves a packet header and a bus turnaround
                /// per motor. Needs firmware new enough to implement it (XL330: v46+);
                /// older firmware does not answer and the read times out.
                pub fn with_fast_sync_read(self) -> Self {
                    let dph = self
                        .dph
                        .unwrap_or_else($crate::DynamixelProtocolHandler::v2);
                    Self {
                        dph: Some(dph.with_fast_sync_read()),
                        ..self
                    }
                }

                /// Turn the fast sync read routing of the `sync_read_*` methods on or off.
                pub fn set_fast_sync_read(&mut self, enabled: bool) {
                    self.dph.as_mut().unwrap().set_fast_sync_read(enabled);
                }
            }

            #[cfg(feature = "python")]
            #[gen_stub_pymethods]
            #[pymethods]
            impl [<$servo_name:camel PyController>] {
                /// Turn the fast sync read routing of the `sync_read_*` methods on or off.
                pub fn set_fast_sync_read(&self, enabled: bool) -> PyResult<()> {
                    let mut guard = self.0.lock().unwrap();
                    Self::borrow(&mut guard)
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)?
                        .set_fast_sync_read(enabled);
                    Ok(())
                }
            }

            #[cfg(feature = "python")]
            #[gen_stub_pymethods]
            #[pymethods]
            impl [<$servo_name:camel PyController>] {
                #[new]
                pub fn new(serial_port: &str, baudrate: u32, timeout: f32) -> PyResult<Self> {
                    let serial_port = serialport::new(serial_port, baudrate)
                        .timeout(std::time::Duration::from_secs_f32(timeout))
                        .open()
                        .map_err(|e| pyo3::exceptions::PyIOError::new_err(e.to_string()))?;

                    let c = [<$servo_name:camel Controller>]::new()
                        .with_serial_port(serial_port)
                        .with_protocol_v2();

                    Ok(Self(std::sync::Mutex::new(Some(c))))
                }
            }
        }
    };
}

#[macro_export]
macro_rules! generate_special_instructions {
    ($servo_macro:ident) => {
        paste::paste! {
            impl [<$servo_macro:camel Controller>] {
                pub fn ping(&mut self, id: u8) -> $crate::Result<bool> {
                    let dph = self.dph.as_ref().unwrap();
                    let serial_port = self.serial_port.as_mut().unwrap().as_mut();
                    dph.ping(serial_port, id)
                }

                pub fn reboot(&mut self, id: u8) -> $crate::Result<bool> {
                    let dph = self.dph.as_ref().unwrap();
                    let serial_port = self.serial_port.as_mut().unwrap().as_mut();
                    dph.reboot(serial_port, id)
                }

                pub fn factory_reset(
                    &mut self,
                    id: u8,
                    conserve_id_only: bool,
                    conserve_id_and_baudrate: bool,
                ) -> $crate::Result<()> {
                    let dph = self.dph.as_ref().unwrap();
                    let serial_port = self.serial_port.as_mut().unwrap().as_mut();
                    dph.factory_reset(serial_port, id, conserve_id_only, conserve_id_and_baudrate)
                }
            }
        }
    };
}

#[macro_export]
macro_rules! generate_addr_read_write {
    ($servo_name:ident) => {
        paste::paste! {
            impl [<$servo_name:camel Controller>] {

                pub fn read_raw_data(
                    &mut self,
                    id: u8,
                    addr: u8,
                    length: u8,
                ) -> $crate::Result<Vec<u8>> {
                    let dph = self.dph.as_ref().unwrap();
                    let serial_port = self.serial_port.as_mut().unwrap().as_mut();
                    dph.read(serial_port, id, addr, length)
                }

                pub fn write_raw_data(
                    &mut self,
                    id: u8,
                    addr: u8,
                    data: Vec<u8>,
                ) -> $crate::Result<()> {
                    let dph = self.dph.as_ref().unwrap();
                    let serial_port = self.serial_port.as_mut().unwrap().as_mut();
                    dph.write(serial_port, id, addr, &data)
                }

                /// Same as [`read_raw_data`](Self::read_raw_data), plus the status
                /// packet's error field. See [`$crate::DynamixelProtocolHandler::read_with_error`].
                pub fn read_raw_data_with_error(
                    &mut self,
                    id: u8,
                    addr: u8,
                    length: u8,
                ) -> $crate::Result<(Vec<u8>, $crate::StatusError)> {
                    let dph = self.dph.as_ref().unwrap();
                    let serial_port = self.serial_port.as_mut().unwrap().as_mut();
                    dph.read_with_error(serial_port, id, addr, length)
                }

                /// Same as [`write_raw_data`](Self::write_raw_data), plus the status
                /// packet's error field.
                pub fn write_raw_data_with_error(
                    &mut self,
                    id: u8,
                    addr: u8,
                    data: Vec<u8>,
                ) -> $crate::Result<$crate::StatusError> {
                    let dph = self.dph.as_ref().unwrap();
                    let serial_port = self.serial_port.as_mut().unwrap().as_mut();
                    dph.write_with_error(serial_port, id, addr, &data)
                }

                /// Same as [`sync_read_raw_data`](Self::sync_read_raw_data), plus each
                /// motor's error field.
                pub fn sync_read_raw_data_with_error(
                    &mut self,
                    ids: &[u8],
                    addr: u8,
                    length: u8,
                ) -> $crate::Result<Vec<(Vec<u8>, $crate::StatusError)>> {
                    let dph = self.dph.as_ref().unwrap();
                    let serial_port = self.serial_port.as_mut().unwrap().as_mut();
                    dph.sync_read_with_error(serial_port, ids, addr, length)
                }

                pub fn sync_read_raw_data(
                    &mut self,
                    ids: &[u8],
                    addr: u8,
                    length: u8,
                ) -> $crate::Result<Vec<Vec<u8>>> {
                    let dph = self.dph.as_ref().unwrap();
                    let serial_port = self.serial_port.as_mut().unwrap().as_mut();
                    dph.sync_read(serial_port, ids, addr, length)
                }

                pub fn sync_write_raw_data(
                    &mut self,
                    ids: &[u8],
                    addr: u8,
                    data: &[Vec<u8>],
                ) -> $crate::Result<()> {
                    let dph = self.dph.as_ref().unwrap();
                    let serial_port = self.serial_port.as_mut().unwrap().as_mut();
                    dph.sync_write(serial_port, ids, addr, data)
                }
            }

            #[cfg(feature = "python")]
            #[gen_stub_pymethods]
            #[pymethods]
            impl [<$servo_name:camel PyController>] {
                pub fn read_raw_data(
                    &self,
                    py: Python,
                    id: u8,
                    addr: u8,
                    length: u8,
                ) -> PyResult<Py<PyAny>> {


                    let x = py
                        .detach(|| {
                            let mut guard = self.0.lock().unwrap();
                            Self::borrow(&mut guard)?
                                .read_raw_data(id, addr, length)
                                .map_err(|e| e.to_string())
                        })
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)?;
                    let l = pyo3::types::PyList::new(py, x)?;

                    Ok(l.into())
                }

                pub fn write_raw_data(
                    &self,
                    py: Python,
                    id: u8,
                    addr: u8,
                    data: &Bound<'_, pyo3::types::PyList>,
                ) -> PyResult<()> {
                    let data = data.extract::<Vec<u8>>()?;

                    py
                        .detach(|| {
                            let mut guard = self.0.lock().unwrap();
                            Self::borrow(&mut guard)?
                                .write_raw_data(id, addr, data)
                                .map_err(|e| e.to_string())
                        })
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)?;
                    Ok(())
                }

                /// Read raw bytes and the status packet's error field.
                ///
                /// The error is the raw byte. On protocol v1 it is a bitfield of motor
                /// conditions; on v2, bits 0-6 are an instruction-error number and bit 7
                /// is the alert flag.
                pub fn read_raw_data_with_error(
                    &self,
                    py: Python,
                    id: u8,
                    addr: u8,
                    length: u8,
                ) -> PyResult<(Py<PyAny>, u8)> {
                    let (x, err) = py
                        .detach(|| {
                            let mut guard = self.0.lock().unwrap();
                            Self::borrow(&mut guard)?
                                .read_raw_data_with_error(id, addr, length)
                                .map_err(|e| e.to_string())
                        })
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)?;
                    let l = pyo3::types::PyList::new(py, x)?;

                    Ok((l.into(), err.byte()))
                }

                /// Sync read raw bytes and each motor's error field.
                ///
                /// Returns one (values, error) pair per id, in the order they were asked
                /// for. See `read_raw_data_with_error` for how to read the byte.
                pub fn sync_read_raw_data_with_error(
                    &self,
                    py: Python,
                    ids: &Bound<'_, pyo3::types::PyList>,
                    addr: u8,
                    length: u8,
                ) -> PyResult<Vec<(Py<PyAny>, u8)>> {
                    let ids = ids.extract::<Vec<u8>>()?;

                    let values = py
                        .detach(|| {
                            let mut guard = self.0.lock().unwrap();
                            Self::borrow(&mut guard)?
                                .sync_read_raw_data_with_error(&ids, addr, length)
                                .map_err(|e| e.to_string())
                        })
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)?;

                    values
                        .into_iter()
                        .map(|(x, err)| {
                            let l = pyo3::types::PyList::new(py, x)?;
                            Ok((l.into(), err.byte()))
                        })
                        .collect()
                }

                /// Write raw bytes and return the status packet's error field.
                pub fn write_raw_data_with_error(
                    &self,
                    py: Python,
                    id: u8,
                    addr: u8,
                    data: &Bound<'_, pyo3::types::PyList>,
                ) -> PyResult<u8> {
                    let data = data.extract::<Vec<u8>>()?;

                    py
                        .detach(|| {
                            let mut guard = self.0.lock().unwrap();
                            Self::borrow(&mut guard)?
                                .write_raw_data_with_error(id, addr, data)
                                .map_err(|e| e.to_string())
                        })
                        .map(|err| err.byte())
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)
                }

                pub fn sync_read_raw_data(
                    &self,
                    py: Python,
                    ids: &Bound<'_, pyo3::types::PyList>,
                    addr: u8,
                    length: u8,
                ) -> PyResult<Py<PyAny>> {
                    let ids = ids.extract::<Vec<u8>>()?;

                    let x = py
                        .detach(|| {
                            let mut guard = self.0.lock().unwrap();
                            Self::borrow(&mut guard)?
                                .sync_read_raw_data(&ids, addr, length)
                                .map_err(|e| e.to_string())
                        })
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)?;
                    let l = pyo3::types::PyList::new(py, x)?;

                    Ok(l.into())
                }

                pub fn sync_write_raw_data(
                    &self,
                    py: Python,
                    ids: &Bound<'_, pyo3::types::PyList>,
                    addr: u8,
                    data: &Bound<'_, pyo3::types::PyList>,
                ) -> PyResult<()> {
                    let ids = ids.extract::<Vec<u8>>()?;
                    let data = data.extract::<Vec<Vec<u8>>>()?;

                    py
                        .detach(|| {
                            let mut guard = self.0.lock().unwrap();
                            Self::borrow(&mut guard)?
                                .sync_write_raw_data(&ids, addr, &data)
                                .map_err(|e| e.to_string())
                        })
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)?;
                    Ok(())
                }

                pub fn ping(&self, py: Python, id: u8) -> PyResult<bool> {
                    py
                        .detach(|| {
                            let mut guard = self.0.lock().unwrap();
                            Self::borrow(&mut guard)?
                                .ping(id)
                                .map_err(|e| e.to_string())
                        })
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)
                }

                pub fn reboot(&self, py: Python, id: u8) -> PyResult<bool> {
                    py
                        .detach(|| {
                            let mut guard = self.0.lock().unwrap();
                            Self::borrow(&mut guard)?
                                .reboot(id)
                                .map_err(|e| e.to_string())
                        })
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)
                }

                #[pyo3(signature = (
                    id,
                    conserve_id_only = true,
                    conserve_id_and_baudrate = true
                ))]
                pub fn factory_reset(
                    &self,
                    py: Python,
                    id: u8,
                    conserve_id_only: bool,
                    conserve_id_and_baudrate: bool,
                ) -> PyResult<()> {
                    py
                        .detach(|| {
                            let mut guard = self.0.lock().unwrap();
                            Self::borrow(&mut guard)?
                                .factory_reset(id, conserve_id_only, conserve_id_and_baudrate)
                                .map_err(|e| e.to_string())
                        })
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)
                }
            }
        }
    };
}

/// Integer access to registers chosen by name, for code that picks them at runtime.
///
/// The raw address API hands back bytes; these methods apply what the definition
/// states about them -- the servo's word order and each register's sign encoding --
/// so a Feetech sign-magnitude position or a Dynamixel two's complement offset
/// comes and goes as the value it stands for. The generated `read_<name>` methods
/// remain the typed way for a register known in source.
#[macro_export]
macro_rules! generate_register_access {
    ($servo_name:ident) => {
        paste::paste! {
            impl [<$servo_name:camel Controller>] {
                fn named(name: &str) -> $crate::Result<$crate::servo::RegisterInfo> {
                    register(name).ok_or_else(|| {
                        $crate::servo::RegisterError::Unknown(name.to_string()).into()
                    })
                }

                /// The definition motor `id` is addressed through by name: the one given
                /// to [`set_definition`](Self::set_definition), or this controller's own.
                pub fn definition_of(&self, id: u8) -> $crate::servo::ServoDefinition {
                    self.definitions.get(&id).copied().unwrap_or(DEFINITION)
                }

                /// Address motor `id` through `definition` in the by-name access, for a
                /// bus that mixes servos of several definitions on one port.
                ///
                /// Its registers are then looked up, laid out and signed as `definition`
                /// states. The typed accessors, the raw address API and `scan` keep this
                /// controller's own definition. `definition` must speak the same protocol.
                pub fn set_definition(
                    &mut self,
                    id: u8,
                    definition: $crate::servo::ServoDefinition,
                ) -> $crate::Result<()> {
                    if definition.protocol != DEFINITION.protocol {
                        return Err(format!(
                            "{} speaks protocol v{}, this controller speaks v{}",
                            definition.name, definition.protocol, DEFINITION.protocol
                        )
                        .into());
                    }
                    self.definitions.insert(id, definition);
                    Ok(())
                }

                /// Register `name` of motor `id`, with the word order it is laid out in.
                fn named_for(
                    &self,
                    id: u8,
                    name: &str,
                ) -> $crate::Result<($crate::servo::RegisterInfo, $crate::servo::WordOrder)> {
                    let definition = self.definition_of(id);
                    let reg = definition
                        .register(name)
                        .ok_or_else(|| $crate::servo::RegisterError::Unknown(name.to_string()))?;
                    Ok((reg, definition.info.word_order))
                }

                /// Register `name` of each of `ids`. One Sync Read or Sync Write carries a
                /// single address and length, so it must sit at the same place on all.
                fn named_for_all(
                    &self,
                    ids: &[u8],
                    name: &str,
                ) -> $crate::Result<Vec<($crate::servo::RegisterInfo, $crate::servo::WordOrder)>> {
                    let regs = ids
                        .iter()
                        .map(|&id| self.named_for(id, name))
                        .collect::<$crate::Result<Vec<_>>>()?;
                    if regs
                        .windows(2)
                        .any(|pair| (pair[0].0.addr, pair[0].0.size) != (pair[1].0.addr, pair[1].0.size))
                    {
                        return Err($crate::servo::RegisterError::Layout(name.to_string()).into());
                    }
                    Ok(regs)
                }

                /// Whether the firmware of every one of `ids` answers Sync Read.
                fn answers_sync_read(&self, ids: &[u8]) -> bool {
                    ids.iter().all(|&id| self.definition_of(id).info.supports_sync_read)
                }

                /// Read register `name` as an integer, decoded from the word order and
                /// sign encoding of the motor's definition.
                pub fn read_register(&mut self, id: u8, name: &str) -> $crate::Result<i64> {
                    let (reg, order) = self.named_for(id, name)?;
                    let bytes = self.read_raw_data(id, reg.addr, reg.size)?;
                    Ok(reg.decode(order, &bytes)?)
                }

                /// Same as [`read_register`](Self::read_register), plus the status
                /// packet's error field.
                pub fn read_register_with_error(
                    &mut self,
                    id: u8,
                    name: &str,
                ) -> $crate::Result<(i64, $crate::StatusError)> {
                    let (reg, order) = self.named_for(id, name)?;
                    let (bytes, error) = self.read_raw_data_with_error(id, reg.addr, reg.size)?;
                    Ok((reg.decode(order, &bytes)?, error))
                }

                /// Write `value` to register `name`, laid out in the word order and sign
                /// encoding of the motor's definition. A value that does not fit the
                /// register fails before anything reaches the bus.
                pub fn write_register(&mut self, id: u8, name: &str, value: i64) -> $crate::Result<()> {
                    let (reg, order) = self.named_for(id, name)?;
                    self.write_raw_data(id, reg.addr, reg.encode(order, value)?)
                }

                /// Same as [`write_register`](Self::write_register), plus the status
                /// packet's error field.
                pub fn write_register_with_error(
                    &mut self,
                    id: u8,
                    name: &str,
                    value: i64,
                ) -> $crate::Result<$crate::StatusError> {
                    let (reg, order) = self.named_for(id, name)?;
                    self.write_raw_data_with_error(id, reg.addr, reg.encode(order, value)?)
                }

                /// Sync read register `name` from `ids`, each value decoded like
                /// [`read_register`](Self::read_register).
                ///
                /// When the firmware of one of `ids` has no Sync Read (`supports_sync_read`
                /// false in its definition: the Feetech SCS series), they are read one id
                /// at a time instead, in the order asked. The values come back the same
                /// way, but from one transaction per id rather than one for the whole bus.
                /// Otherwise the register must sit at the same address and size in every
                /// motor's definition (`RegisterError::Layout`).
                pub fn sync_read_register(&mut self, ids: &[u8], name: &str) -> $crate::Result<Vec<i64>> {
                    if !self.answers_sync_read(ids) {
                        return ids.iter().map(|&id| self.read_register(id, name)).collect();
                    }
                    let regs = self.named_for_all(ids, name)?;
                    let Some(&(first, _)) = regs.first() else {
                        return Ok(Vec::new());
                    };
                    let answers = self.sync_read_raw_data(ids, first.addr, first.size)?;
                    regs.iter()
                        .zip(answers)
                        .map(|(&(reg, order), bytes)| Ok(reg.decode(order, &bytes)?))
                        .collect()
                }

                /// Same as [`sync_read_register`](Self::sync_read_register), plus each
                /// motor's error field, with the same fallback to one read per id.
                pub fn sync_read_register_with_error(
                    &mut self,
                    ids: &[u8],
                    name: &str,
                ) -> $crate::Result<Vec<(i64, $crate::StatusError)>> {
                    if !self.answers_sync_read(ids) {
                        return ids
                            .iter()
                            .map(|&id| self.read_register_with_error(id, name))
                            .collect();
                    }
                    let regs = self.named_for_all(ids, name)?;
                    let Some(&(first, _)) = regs.first() else {
                        return Ok(Vec::new());
                    };
                    let answers = self.sync_read_raw_data_with_error(ids, first.addr, first.size)?;
                    regs.iter()
                        .zip(answers)
                        .map(|(&(reg, order), (bytes, error))| Ok((reg.decode(order, &bytes)?, error)))
                        .collect()
                }

                /// Sync write `values` to register `name` of `ids`, one value per id,
                /// each encoded like [`write_register`](Self::write_register). The
                /// register must sit at the same address and size in every motor's
                /// definition (`RegisterError::Layout`).
                pub fn sync_write_register(
                    &mut self,
                    ids: &[u8],
                    name: &str,
                    values: &[i64],
                ) -> $crate::Result<()> {
                    let regs = self.named_for_all(ids, name)?;
                    let Some(&(first, _)) = regs.first() else {
                        return Ok(());
                    };
                    let data = regs
                        .iter()
                        .zip(values)
                        .map(|(&(reg, order), &value)| reg.encode(order, value))
                        .collect::<Result<Vec<_>, _>>()?;
                    self.sync_write_raw_data(ids, first.addr, &data)
                }

                /// Run `op`, and run it again up to `retries` more times while it fails
                /// on the bus.
                ///
                /// A timeout or a corrupted status packet is worth another try, and each
                /// attempt starts with the pre-send flush. A bad name or value is not: it
                /// fails at once, before anything reaches the bus. A motor that answers
                /// with a fault did answer, so the `_with_error` variants hand its error
                /// field back rather than retry. For instance,
                /// `c.with_retries(3, |c| c.read_register(1, "present_position"))`.
                pub fn with_retries<T>(
                    &mut self,
                    retries: u32,
                    mut op: impl FnMut(&mut Self) -> $crate::Result<T>,
                ) -> $crate::Result<T> {
                    let mut attempt = 0;
                    loop {
                        match op(self) {
                            Err(e) if attempt < retries && !e.is::<$crate::servo::RegisterError>() => {
                                attempt += 1
                            }
                            result => return result,
                        }
                    }
                }
            }

            #[cfg(feature = "python")]
            impl [<$servo_name:camel PyController>] {
                /// Run a name-based access with the GIL released, telling a bad name or
                /// value (`ValueError`) apart from a bus failure (`RuntimeError`).
                fn by_name<T: Send>(
                    &self,
                    py: Python,
                    op: impl FnOnce(&mut [<$servo_name:camel Controller>]) -> $crate::Result<T> + Send,
                ) -> PyResult<T> {
                    py.detach(|| {
                        let mut guard = self.0.lock().unwrap();
                        let controller = Self::borrow(&mut guard).map_err(|e| (false, e))?;
                        op(controller)
                            .map_err(|e| (e.is::<$crate::servo::RegisterError>(), e.to_string()))
                    })
                    .map_err(|(bad_argument, message)| {
                        if bad_argument {
                            pyo3::exceptions::PyValueError::new_err(message)
                        } else {
                            pyo3::exceptions::PyRuntimeError::new_err(message)
                        }
                    })
                }
            }

            #[cfg(feature = "python")]
            #[gen_stub_pymethods]
            #[pymethods]
            impl [<$servo_name:camel PyController>] {
                /// Address motor `id` through `definition` in the by-name methods, for a
                /// bus mixing servos of several definitions: on an XL430 controller,
                /// `set_definition(2, Xl330PyController.definition())`. The typed
                /// methods, the raw address ones and `scan` keep this controller's own
                /// definition. A definition of another protocol raises `ValueError`.
                pub fn set_definition(&self, id: u8, definition: $crate::servo::ServoDefinition) -> PyResult<()> {
                    let mut guard = self.0.lock().unwrap();
                    Self::borrow(&mut guard)
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)?
                        .set_definition(id, definition)
                        .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))
                }

                /// Read register `name` as an integer, decoded from the word order and
                /// sign encoding of the motor's definition. A bus failure is tried again
                /// up to `retries` more times; a bad name never is.
                #[pyo3(signature = (id, name, retries = 0))]
                pub fn read_register(&self, py: Python, id: u8, name: String, retries: u32) -> PyResult<i64> {
                    self.by_name(py, |c| c.with_retries(retries, |c| c.read_register(id, &name)))
                }

                /// Same as `read_register`, plus the status packet's error field. See
                /// `read_raw_data_with_error` for how to read the byte. A motor that
                /// answers with a fault is not tried again.
                #[pyo3(signature = (id, name, retries = 0))]
                pub fn read_register_with_error(
                    &self,
                    py: Python,
                    id: u8,
                    name: String,
                    retries: u32,
                ) -> PyResult<(i64, u8)> {
                    self.by_name(py, |c| c.with_retries(retries, |c| c.read_register_with_error(id, &name)))
                        .map(|(value, error)| (value, error.byte()))
                }

                /// Write `value` to register `name`, laid out in the servo's word order
                /// and the register's sign encoding. A value that does not fit the
                /// register, or a name the servo does not define, raises `ValueError`
                /// before anything reaches the bus. A bus failure is tried again up to
                /// `retries` more times.
                #[pyo3(signature = (id, name, value, retries = 0))]
                pub fn write_register(
                    &self,
                    py: Python,
                    id: u8,
                    name: String,
                    value: i64,
                    retries: u32,
                ) -> PyResult<()> {
                    self.by_name(py, |c| c.with_retries(retries, |c| c.write_register(id, &name, value)))
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
                    self.by_name(py, |c| {
                        c.with_retries(retries, |c| c.write_register_with_error(id, &name, value))
                    })
                    .map(|error| error.byte())
                }

                /// Sync read register `name` from `ids`, each value decoded like
                /// `read_register`, with the same `retries`. A servo without Sync Read
                /// (`supports_sync_read()`) is read one id at a time instead.
                #[pyo3(signature = (ids, name, retries = 0))]
                pub fn sync_read_register(
                    &self,
                    py: Python,
                    ids: Vec<u8>,
                    name: String,
                    retries: u32,
                ) -> PyResult<Vec<i64>> {
                    self.by_name(py, |c| c.with_retries(retries, |c| c.sync_read_register(&ids, &name)))
                }

                /// Same as `sync_read_register`, plus each motor's error field, as one
                /// (value, error) pair per id in the order they were asked for.
                #[pyo3(signature = (ids, name, retries = 0))]
                pub fn sync_read_register_with_error(
                    &self,
                    py: Python,
                    ids: Vec<u8>,
                    name: String,
                    retries: u32,
                ) -> PyResult<Vec<(i64, u8)>> {
                    self.by_name(py, |c| {
                        c.with_retries(retries, |c| c.sync_read_register_with_error(&ids, &name))
                    })
                    .map(|values| values.into_iter().map(|(v, e)| (v, e.byte())).collect())
                }

                /// Sync write `values` to register `name` of `ids`, one value per id,
                /// each encoded like `write_register`, with the same `retries`.
                #[pyo3(signature = (ids, name, values, retries = 0))]
                pub fn sync_write_register(
                    &self,
                    py: Python,
                    ids: Vec<u8>,
                    name: String,
                    values: Vec<i64>,
                    retries: u32,
                ) -> PyResult<()> {
                    self.by_name(py, |c| {
                        c.with_retries(retries, |c| c.sync_write_register(&ids, &name, &values))
                    })
                }
            }
        }
    };
}

/// A sweep of the bus: which ids answer, and what they are.
#[macro_export]
macro_rules! generate_scan {
    ($servo_name:ident) => {
        paste::paste! {
            impl [<$servo_name:camel Controller>] {
                /// Which of `ids` answer, with their model number.
                ///
                /// One Model Number read per id, so presence and identity cost a single
                /// round trip. An absent id costs a timeout, so the sweep runs under one
                /// sized to the baud rate, see [`scan_timeout`](crate::servo::scan_timeout),
                /// and puts the port's timeout back afterwards. An id that answers with
                /// anything the protocol cannot parse counts as absent.
                pub fn scan(&mut self, ids: &[u8]) -> $crate::Result<std::collections::BTreeMap<u8, u16>> {
                    let reg = Self::named("model_number")?;
                    let port = self.serial_port.as_mut().unwrap();
                    let timeout = port.timeout();
                    port.set_timeout($crate::servo::scan_timeout(port.baud_rate()?))?;
                    let found = ids
                        .iter()
                        .filter_map(|&id| {
                            let bytes = self.read_raw_data(id, reg.addr, reg.size).ok()?;
                            let model = reg.decode(INFO.word_order, &bytes).ok()?;
                            Some((id, model as u16))
                        })
                        .collect();
                    self.serial_port.as_mut().unwrap().set_timeout(timeout)?;
                    Ok(found)
                }

                /// [`scan`](Self::scan) every id the protocol allows, from 0 to
                /// [`max_id`](crate::DynamixelProtocolHandler::max_id).
                pub fn scan_all(&mut self) -> $crate::Result<std::collections::BTreeMap<u8, u16>> {
                    let ids: Vec<u8> = (0..=self.dph.as_ref().unwrap().max_id()).collect();
                    self.scan(&ids)
                }
            }

            #[cfg(feature = "python")]
            #[gen_stub_pymethods]
            #[pymethods]
            impl [<$servo_name:camel PyController>] {
                /// Which of `ids` answer, as {id: model number}; every id the protocol
                /// allows when `ids` is left out (0 to 253 on v1, 0 to 252 on v2).
                ///
                /// One Model Number read per id, under a timeout sized to the baud rate
                /// so that absent ids do not each cost the port's timeout; the port's
                /// timeout is put back afterwards.
                #[pyo3(signature = (ids = None))]
                pub fn scan(
                    &self,
                    py: Python,
                    ids: Option<Vec<u8>>,
                ) -> PyResult<std::collections::BTreeMap<u8, u16>> {
                    self.by_name(py, |c| match &ids {
                        Some(ids) => c.scan(ids),
                        None => c.scan_all(),
                    })
                }
            }
        }
    };
}

/// Maps the access ident of a servo definition (`r`, `w`, `rw`) to a
/// [`RegisterAccess`](crate::servo::RegisterAccess).
#[macro_export]
macro_rules! register_access {
    (r) => {
        $crate::servo::RegisterAccess::Read
    };
    (w) => {
        $crate::servo::RegisterAccess::Write
    };
    (rw) => {
        $crate::servo::RegisterAccess::ReadWrite
    };
}

/// Maps an `encoding:` entry of a servo definition (`unsigned`, `twos_complement`,
/// `sign_magnitude(bit)`) to an [`Encoding`](crate::servo::Encoding).
#[macro_export]
macro_rules! register_encoding {
    (unsigned) => {
        $crate::servo::Encoding::Unsigned
    };
    (twos_complement) => {
        $crate::servo::Encoding::TwosComplement
    };
    (sign_magnitude($bit:expr)) => {
        $crate::servo::Encoding::SignMagnitude { sign_bit: $bit }
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! servo_resolution {
    () => {
        None
    };
    ($resolution:expr) => {
        Some($resolution)
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! servo_word_order {
    () => {
        $crate::servo::WordOrder::Little
    };
    (little) => {
        $crate::servo::WordOrder::Little
    };
    (big) => {
        $crate::servo::WordOrder::Big
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! protocol_version {
    (v1) => {
        1
    };
    (v2) => {
        2
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! servo_supports_sync_read {
    () => {
        true
    };
    ($supports_sync_read:expr) => {
        $supports_sync_read
    };
}

#[macro_export]
macro_rules! generate_reg_access {
    ($servo_name:ident, $reg_name:ident, r, $reg_addr:expr, $reg_type:ty, $conv:ident) => {
        $crate::generate_reg_read!($servo_name, $reg_name, $reg_addr, $reg_type, $conv);
    };
    ($servo_name:ident, $reg_name:ident, w, $reg_addr:expr, $reg_type:ty, $conv:ident) => {
        $crate::generate_reg_write!($servo_name, $reg_name, $reg_addr, $reg_type, $conv);
    };
    ($servo_name:ident, $reg_name:ident, rw, $reg_addr:expr, $reg_type:ty, $conv:ident) => {
        $crate::generate_reg_read!($servo_name, $reg_name, $reg_addr, $reg_type, $conv);
        $crate::generate_reg_write!($servo_name, $reg_name, $reg_addr, $reg_type, $conv);
    };
}
#[macro_export]
macro_rules! generate_reg_read {
    ($servo_name:ident, $reg_name:ident, $reg_addr:expr, $reg_type:ty, None) => {
        paste::paste! {
            #[doc = concat!("Read register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: ", stringify!($reg_type), ")")]
            pub fn [<read_ $reg_name>](
                io: &$crate::DynamixelProtocolHandler,
                serial_port: &mut dyn serialport::SerialPort,
                id: u8,
            ) -> $crate::Result<$reg_type> {
                let val = io.read(serial_port, id, $reg_addr, size_of::<$reg_type>().try_into().unwrap())?;
                let val = $reg_type::from_le_bytes(val.try_into().unwrap());

                Ok(val)
            }

            #[doc = concat!("Sync read register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: ", stringify!($reg_type), ")")]
            pub fn [<sync_read_ $reg_name>](
                io: &$crate::DynamixelProtocolHandler,
                serial_port: &mut dyn serialport::SerialPort,
                ids: &[u8],
            ) -> $crate::Result<Vec<$reg_type>> {
                let val: Vec<Vec<u8>> = io.sync_read(serial_port, ids, $reg_addr, size_of::<$reg_type>().try_into().unwrap())?;
                let val = val
                    .iter()
                    .map(|v| $reg_type::from_le_bytes(v.as_slice().try_into().unwrap()))
                    .collect();

                Ok(val)
            }

            impl [<$servo_name:camel Controller>] {
                #[doc = concat!("Sync read register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<sync_read_ $reg_name>](
                    &mut self,
                    ids: &[u8],
                ) -> $crate::Result<Vec<$reg_type>> {
                    [<sync_read_ $reg_name>](
                        self.dph.as_ref().unwrap(),
                        self.serial_port.as_mut().unwrap().as_mut(),
                        ids,
                    )
                }
            }


            impl [<$servo_name:camel Controller>] {
                #[doc = concat!("Read register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<read_ $reg_name>](
                    &mut self,
                    id: u8,
                ) -> $crate::Result<Vec<$reg_type>> {
                    let r= match [<read_ $reg_name>](
                        self.dph.as_ref().unwrap(),
                        self.serial_port.as_mut().unwrap().as_mut(),
                        id,
                    ){
                        Ok(r) =>Ok(vec![r]),
                        Err(e) => Err(e),
                    };
                    r
                }
            }


            #[cfg(feature = "python")]
            #[gen_stub_pymethods]
            #[pymethods]
            impl [<$servo_name:camel PyController>] {
                #[doc = concat!("Sync read register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<sync_read_ $reg_name>](
                    &self,
                    py: Python,
                    ids: &Bound<'_, pyo3::types::PyList>,
                ) -> PyResult<Py<PyAny>> {
                    let ids = ids.extract::<Vec<u8>>()?;

                    let x = py
                        .detach(|| {
                            let mut guard = self.0.lock().unwrap();
                            Self::borrow(&mut guard)?
                                .[<sync_read_ $reg_name>](&ids)
                                .map_err(|e| e.to_string())
                        })
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)?;
                    let l = pyo3::types::PyList::new(py, x)?;

                    Ok(l.into())
                }
            }


            #[cfg(feature = "python")]
            #[gen_stub_pymethods]
            #[pymethods]
            impl [<$servo_name:camel PyController>] {
                #[doc = concat!("Read register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<read_ $reg_name>](
                    &self,
                    py: Python,
                    id: u8,
                ) -> PyResult<Py<PyAny>> {

                    let x = py
                        .detach(|| {
                            let mut guard = self.0.lock().unwrap();
                            Self::borrow(&mut guard)?
                                .[<read_ $reg_name>](id)
                                .map_err(|e| e.to_string())
                        })
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)?;
                    let l = pyo3::types::PyList::new(py, x)?;

                    Ok(l.into())
                }
            }


        }
    };
    ($servo_name:ident, $reg_name:ident, $reg_addr:expr, $reg_type:ty, $conv:ident) => {
        paste::paste! {
            #[doc = concat!("Read raw register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: ", stringify!($reg_type), ")")]
            pub fn [<read_raw_ $reg_name>](
                io: &$crate::DynamixelProtocolHandler,
                serial_port: &mut dyn serialport::SerialPort,
                id: u8,
            ) -> $crate::Result<$reg_type> {
                let val = io.read(serial_port, id, $reg_addr, size_of::<$reg_type>().try_into().unwrap())?;
                let val = $reg_type::from_le_bytes(val.try_into().unwrap());

                Ok(val)
            }

            #[doc = concat!("Read register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", converted by ", stringify!($conv), ")")]
            pub fn [<read_ $reg_name>](
                io: &$crate::DynamixelProtocolHandler,
                serial_port: &mut dyn serialport::SerialPort,
                id: u8,
            ) -> $crate::Result<<$conv as Conversion>::UsiType> {
                let val = [<read_raw_ $reg_name>](io, serial_port, id)?;
                let val = $conv::from_raw(val);
                Ok(val)
            }

            #[doc = concat!("Sync read raw register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: ", stringify!($reg_type), ")")]
            pub fn [<sync_read_raw_ $reg_name>](
                io: &$crate::DynamixelProtocolHandler,
                serial_port: &mut dyn serialport::SerialPort,
                ids: &[u8],
            ) -> $crate::Result<Vec<$reg_type>> {
                let val: Vec<Vec<u8>> = io.sync_read(serial_port, ids, $reg_addr, size_of::<$reg_type>().try_into().unwrap())?;
                let val = val
                    .iter()
                    .map(|v| $reg_type::from_le_bytes(v.as_slice().try_into().unwrap()))
                    .collect();

                Ok(val)
            }

            #[doc = concat!("Sync read register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", converted by ", stringify!($conv), ")")]
            pub fn [<sync_read_ $reg_name>](
                io: &$crate::DynamixelProtocolHandler,
                serial_port: &mut dyn serialport::SerialPort,
                ids: &[u8],
            ) -> $crate::Result<Vec<<$conv as Conversion>::UsiType>> {
                let val = [<sync_read_raw_ $reg_name>](io, serial_port, ids)?;
                let val = val
                    .iter()
                    .map(|&v| $conv::from_raw(v))
                    .collect();

                Ok(val)
            }

            impl [<$servo_name:camel Controller>] {
                #[doc = concat!("Sync read raw register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<sync_read_raw_ $reg_name>](
                    &mut self,
                    ids: &[u8],
                ) -> $crate::Result<Vec<$reg_type>> {
                    [<sync_read_raw_ $reg_name>](
                        self.dph.as_ref().unwrap(),
                        self.serial_port.as_mut().unwrap().as_mut(),
                        ids,
                    )
                }

                #[doc = concat!("Sync read register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", converted by ", stringify!($conv), ")")]
                pub fn [<sync_read_ $reg_name>](
                    &mut self,
                    ids: &[u8],
                ) -> $crate::Result<Vec<<$conv as Conversion>::UsiType>> {
                    [<sync_read_ $reg_name>](
                        self.dph.as_ref().unwrap(),
                        self.serial_port.as_mut().unwrap().as_mut(),
                        ids,
                    )
                }

                #[doc = concat!("Read raw register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<read_raw_ $reg_name>](
                    &mut self,
                    id: u8,
                ) -> $crate::Result<Vec<$reg_type>> {
                    let r=match([<read_raw_ $reg_name>](
                        self.dph.as_ref().unwrap(),
                        self.serial_port.as_mut().unwrap().as_mut(),
                        id,
                    ))
                    {
                        Ok(r) => Ok(vec![r]),
                        Err(e) => Err(e),
                    };
                    r
                }

                #[doc = concat!("Read register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", converted by ", stringify!($conv), ")")]
                pub fn [<read_ $reg_name>](
                    &mut self,
                    id: u8,
                ) -> $crate::Result< Vec<<$conv as Conversion>::UsiType  >> {
                    let r=match([<read_ $reg_name>](
                        self.dph.as_ref().unwrap(),
                        self.serial_port.as_mut().unwrap().as_mut(),
                        id,
                    )){
                        Ok(r) => Ok(vec![r]),
                        Err(e) => Err(e),
                    };
                    r
                }


            }

            #[cfg(feature = "python")]
            #[gen_stub_pymethods]
            #[pymethods]
            impl [<$servo_name:camel PyController>] {
                #[doc = concat!("Sync read raw register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<sync_read_raw_ $reg_name>](
                    &self,
                    py: Python,
                    ids: &Bound<'_, pyo3::types::PyList>,
                ) -> PyResult<Py<PyAny>> {
                    let ids = ids.extract::<Vec<u8>>()?;

                    let x = py
                        .detach(|| {
                            let mut guard = self.0.lock().unwrap();
                            Self::borrow(&mut guard)?
                                .[<sync_read_raw_ $reg_name>](&ids)
                                .map_err(|e| e.to_string())
                        })
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)?;
                    let l = pyo3::types::PyList::new(py, x)?;
                    Ok(l.into())
                }

                #[doc = concat!("Sync read register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", converted by ", stringify!($conv), ")")]
                pub fn [<sync_read_ $reg_name>](
                    &self,
                    py: Python,
                    ids: Bound<'_, pyo3::types::PyList>,
                ) -> PyResult<Py<PyAny>> {
                    let ids = ids.extract::<Vec<u8>>()?;

                    let x = py
                        .detach(|| {
                            let mut guard = self.0.lock().unwrap();
                            Self::borrow(&mut guard)?
                                .[<sync_read_ $reg_name>](&ids)
                                .map_err(|e| e.to_string())
                        })
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)?;
                    let l = pyo3::types::PyList::new(py, x)?;
                    Ok(l.into())
                }

                #[doc = concat!("Read raw register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<read_raw_ $reg_name>](
                    &self,
                    py: Python,
                    id: u8,
                ) -> PyResult<Py<PyAny>> {


                    let x = py
                        .detach(|| {
                            let mut guard = self.0.lock().unwrap();
                            Self::borrow(&mut guard)?
                                .[<read_raw_ $reg_name>](id)
                                .map_err(|e| e.to_string())
                        })
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)?;
                    let l = pyo3::types::PyList::new(py, x)?;
                    Ok(l.into())
                }

                #[doc = concat!("Read register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", converted by ", stringify!($conv), ")")]
                pub fn [<read_ $reg_name>](
                    &self,
                    py: Python,
                    id: u8,
                ) -> PyResult<Py<PyAny>> {


                    let x = py
                        .detach(|| {
                            let mut guard = self.0.lock().unwrap();
                            Self::borrow(&mut guard)?
                                .[<read_ $reg_name>](id)
                                .map_err(|e| e.to_string())
                        })
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)?;
                    let l = pyo3::types::PyList::new(py, x)?;
                    Ok(l.into())
                }

            }

        }
    };
}
#[macro_export]
macro_rules! generate_reg_write {
    ($servo_name:ident, $reg_name:ident, $reg_addr:expr, $reg_type:ty, None) => {
        paste::paste! {
            #[doc = concat!("Write register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: ", stringify!($reg_type), ")")]
            pub fn [<write_ $reg_name>](
                io: &$crate::DynamixelProtocolHandler,
                serial_port: &mut dyn serialport::SerialPort,
                id: u8,
                val: $reg_type,
            ) -> $crate::Result<()> {
                io.write(serial_port, id, $reg_addr, &val.to_le_bytes())
            }

            #[doc = concat!("Sync write register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: ", stringify!($reg_type), ")")]
            pub fn [<sync_write_ $reg_name>](
                io: &$crate::DynamixelProtocolHandler,
                serial_port: &mut dyn serialport::SerialPort,
                ids: &[u8],
                values: &[$reg_type],
            ) -> $crate::Result<()> {
                io.sync_write(
                    serial_port,
                    ids,
                    $reg_addr,
                    &values
                        .iter()
                        .map(|v| v.to_le_bytes().to_vec())
                        .collect::<Vec<Vec<u8>>>(),
                )
            }

            impl [<$servo_name:camel Controller>] {
                #[doc = concat!("Sync write register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<sync_write_ $reg_name>](
                    &mut self,
                    ids: &[u8],
                    values: &[$reg_type],
                ) -> $crate::Result<()> {
                    [<sync_write_ $reg_name>](
                        self.dph.as_ref().unwrap(),
                        self.serial_port.as_mut().unwrap().as_mut(),
                        ids,
                        values,
                    )
                }


                #[doc = concat!("Write register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<write_ $reg_name>](
                    &mut self,
                    id: u8,
                    value: $reg_type,
                ) -> $crate::Result<()> {
                    [<write_ $reg_name>](
                        self.dph.as_ref().unwrap(),
                        self.serial_port.as_mut().unwrap().as_mut(),
                        id,
                        value,
                    )
                }

            }

            #[cfg(feature = "python")]
            #[gen_stub_pymethods]
            #[pymethods]
            impl [<$servo_name:camel PyController>] {
                #[doc = concat!("Sync write register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<sync_write_ $reg_name>](
                    &self,
                    py: Python,
                    ids: Bound<'_, pyo3::types::PyList>,
                    values: Bound<'_, pyo3::types::PyList>,
                ) -> PyResult<()> {
                    let ids = ids.extract::<Vec<u8>>()?;
                    let values = values.extract::<Vec<$reg_type>>()?;

                    py
                        .detach(|| {
                            let mut guard = self.0.lock().unwrap();
                            Self::borrow(&mut guard)?
                                .[<sync_write_ $reg_name>](&ids, &values)
                                .map_err(|e| e.to_string())
                        })
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)
                }

                #[doc = concat!("Write register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<write_ $reg_name>](
                    &self,
                    py: Python,
                    id: u8,
                    value: $reg_type,
                ) -> PyResult<()> {

                    py
                        .detach(|| {
                            let mut guard = self.0.lock().unwrap();
                            Self::borrow(&mut guard)?
                                .[<write_ $reg_name>](id, value)
                                .map_err(|e| e.to_string())
                        })
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)
                }

            }

        }

    };
    ($servo_name:ident, $reg_name:ident, $reg_addr:expr, $reg_type:ty, $conv:ident) => {
        paste::paste! {
            #[doc = concat!("Write raw register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: ", stringify!($reg_type), ")")]
            pub fn [<write_raw_ $reg_name>](
                io: &$crate::DynamixelProtocolHandler,
                serial_port: &mut dyn serialport::SerialPort,
                id: u8,
                val: $reg_type,
            ) -> $crate::Result<()> {
                io.write(serial_port, id, $reg_addr, &val.to_le_bytes())
            }

            #[doc = concat!("Write register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", converted by ", stringify!($conv), ")")]
            pub fn [<write_ $reg_name>](
                io: &$crate::DynamixelProtocolHandler,
                serial_port: &mut dyn serialport::SerialPort,
                id: u8,
                val: <$conv as Conversion>::UsiType,
            ) -> $crate::Result<()> {
                let val = $conv::to_raw(val);
                [<write_raw_ $reg_name>](io, serial_port, id, val)
            }

            #[doc = concat!("Sync write raw register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: ", stringify!($reg_type), ")")]
            pub fn [<sync_write_raw_ $reg_name>](
                io: &$crate::DynamixelProtocolHandler,
                serial_port: &mut dyn serialport::SerialPort,
                ids: &[u8],
                values: &[$reg_type],
            ) -> $crate::Result<()> {
                io.sync_write(
                    serial_port,
                    ids,
                    $reg_addr,
                    &values
                        .iter()
                        .map(|v| v.to_le_bytes().to_vec())
                        .collect::<Vec<Vec<u8>>>(),
                )
            }

            #[doc = concat!("Sync write register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", converted by ", stringify!($conv), ")")]
            pub fn [<sync_write_ $reg_name>](
                io: &$crate::DynamixelProtocolHandler,
                serial_port: &mut dyn serialport::SerialPort,
                ids: &[u8],
                values: &[<$conv as Conversion>::UsiType],
            ) -> $crate::Result<()> {
                let values = values
                    .iter()
                    .map(|&v| $conv::to_raw(v))
                    .collect::<Vec<_>>();
                [<sync_write_raw_ $reg_name>](io, serial_port, ids, &values)
            }

            impl [<$servo_name:camel Controller>] {
                #[doc = concat!("Sync write raw register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<sync_write_raw_ $reg_name>](
                    &mut self,
                    ids: &[u8],
                    values: &[$reg_type],
                ) -> $crate::Result<()> {
                    [<sync_write_raw_ $reg_name>](
                        self.dph.as_ref().unwrap(),
                        self.serial_port.as_mut().unwrap().as_mut(),
                        ids,
                        values,
                    )
                }

                #[doc = concat!("Sync write register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", converted by ", stringify!($conv), ")")]
                pub fn [<sync_write_ $reg_name>](
                    &mut self,
                    ids: &[u8],
                    values: &[<$conv as Conversion>::UsiType],
                ) -> $crate::Result<()> {
                    [<sync_write_ $reg_name>](
                        self.dph.as_ref().unwrap(),
                        self.serial_port.as_mut().unwrap().as_mut(),
                        ids,
                        values,
                    )
                }

                #[doc = concat!("Write raw register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<write_raw_ $reg_name>](
                    &mut self,
                    id: u8,
                    value: $reg_type,
                ) -> $crate::Result<()> {
                    [<write_raw_ $reg_name>](
                        self.dph.as_ref().unwrap(),
                        self.serial_port.as_mut().unwrap().as_mut(),
                        id,
                        value,
                    )
                }

                #[doc = concat!("Write register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", converted by ", stringify!($conv), ")")]
                pub fn [<write_ $reg_name>](
                    &mut self,
                    id: u8,
                    value: <$conv as Conversion>::UsiType,
                ) -> $crate::Result<()> {
                    [<write_ $reg_name>](
                        self.dph.as_ref().unwrap(),
                        self.serial_port.as_mut().unwrap().as_mut(),
                        id,
                        value,
                    )
                }

            }

            #[cfg(feature = "python")]
            #[gen_stub_pymethods]
            #[pymethods]
            impl [<$servo_name:camel PyController>] {
                #[doc = concat!("Sync write raw register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<sync_write_raw_ $reg_name>](
                    &self,
                    py: Python,
                    ids: Bound<'_, pyo3::types::PyList>,
                    values: Bound<'_, pyo3::types::PyList>,
                ) -> PyResult<()> {
                    let ids = ids.extract::<Vec<u8>>()?;
                    let values = values.extract::<Vec<$reg_type>>()?;

                    py
                        .detach(|| {
                            let mut guard = self.0.lock().unwrap();
                            Self::borrow(&mut guard)?
                                .[<sync_write_raw_ $reg_name>](&ids, &values)
                                .map_err(|e| e.to_string())
                        })
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)
                }

                #[doc = concat!("Sync write register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", converted by ", stringify!($conv), ")")]
                pub fn [<sync_write_ $reg_name>](
                    &self,
                    py: Python,
                    ids: &Bound<'_, pyo3::types::PyList>,
                    values: &Bound<'_, pyo3::types::PyList>,
                ) -> PyResult<()> {
                    let ids = ids.extract::<Vec<u8>>()?;
                    let values = values.extract::<Vec<<$conv as Conversion>::UsiType>>()?;

                    py
                        .detach(|| {
                            let mut guard = self.0.lock().unwrap();
                            Self::borrow(&mut guard)?
                                .[<sync_write_ $reg_name>](&ids, &values)
                                .map_err(|e| e.to_string())
                        })
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)
                }


                #[doc = concat!("Write raw register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<write_raw_ $reg_name>](
                    &self,
                    py: Python,
                    id: u8,
                    value: $reg_type,
                ) -> PyResult<()> {

                    py
                        .detach(|| {
                            let mut guard = self.0.lock().unwrap();
                            Self::borrow(&mut guard)?
                                .[<write_raw_ $reg_name>](id, value)
                                .map_err(|e| e.to_string())
                        })
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)
                }

                #[doc = concat!("Write register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", converted by ", stringify!($conv), ")")]
                pub fn [<write_ $reg_name>](
                    &self,
                    py: Python,
                    id: u8,
                    value: <$conv as Conversion>::UsiType,
                ) -> PyResult<()> {

                    py
                        .detach(|| {
                            let mut guard = self.0.lock().unwrap();
                            Self::borrow(&mut guard)?
                                .[<write_ $reg_name>](id, value)
                                .map_err(|e| e.to_string())
                        })
                        .map_err(pyo3::exceptions::PyRuntimeError::new_err)
                }

            }

        }

    };
}

/// Generates write and sync_write functions with feedback for given register
#[macro_export]
macro_rules! generate_reg_write_fb {
    ($name:ident, $addr:expr, $reg_type:ty, $fb_type: ty) => {
        paste::paste! {
            #[doc = concat!("Write register with fb *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
            pub fn [<write_ $name _fb>](
                dph: &$crate::DynamixelProtocolHandler,
                serial_port: &mut dyn serialport::SerialPort,
                id: u8,
                val: $reg_type,
            ) -> $crate::Result<$fb_type> {
                let fb = dph.write_fb(serial_port, id, $addr, &val.to_le_bytes())?;
                let fb = $fb_type::from_le_bytes(fb.try_into().unwrap());
                Ok(fb)
            }

            #[doc = concat!("Sync write register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
            pub fn [<sync_write_ $name _fb>](
                dph: &$crate::DynamixelProtocolHandler,
                serial_port: &mut dyn serialport::SerialPort,
                ids: &[u8],
                values: &[$reg_type],
            ) -> $crate::Result<()> {
                dph.sync_write(
                    serial_port,
                    ids,
                    $addr,
                    &values
                        .iter()
                        .map(|v| v.to_le_bytes().to_vec())
                        .collect::<Vec<Vec<u8>>>(),
                )
            }
        }
    };
}

#[macro_export]
macro_rules! register_servo {
    ($(servo: ($group:ident, $servo:ident,
               $(($name:ident, $model_number:expr)),+)
    ),+) => {
        paste::paste! {
            #[derive(Debug, Clone, Copy)]
            pub enum ServoKind {
                $(
                    $(
                        #[allow(non_camel_case_types)]
                        [<$group _ $name>],
                    )+
                )+
            }
            impl ServoKind {
                pub fn try_from(model_number: u16) -> Result<Self, String> {
                    match model_number {
                        $(
                            $(
                                $model_number => Ok(Self::[<$group _ $name>]),
                            )+
                        )+
                            _ => Err(format!("Unknown model number: {}", model_number)),
                    }
                }
            }

            $(
                impl $group::[<$servo:lower>]::[<$servo:camel Controller>] {
                    /// Model numbers of the servos this definition covers, by name.
                    pub const MODELS: &'static [(&'static str, u16)] = &[
                        $((stringify!($name), $model_number)),+
                    ];
                }
            )+

            #[cfg(feature = "python")]
            use pyo3::prelude::*;
            #[cfg(feature = "python")]
            use pyo3_stub_gen::derive::*;

            $(
                #[cfg(feature = "python")]
                #[gen_stub_pymethods]
                #[pymethods]
                impl $group::[<$servo:lower>]::[<$servo:camel PyController>] {
                    /// Model numbers of the servos this definition covers, as {name: number}.
                    #[staticmethod]
                    pub fn models() -> std::collections::HashMap<String, u16> {
                        $group::[<$servo:lower>]::[<$servo:camel Controller>]::MODELS
                            .iter()
                            .map(|&(name, number)| (name.to_string(), number))
                            .collect()
                    }
                }
            )+

            #[cfg(feature = "python")]
            pub(crate) fn register_class(m: &Bound<'_, PyModule>) -> PyResult<()> {
                $(
                    m.add_class::<$group::[<$servo:lower>]::[<$servo:camel PyController>]>()?;
                )+

                Ok(())
            }
        }
    };
}
