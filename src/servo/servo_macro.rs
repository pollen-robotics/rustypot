#[macro_export]
macro_rules! generate_servo {
    ($servo_name:ident, $protocol:ident,
        $(reg: ($reg_name:ident, $reg_access:ident, $reg_addr:expr, $reg_type:ty),)+
    ) => {
        paste::paste! {
            pub struct [<$servo_name Controller>] {
                dph: Option<$crate::DynamixelProtocolHandler>,
                serial_port: Option<Box<dyn serialport::SerialPort>>,
            }

            impl Default for [<$servo_name Controller>] {
                fn default() -> Self {
                    Self::new()
                }
            }

            impl [<$servo_name Controller>] {
                pub fn new() -> Self {
                    Self {dph: None, serial_port: None}
                }
                pub fn with_serial_port(self,
                    serial_port: Box<dyn serialport::SerialPort>,
                ) -> Self {
                    Self {
                        serial_port: Some(serial_port),
                        ..self
                    }
                }
            }

            #[cfg(feature = "python")]
            #[pyo3::pyclass(frozen)]
            pub struct [<$servo_name SyncController>](std::sync::Mutex<[<$servo_name Controller>]>);
        }

        $crate::generate_protocol_constructor!($servo_name, $protocol);

        $(
            $crate::generate_reg_access!($servo_name, $reg_name, $reg_access, $reg_addr, $reg_type);
        )*
    };
}

#[macro_export]
macro_rules! generate_protocol_constructor {
    ($servo_name:ident, v1) => {
        paste::paste! {
            impl [<$servo_name Controller>] {
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
            #[pyo3::pymethods]
            impl [<$servo_name SyncController>] {
                #[new]
                pub fn new(serial_port: &str, baudrate: u32, timeout: f32) -> pyo3::PyResult<Self> {
                    let serial_port = serialport::new(serial_port, baudrate)
                        .timeout(std::time::Duration::from_secs_f32(timeout))
                        .open()
                        .map_err(|e| pyo3::exceptions::PyIOError::new_err(e.to_string()))?;

                    let c = [<$servo_name Controller>]::new()
                        .with_serial_port(serial_port)
                        .with_protocol_v1();

                    Ok(Self(std::sync::Mutex::new(c)))
                }
            }
        }
    };
    ($servo_name:ident, v2) => {
        paste::paste! {
            impl [<$servo_name Controller>] {
                pub fn with_protocol_v2(
                    self,
                ) -> Self {
                    Self {
                        dph: Some($crate::DynamixelProtocolHandler::v2()),
                        ..self
                    }
                }
            }
            #[cfg(feature = "python")]
            #[pyo3::pymethods]
            impl [<$servo_name SyncController>] {
                #[new]
                pub fn new(serial_port: &str, baudrate: u32, timeout: f32) -> pyo3::PyResult<Self> {
                    let serial_port = serialport::new(serial_port, baudrate)
                        .timeout(std::time::Duration::from_secs_f32(timeout))
                        .open()
                        .map_err(|e| pyo3::exceptions::PyIOError::new_err(e.to_string()))?;

                    let c = [<$servo_name Controller>]::new()
                        .with_serial_port(serial_port)
                        .with_protocol_v2();

                    Ok(Self(std::sync::Mutex::new(c)))
                }
            }
        }
    };
}

#[macro_export]
macro_rules! generate_reg_access {
    ($servo_name:ident, $reg_name:ident, r, $reg_addr:expr, $reg_type:ty) => {
        $crate::generate_reg_read!($servo_name, $reg_name, $reg_addr, $reg_type);
    };
    ($servo_name:ident, $reg_name:ident, w, $reg_addr:expr, $reg_type:ty) => {
        $crate::generate_reg_write!($servo_name, $reg_name, $reg_addr, $reg_type);
    };
    ($servo_name:ident, $reg_name:ident, rw, $reg_addr:expr, $reg_type:ty) => {
        $crate::generate_reg_read!($servo_name, $reg_name, $reg_addr, $reg_type);
        $crate::generate_reg_write!($servo_name, $reg_name, $reg_addr, $reg_type);
    };
}
#[macro_export]
macro_rules! generate_reg_read {
    ($servo_name:ident, $reg_name:ident, $reg_addr:expr, $reg_type:ty) => {
        paste::paste! {
            #[doc = concat!("Read register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
            pub fn [<read_ $reg_name>](
                io: &$crate::DynamixelProtocolHandler,
                serial_port: &mut dyn serialport::SerialPort,
                id: u8,
            ) -> $crate::Result<$reg_type> {
                let val = io.read(serial_port, id, $reg_addr, size_of::<$reg_type>().try_into().unwrap())?;
                let val = $reg_type::from_le_bytes(val.try_into().unwrap());

                Ok(val)
            }

            #[doc = concat!("Sync read register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
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

        impl [<$servo_name Controller>] {
            #[doc = concat!("Read register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
            pub fn [<read_ $reg_name>](
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

        #[cfg(feature = "python")]
        #[pyo3::pymethods]
        impl [<$servo_name SyncController>] {
            #[doc = concat!("Read register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
            pub fn [<read_ $reg_name>](
                &self,
                ids: Vec<u8>,
            ) -> pyo3::PyResult<Vec<$reg_type>> {
                self.0.lock().unwrap().[<read_ $reg_name>](&ids)
                    .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
            }
        }

        }
    };
}
#[macro_export]
macro_rules! generate_reg_write {
    ($servo_name:ident, $reg_name:ident, $reg_addr:expr, $reg_type:ty) => {
        paste::paste! {
            #[doc = concat!("Write register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
            pub fn [<write_ $reg_name>](
                io: &$crate::DynamixelProtocolHandler,
                serial_port: &mut dyn serialport::SerialPort,
                id: u8,
                val: $reg_type,
            ) -> $crate::Result<()> {
                io.write(serial_port, id, $reg_addr, &val.to_le_bytes())
            }

            #[doc = concat!("Sync write register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
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

        impl [<$servo_name Controller>] {
            #[doc = concat!("Write register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
            pub fn [<write_ $reg_name>](
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
        }

        #[cfg(feature = "python")]
        #[pyo3::pymethods]
        impl [<$servo_name SyncController>] {
            #[doc = concat!("Write register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
            pub fn [<write_ $reg_name>](
                &self,
                ids: &[u8],
                values: Vec<$reg_type>,
            ) -> pyo3::PyResult<()> {
                self.0.lock().unwrap().[<write_ $reg_name>](ids, &values).map_err(|e| {
                    pyo3::exceptions::PyRuntimeError::new_err(e.to_string())
                })
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
                        [<$group:camel $name:camel>],
                    )+
                )+
            }
            impl ServoKind {
                pub fn try_from(model_number: u16) -> Result<Self, String> {
                    match model_number {
                        $(
                            $(
                                $model_number => Ok(Self::[<$group:camel $name:camel>]),
                            )+
                        )+
                        _ => Err(format!("Unknown model number: {}", model_number)),
                    }
                }
            }

            #[cfg(feature = "python")]
            use pyo3::prelude::*;

            #[cfg(feature = "python")]
            pub(crate) fn register_module(parent_module: &Bound<'_, PyModule>) -> PyResult<()> {
                let child_module = PyModule::new(parent_module.py(), "servo")?;

                $(
                    child_module.add_class::<$group::[<$servo:lower>]::[<$servo SyncController>]>()?;
                )+

                parent_module.add_submodule(&child_module)
            }
        }
    };
}
