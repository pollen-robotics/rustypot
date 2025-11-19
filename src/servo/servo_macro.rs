#[macro_export]
macro_rules! generate_servo {
    ($servo_name:ident, $protocol:ident,
     $(reg: ($reg_name:ident, $reg_access:ident, $reg_addr:expr, $reg_type:ty, $conv:ident),)+
    ) => {
        paste::paste! {
            pub struct [<$servo_name:camel Controller>] {
                dph: Option<$crate::DynamixelProtocolHandler>,
                serial_port: Option<Box<dyn serialport::SerialPort>>,
            }

            impl Default for [<$servo_name:camel Controller>] {
                fn default() -> Self {
                    Self::new()
                }
            }

            impl [<$servo_name:camel Controller>] {
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
            #[gen_stub_pyclass]
            #[pyo3::pyclass(frozen)]
            pub struct [<$servo_name:camel PyController>](std::sync::Mutex<[<$servo_name:camel Controller>]>);
        }

        #[cfg(feature = "python")]
        use pyo3::prelude::*;
        #[cfg(feature = "python")]
        use pyo3_stub_gen::derive::*;

        $crate::generate_protocol_constructor!($servo_name, $protocol);
        $crate::generate_special_instructions!($servo_name);
        $crate::generate_addr_read_write!($servo_name);

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

                    Ok(Self(std::sync::Mutex::new(c)))
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

                    Ok(Self(std::sync::Mutex::new(c)))
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
                ) -> PyResult<PyObject> {


                    let x = self.0.lock().unwrap().read_raw_data(id, addr, length)
                        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
                    let l = pyo3::types::PyList::new(py, x.clone())?;

                    Ok(l.into())
                }

                pub fn write_raw_data(
                    &self,
                    id: u8,
                    addr: u8,
                    data: &Bound<'_, pyo3::types::PyList>,
                ) -> PyResult<()> {
                    let data = data.extract::<Vec<u8>>()?;

                    self.0.lock().unwrap().write_raw_data(id, addr, data)
                        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
                    Ok(())
                }

                pub fn sync_read_raw_data(
                    &self,
                    py: Python,
                    ids: &Bound<'_, pyo3::types::PyList>,
                    addr: u8,
                    length: u8,
                ) -> PyResult<PyObject> {
                    let ids = ids.extract::<Vec<u8>>()?;

                    let x = self.0.lock().unwrap().sync_read_raw_data(&ids, addr, length)
                        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
                    let l = pyo3::types::PyList::new(py, x.clone())?;

                    Ok(l.into())
                }

                pub fn sync_write_raw_data(
                    &self,
                    ids: &Bound<'_, pyo3::types::PyList>,
                    addr: u8,
                    data: &Bound<'_, pyo3::types::PyList>,
                ) -> PyResult<()> {
                    let ids = ids.extract::<Vec<u8>>()?;
                    let data = data.extract::<Vec<Vec<u8>>>()?;

                    self.0.lock().unwrap().sync_write_raw_data(&ids, addr, &data)
                        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
                    Ok(())
                }

                pub fn ping(&self, id: u8) -> PyResult<bool> {
                    self.0.lock().unwrap().ping(id)
                        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
                }

                pub fn reboot(&self, id: u8) -> PyResult<bool> {
                    self.0.lock().unwrap().reboot(id)
                        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
                }

                #[pyo3(signature = (
                    id,
                    conserve_id_only = true,
                    conserve_id_and_baudrate = true
                ))]
                pub fn factory_reset(
                    &self,
                    id: u8,
                    conserve_id_only: bool,
                    conserve_id_and_baudrate: bool,
                ) -> PyResult<()> {
                    self.0.lock().unwrap().factory_reset(id, conserve_id_only, conserve_id_and_baudrate)
                        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
                }
            }
        }
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

            impl [<$servo_name:camel Controller>] {
                #[doc = concat!("Sync read register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
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
                #[doc = concat!("Read register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
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
                pub fn [<sync_read_ $reg_name>](
                    &self,
                    py: Python,
                    ids: &Bound<'_, pyo3::types::PyList>,
                ) -> PyResult<PyObject> {
                    let ids = ids.extract::<Vec<u8>>()?;

                    let x = self.0.lock().unwrap().[<sync_read_ $reg_name>](&ids)
                        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
                    let l = pyo3::types::PyList::new(py, x.clone())?;

                    Ok(l.into())
                }
            }


            #[cfg(feature = "python")]
            #[gen_stub_pymethods]
            #[pymethods]
            impl [<$servo_name:camel PyController>] {
                pub fn [<read_ $reg_name>](
                    &self,
                    py: Python,
                    id: u8,
                ) -> PyResult<PyObject> {

                    let x = self.0.lock().unwrap().[<read_ $reg_name>](id)
                        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
                    let l = pyo3::types::PyList::new(py, x.clone())?;

                    Ok(l.into())
                }
            }


        }
    };
    ($servo_name:ident, $reg_name:ident, $reg_addr:expr, $reg_type:ty, $conv:ident) => {
        paste::paste! {
            #[doc = concat!("Read register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
            pub fn [<read_raw_ $reg_name>](
                io: &$crate::DynamixelProtocolHandler,
                serial_port: &mut dyn serialport::SerialPort,
                id: u8,
            ) -> $crate::Result<$reg_type> {
                let val = io.read(serial_port, id, $reg_addr, size_of::<$reg_type>().try_into().unwrap())?;
                let val = $reg_type::from_le_bytes(val.try_into().unwrap());

                Ok(val)
            }

            pub fn [<read_ $reg_name>](
                io: &$crate::DynamixelProtocolHandler,
                serial_port: &mut dyn serialport::SerialPort,
                id: u8,
            ) -> $crate::Result<<$conv as Conversion>::UsiType> {
                let val = [<read_raw_ $reg_name>](io, serial_port, id)?;
                let val = $conv::from_raw(val);
                Ok(val)
            }

            #[doc = concat!("Sync read register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
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
                #[doc = concat!("Sync read raw register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!(<$conv as Conversion>::UsiType), ")")]
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

                #[doc = concat!("Sync read register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
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

                #[doc = concat!("Read raw register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!(<$conv as Conversion>::UsiType), ")")]
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

                #[doc = concat!("Read register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
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
                #[doc = concat!("Sync read raw register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<sync_read_raw_ $reg_name>](
                    &self,
                    py: Python,
                    ids: &Bound<'_, pyo3::types::PyList>,
                ) -> PyResult<PyObject> {
                    let ids = ids.extract::<Vec<u8>>()?;

                    let x = self.0.lock().unwrap().[<sync_read_raw_ $reg_name>](&ids)
                        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
                    let l = pyo3::types::PyList::new(py, x.clone())?;
                    Ok(l.into())
                }

                #[doc = concat!("Sync read register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<sync_read_ $reg_name>](
                    &self,
                    py: Python,
                    ids: Bound<'_, pyo3::types::PyList>,
                ) -> PyResult<PyObject> {
                    let ids = ids.extract::<Vec<u8>>()?;

                    let x = self.0.lock().unwrap().[<sync_read_ $reg_name>](&ids)
                        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
                    let l = pyo3::types::PyList::new(py, x.clone())?;
                    Ok(l.into())
                }

                #[doc = concat!("Read raw register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<read_raw_ $reg_name>](
                    &self,
                    py: Python,
                    id: u8,
                ) -> PyResult<PyObject> {


                    let x = self.0.lock().unwrap().[<read_raw_ $reg_name>](id)
                        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
                    let l = pyo3::types::PyList::new(py, x.clone())?;
                    Ok(l.into())
                }

                #[doc = concat!("Read register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<read_ $reg_name>](
                    &self,
                    py: Python,
                    id: u8,
                ) -> PyResult<PyObject> {


                    let x = self.0.lock().unwrap().[<read_ $reg_name>](id)
                        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
                    let l = pyo3::types::PyList::new(py, x.clone())?;
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

            impl [<$servo_name:camel Controller>] {
                #[doc = concat!("Sync write register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
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


                #[doc = concat!("Write register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
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
                #[doc = concat!("Sync write register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<sync_write_ $reg_name>](
                    &self,
                    ids: Bound<'_, pyo3::types::PyList>,
                    values: Bound<'_, pyo3::types::PyList>,
                ) -> PyResult<()> {
                    let ids = ids.extract::<Vec<u8>>()?;
                    let values = values.extract::<Vec<$reg_type>>()?;

                    self.0.lock().unwrap().[<sync_write_ $reg_name>](&ids, &values).map_err(|e| {
                        pyo3::exceptions::PyRuntimeError::new_err(e.to_string())
                    })
                }

                #[doc = concat!("Write register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<write_ $reg_name>](
                    &self,
                    id: u8,
                    value: $reg_type,
                ) -> PyResult<()> {

                    self.0.lock().unwrap().[<write_ $reg_name>](id, value).map_err(|e| {
                        pyo3::exceptions::PyRuntimeError::new_err(e.to_string())
                    })
                }

            }

        }

    };
    ($servo_name:ident, $reg_name:ident, $reg_addr:expr, $reg_type:ty, $conv:ident) => {
        paste::paste! {
            #[doc = concat!("Write register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
            pub fn [<write_raw_ $reg_name>](
                io: &$crate::DynamixelProtocolHandler,
                serial_port: &mut dyn serialport::SerialPort,
                id: u8,
                val: $reg_type,
            ) -> $crate::Result<()> {
                io.write(serial_port, id, $reg_addr, &val.to_le_bytes())
            }

            pub fn [<write_ $reg_name>](
                io: &$crate::DynamixelProtocolHandler,
                serial_port: &mut dyn serialport::SerialPort,
                id: u8,
                val: <$conv as Conversion>::UsiType,
            ) -> $crate::Result<()> {
                let val = $conv::to_raw(val);
                [<write_raw_ $reg_name>](io, serial_port, id, val)
            }

            #[doc = concat!("Sync write register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
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
                #[doc = concat!("Sync write raw register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
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

                #[doc = concat!("Sync write register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!(<$conv as Conversion>::UsiType), ")")]
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

                #[doc = concat!("Write raw register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
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

                #[doc = concat!("Write register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!(<$conv as Conversion>::UsiType), ")")]
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
                #[doc = concat!("Sync write raw register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<sync_write_raw_ $reg_name>](
                    &self,
                    ids: Bound<'_, pyo3::types::PyList>,
                    values: Bound<'_, pyo3::types::PyList>,
                ) -> PyResult<()> {
                    let ids = ids.extract::<Vec<u8>>()?;
                    let values = values.extract::<Vec<$reg_type>>()?;

                    self.0.lock().unwrap().[<sync_write_raw_ $reg_name>](&ids, &values).map_err(|e| {
                        pyo3::exceptions::PyRuntimeError::new_err(e.to_string())
                    })
                }

                #[doc = concat!("Sync write register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<sync_write_ $reg_name>](
                    &self,
                    ids: &Bound<'_, pyo3::types::PyList>,
                    values: &Bound<'_, pyo3::types::PyList>,
                ) -> PyResult<()> {
                    let ids = ids.extract::<Vec<u8>>()?;
                    let values = values.extract::<Vec<<$conv as Conversion>::UsiType>>()?;

                    self.0.lock().unwrap().[<sync_write_ $reg_name>](&ids, &values).map_err(|e| {
                        pyo3::exceptions::PyRuntimeError::new_err(e.to_string())
                    })
                }


                #[doc = concat!("Write raw register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<write_raw_ $reg_name>](
                    &self,
                    id: u8,
                    value: $reg_type,
                ) -> PyResult<()> {

                    self.0.lock().unwrap().[<write_raw_ $reg_name>](id, value).map_err(|e| {
                        pyo3::exceptions::PyRuntimeError::new_err(e.to_string())
                    })
                }

                #[doc = concat!("Write register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
                pub fn [<write_ $reg_name>](
                    &self,
                    id: u8,
                    value: <$conv as Conversion>::UsiType,
                ) -> PyResult<()> {

                    self.0.lock().unwrap().[<write_ $reg_name>](id, value).map_err(|e| {
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

            #[cfg(feature = "python")]
            use pyo3::prelude::*;

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
