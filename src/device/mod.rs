//! High-level register access functions for a specific dynamixel device

use num_enum::IntoPrimitive;
use num_enum::TryFromPrimitive;

use paste::paste;
use std::mem::size_of;

use crate::{reg_read_only, reg_read_write, DynamixelSerialIO, Result};

#[derive(Debug, IntoPrimitive, TryFromPrimitive)]
#[repr(u16)]
pub enum DxlModel {
    AX12A = 12,
    AX12W = 300,
    AX18A = 18,
    RX10 = 10,
    RX24F = 24,
    RX28 = 28,
    RX64 = 64,
    EX106 = 107,
    MX12W = 360,
    MX28 = 29,
    MX282 = 30,
    MX64 = 310,
    MX642 = 311,
    MX106 = 320,
    MX1062 = 321,
    XL320 = 350,
    XL330M077 = 1190,
    XL330M288 = 1200,
    XC330M181 = 1230,
    XC330M288 = 1240,
    XC330T181 = 1210,
    XC330T288 = 1220,
    XL430W250 = 1060,
    XL430W2502 = 1090,
    XC430W2502 = 1160,
    XC430W150 = 1070,
    XC430W240 = 1080,
    XM430W210 = 1030,
    XM430W350 = 1020,
    XM540W150 = 1130,
    XM540W270 = 1120,
    XH430W210 = 1010,
    XH430W350 = 1000,
    XH430V210 = 1050,
    XH430V350 = 1040,
    XH540W150 = 1110,
    XH540W270 = 1100,
    XH540V150 = 1150,
    XH540V270 = 1140,
    XW540T260 = 1170,
    XW540T140 = 1180,
}

/// Generates read and sync_read functions for given register
#[macro_export]
macro_rules! reg_read_only {
    ($name:ident, $addr:expr, $reg_type:ty) => {

    paste! {
        #[doc = concat!("Read register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
        pub fn [<read_ $name>](
            io: &DynamixelSerialIO,
            serial_port: &mut dyn serialport::SerialPort,
            id: u8,
        ) -> Result<$reg_type> {
            let val = io.read(serial_port, id, $addr, size_of::<$reg_type>().try_into().unwrap())?;
            check_response_size!(&val, $reg_type);
            let val = $reg_type::from_le_bytes(val.try_into().unwrap());

            Ok(val)
        }

        #[doc = concat!("Sync read register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
        pub fn [<sync_read_ $name>](
            io: &DynamixelSerialIO,
            serial_port: &mut dyn serialport::SerialPort,
            ids: &[u8],
        ) -> Result<Vec<$reg_type>> {
            let val = io.sync_read(serial_port, ids, $addr, size_of::<$reg_type>().try_into().unwrap())?;
            check_response_size!(&val[0], $reg_type);
            let val = val
                .iter()
                .map(|v| $reg_type::from_le_bytes(v.as_slice().try_into().unwrap()))
                .collect();

            Ok(val)
        }
    }

    }
}

/// Generates write and sync_write functions for given register
#[macro_export]
macro_rules! reg_write_only {
    ($name:ident, $addr:expr, $reg_type:ty) => {
        paste! {
            #[doc = concat!("Write register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
            pub fn [<write_ $name>](
                io: &DynamixelSerialIO,
                serial_port: &mut dyn serialport::SerialPort,
                id: u8,
                val: $reg_type,
            ) -> Result<()> {
                io.write(serial_port, id, $addr, &val.to_le_bytes())
            }

            #[doc = concat!("Sync write register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
            pub fn [<sync_write_ $name>](
                io: &DynamixelSerialIO,
                serial_port: &mut dyn serialport::SerialPort,
                ids: &[u8],
                values: &[$reg_type],
            ) -> Result<()> {
                io.sync_write(
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

/// Generates write and sync_write functions with feedback for given register
#[macro_export]
macro_rules! reg_write_only_fb {
    ($name:ident, $addr:expr, $reg_type:ty, $fb_type: ty) => {
        paste! {
            #[doc = concat!("Write register with fb *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
            pub fn [<write_ $name>](
                io: &DynamixelSerialIO,
                serial_port: &mut dyn serialport::SerialPort,
                id: u8,
                val: $reg_type,
            ) -> Result<$fb_type> {
                let fb=io.write_fb(serial_port, id, $addr, &val.to_le_bytes())?;
                check_response_size!(&fb, $fb_type);
                let fb = $fb_type::from_le_bytes(fb.try_into().unwrap());
                Ok(fb)
            }

            #[doc = concat!("Sync write register *", stringify!($name), "* (addr: ", stringify!($addr), ", type: ", stringify!($reg_type), ")")]
            pub fn [<sync_write_ $name>](
                io: &DynamixelSerialIO,
                serial_port: &mut dyn serialport::SerialPort,
                ids: &[u8],
                values: &[$reg_type],
            ) -> Result<()> {
                io.sync_write(
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

/// Generates read, sync_read, write and sync_write functions for given register
#[macro_export]
macro_rules! reg_read_write {
    ($name:ident, $addr:expr, $reg_type:ty) => {
        reg_read_only!($name, $addr, $reg_type);
        reg_write_only!($name, $addr, $reg_type);
    };
}

#[macro_export]
macro_rules! reg_read_write_fb {
    ($name:ident, $addr:expr, $reg_type:ty, $fb_type: ty) => {
        reg_read_only!($name, $addr, $reg_type);
        reg_write_only_fb!($name, $addr, $reg_type, $fb_type);
    };
}

// Check if the response size is correct
// If not, return an error
// response is a Vec<u8>
macro_rules! check_response_size {
    ($response:expr, $reg_type:ty) => {{
        let response = $response;
        if response.len() != std::mem::size_of::<$reg_type>() {
            let message = format!(
                "Invalid response size, expected {} received {}",
                std::mem::size_of::<$reg_type>(),
                response.len()
            );
            return Err(message.into());
        }
    }};
}

pub mod l0_force_fan;
pub mod mx;
pub mod orbita2d_poulpe;
pub mod orbita2dof_foc;
pub mod orbita_foc;

pub mod orbita3d_poulpe;
pub mod xl320;
pub mod xl330;
pub mod xl430;
pub mod xm;
