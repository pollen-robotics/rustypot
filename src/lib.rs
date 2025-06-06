//! A low-level communication library for servo (Dynamixel and Feetech motors).
//!
//! ## Feature Overview
//!
//! * Relies on [serialport] for serial communication
//! * Support for dynamixel protocol v1 and v2 (both can be used on the same io)
//! * Support for sync read and sync write operations
//! * Easy support for new type of motors (register definition through macros)
//! * Pure Rust
//!
//! ## APIs
//!
//! It exposes two APIs:
//! * `DynamixelProtocolHandler`: low-level API. It handles the serial communication and the Dynamixel protocol parsing. It can be used for fine-grained control of the shared bus with other communication.
//! * `Controller`: high-level API for the Dynamixel protocol. Simpler and cleaner API but it takes full ownership of the io (it can still be shared if wrapped with a mutex for instance).
//!
//! See the examples below for usage.
//!
//! ## Examples
//!
//! ### With the low-level API
//! ```no_run
//! use rustypot::{DynamixelProtocolHandler, servo::dynamixel::mx};
//! use std::time::Duration;
//!
//! let mut serial_port = serialport::new("/dev/ttyACM0", 1_000_000)
//!     .timeout(Duration::from_millis(10))
//!     .open()
//!     .expect("Failed to open port");
//!
//! let dph = DynamixelProtocolHandler::v1();
//!
//! let raw_pos: i16 = mx::read_raw_present_position(&dph, serial_port.as_mut(), 11).expect("Communication error");
//! let pos: f64 =
//!     mx::read_present_position(&dph, serial_port.as_mut(), 11).expect("Communication error");
//! println!("Motor 11 present position: {:?}rads (raw: {:?})", pos, raw_pos);
//! ```
//!
//! ### With the high-level API
//! ```no_run
//! use rustypot::servo::feetech::sts3215::Sts3215Controller;
//! use std::time::Duration;
//!
//! let serial_port = serialport::new("/dev/ttyUSB0", 1_000_000)
//!     .timeout(Duration::from_millis(1000))
//!     .open()
//!     .unwrap();
//!
//! let mut c = Sts3215Controller::new()
//!         .with_protocol_v1()
//!         .with_serial_port(serial_port);
//!
//! let pos = c.sync_read_present_position(&vec![1, 2]).unwrap();
//! println!("Motors present position: {:?}", pos);
//!
//! c.sync_write_goal_position(&vec![1, 2], &vec![0.0, 90.0_f64.to_radians()]).unwrap();
//! ```

pub mod servo;

mod dynamixel_protocol;
pub use dynamixel_protocol::{CommunicationErrorKind, DynamixelProtocolHandler};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

#[cfg(feature = "python")]
use pyo3::prelude::*;
#[cfg(feature = "python")]
use pyo3_stub_gen::define_stub_info_gatherer;

#[cfg(feature = "python")]
#[pymodule]
fn rustypot(m: &Bound<'_, PyModule>) -> PyResult<()> {
    pyo3_log::init();

    servo::register_class(m)?;

    Ok(())
}

#[cfg(feature = "python")]
define_stub_info_gatherer!(stub_info);
