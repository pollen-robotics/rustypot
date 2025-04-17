//! Yet another communication library for robotis Dynamixel motors.
//!
//! ## Feature Overview
//!
//! * Relies on [serialport] for serial communication
//! * Support for dynamixel protocol v1 and v2 (can also use both on the same io)
//! * Support for sync read and sync write operations
//! * Easy support for new type of motors (register definition through macros)
//! * Pure Rust
//!
//! *Note: this version use std and Vec extensively.*
//!
//! ## Examples
//! ```no_run
//! use rustypot::{DynamixelProtocolHandler, device::mx};
//! use std::time::Duration;
//!
//! let mut serial_port = serialport::new("/dev/ttyACM0", 1_000_000)
//!     .timeout(Duration::from_millis(10))
//!     .open()
//!     .expect("Failed to open port");
//!
//! let dph = DynamixelProtocolHandler::v1();
//!
//! let pos =
//!     mx::read_present_position(&dph, serial_port.as_mut(), 11).expect("Communication error");
//! println!("Motor 11 present position: {:?}", pos);
//! ```

pub mod servo;

mod dynamixel_protocol;
pub use dynamixel_protocol::{CommunicationErrorKind, DynamixelProtocolHandler};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
