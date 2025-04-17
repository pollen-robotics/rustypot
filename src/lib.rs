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
//! let pos =
//!     mx::read_present_position(&dph, serial_port.as_mut(), 11).expect("Communication error");
//! println!("Motor 11 present position: {:?}", pos);
//! ```
//!
//! ### With the high-level API
//! ```no_run
//! use rustypot::servo::feetech::sts3215::STS3215Controller;
//! use std::time::Duration;
//!
//! let serial_port = serialport::new("/dev/ttyUSB0", 1_000_000)
//!     .timeout(Duration::from_millis(1000))
//!     .open()
//!     .unwrap();
//!
//! let mut c = STS3215Controller::new()
//!         .with_protocol_v1()
//!         .with_serial_port(serial_port);
//!
//! let pos = c.read_present_position(&vec![1, 2]).unwrap();
//! println!("Motors present position: {:?}", pos);
//!
//! c.write_goal_position(&vec![1, 2], &vec![1000, 2000]).unwrap();
//! ```

pub mod servo;

mod dynamixel_protocol;
pub use dynamixel_protocol::{CommunicationErrorKind, DynamixelProtocolHandler};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

// TODO: clippy
