# Rustypot: a Rust package to communicate with Dynamixel motors

[![Build Status]][actions] [![Latest Version]][crates.io]

[Build Status]: https://img.shields.io/github/actions/workflow/status/pollen-robotics/rustypot/rust.yml?branch=master
[actions]: https://github.com/pollen-robotics/rustypot/actions?query=branch%3Amaster

[Latest Version]: https://img.shields.io/crates/v/rustypot.svg
[crates.io]: https://crates.io/crates/rustypot

## Getting started

Rustypot is yet another communication library for robotis Dynamixel motors. It is currently used in the [Reachy project](https://www.pollen-robotics.com/reachy/).

## Feature Overview

* Relies on [serialport](https://docs.rs/serialport/latest/serialport/) for serial communication
* Support for dynamixel protocol v1 and v2 (can also use both on the same bus)
* Support for sync read and sync write operations
* Easy support for new type of motors (register definition through macros). Currently support for dynamixel XL320, XL330, XL430, XM430, MX*, Orbita 2D & 3D.
* Pure Rust

### Examples
```rust
use rustypot::{device::mx, DynamixelSerialIO};
use std::time::Duration;

fn main() {
    let mut serial_port = serialport::new("/dev/ttyACM0", 1_000_000)
        .timeout(Duration::from_millis(10))
        .open()
        .expect("Failed to open port");

    let io = DynamixelSerialIO::v1();

    loop {
        let pos =
            mx::read_present_position(&io, serial_port.as_mut(), 11).expect("Communication error");
        println!("Motor 11 present position: {:?}", pos);
    }
}
```

## Documentation

See https://docs.rs/rustypot for more information on APIs and examples.

## License

This library is licensed under the [Apache License 2.0](./LICENSE).

## Support

Rustypot is developed and maintained by [Pollen-Robotics](https://pollen-robotics.com). They developped open-source tools for robotics manipulation.
Visit https://pollen-robotics.com to learn more or join the [Discord community](https://discord.com/invite/Kg3mZHTKgs) if you have any questions or want to share your projects. 
