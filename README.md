# Rustypot: a Rust package to communicate with Dynamixel/Feetech motors

[![Build Status]][actions] [![Latest Version]][crates.io]

[Build Status]: https://img.shields.io/github/actions/workflow/status/pollen-robotics/rustypot/rust.yml?branch=master
[actions]: https://github.com/pollen-robotics/rustypot/actions?query=branch%3Amaster

[Latest Version]: https://img.shields.io/crates/v/rustypot.svg
[crates.io]: https://crates.io/crates/rustypot

## Getting started

Rustypot is a communication library for Dynamixel/Feetech motors. It is notably used in the [Reachy project](https://www.pollen-robotics.com/reachy/). More types of servo can be added in the future.

## Feature Overview

* Relies on [serialport](https://docs.rs/serialport/latest/serialport/) for serial communication
* Support for dynamixel protocol v1 and v2 (can also use both on the same bus)
* Support for sync read and sync write operations
* Easy support for new type of motors (register definition through macros). Currently support for dynamixel XL320, XL330, XL430, XM430, MX*, Orbita 2D & 3D.
* Pure Rust plus python bindings (using [pyo3](https://pyo3.rs/)).

To add new servo, please refer to the [Servo documentation](./servo/README.md).

## APIs

It exposes two APIs:
* `DynamixelProtocolHandler`: low-level API. It handles the serial communication and the Dynamixel protocol parsing. It can be used for fine-grained control of the shared bus with other communication.
* `Controller`: high-level API for the Dynamixel protocol. Simpler and cleaner API but it takes full ownership of the io (it can still be shared if wrapped with a mutex for instance).

See the examples below for usage.

### Examples
```rust
use rustypot::{DynamixelProtocolHandler, servo::dynamixel::mx};
use std::time::Duration;

fn main() {
    let mut serial_port = serialport::new("/dev/ttyACM0", 1_000_000)
        .timeout(Duration::from_millis(10))
        .open()
        .expect("Failed to open port");

    let dph = DynamixelProtocolHandler::v1();

    loop {
        let pos =
            mx::read_present_position(&dph, serial_port.as_mut(), 11).expect("Communication error");
        println!("Motor 11 present position: {:?}", pos);
    }
}
```

```rust
use rustypot::servo::feetech::sts3215::STS3215Controller;
use std::time::Duration;

fn main() {
    let serial_port = serialport::new("/dev/ttyUSB0", 1_000_000)
        .timeout(Duration::from_millis(1000))
        .open()
        .unwrap();

    let mut c = STS3215Controller::new()
            .with_protocol_v1()
            .with_serial_port(serial_port);

    let pos = c.read_present_position(&vec![1, 2]).unwrap();
    println!("Motors present position: {:?}", pos);

    c.write_goal_position(&vec![1, 2], &vec![1000, 2000]).unwrap();
}
```

## Documentation

See https://docs.rs/rustypot for more information on APIs and examples.

See [python/README.md](./python/README.md) for information on how to use the python bindings.

## Python bindings

The python bindings are generated using [pyo3](https://pyo3.rs/). They are available on `pypi`(https://pypi.org/project/rustypot/). You can install them using pip, pix, uv, etc.

To build them locally, you can use [maturin](https://www.maturin.rs).

```bash
maturin build --release --features python
```

or, if you want to install them in your local python environment: 

```bash
maturin develop --release --features python
```

See [maturin official documentation](https://maturin.rs) for more information on how to use it.

## Contributing

If you want to contribute to Rustypot, please fork the repository and create a pull request. We welcome any contributions, including bug fixes, new features, and documentation improvements.
We especially appreciate support for new servos. If you want to add support for a new servo, please follow the instructions in the [Servo documentation](./servo/README.md).

## License

This library is licensed under the [Apache License 2.0](./LICENSE).

## Support

Rustypot is developed and maintained by [Pollen-Robotics](https://pollen-robotics.com). They developed open-source hardware and tools for robotics.
Visit https://pollen-robotics.com to learn more or join the [Discord community](https://discord.com/invite/Kg3mZHTKgs) if you have any questions or want to share your projects. 
