## Adding support for a new servo

> ⚠️ **Warning:** This documentation is only intended for servos that communicate via the Dynamixel Protocol (v1 or v2), such as the Robotis Dynamixel or Feetech servos.

* Create a new file in the service folder (or a subfolder such as feetech or dynamixel), for instance [sts3215.rs](./servo/feetech/sts3215.rs) in the feetech folder. Make sure to add its declaration in the parent module (for instance in [./servo/feetech/mod.rs]) as they must be explicitly declared in Rust. Something like this: 
```rust
pub mod sts3215.rs
```

* Add the servo definition in the new file. You can use the [XL430](./servo/dynamixel/xl430.rs) as a template. The macro should defined the `name` of the servo, the `protocol version` used and then a list of all registers with their name, address, access and type. 

* Finally, add the servo registration in the servo root module [./servo/mod.rs]. You can specify all variants supported by your servo definition. This registration allows for the scan function to detect your new kind of servo.