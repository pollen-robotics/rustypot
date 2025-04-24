## Adding support for a new servo

> ⚠️ **Warning:** This documentation is only intended for servos that communicate via the Dynamixel Protocol (v1 or v2), such as the Robotis Dynamixel or Feetech servos.

* Create a new file in the service folder (or a subfolder such as feetech or dynamixel), for instance [sts3215.rs](./feetech/sts3215.rs) in the feetech folder. Make sure to add its declaration in the parent module (for instance in [./feetech/mod.rs]) as they must be explicitly declared in Rust. Something like this: 
```rust
pub mod sts3215.rs
```

* Add the servo definition in the new file. You can use the [MX](./servo/dynamixel/mx.rs) as a template. The macro should defined the `name` of the servo, the `protocol version` used and then a list of all registers with their name, address, access, type and conversion type (can be set to None to get the raw register value). 

* Finally, add the servo registration in the servo root module [./mod.rs]. You can specify all variants supported by your servo definition. This registration allows for the scan function to detect your new kind of servo.

By doing this, you will be able to use the servo in the same way as the other servos. The servo will be automatically detected and registered when you run the scan function. You can then use it in your application. 

It will be available in the rust lib but also the python bindings.

### Conversion types

If you want to define custom conversion function for a register (such as transforming the raw encode position to radians for instance), you need to define a struct that implements the `Conversion` trait. 

See the [AnglePosition](./dynamixel/mx.rs) for an example. You can see that the `position` register uses the `AnglePosition` conversion type.