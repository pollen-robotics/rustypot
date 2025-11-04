## Version 1.4.0

- Add support for factory reset in core library and python bindings.

## Version 1.3.0

- Add reboot support in core library.
- Add ping and reboot support in python bindings.

## Version 1.2.0

- Add support for AX motors (see https://github.com/pollen-robotics/rustypot/pull/93, thanks to @kacper-uminski)


## Version 1.1.0

- Add support for feetech Scs0009
- Add python type annotation 

## Version 1.0.0

- Cleanup APIs to offer two interfaces:
  - high-level interface (Controller) with a simple API for the most common use cases.
  - low-level interface (DynamixelProtocolHandler) for direct access to the protocol and fine-grained control of the bus ownership.
- Add Python bindings for the library (controller API).
- Add support for the feetech servo.
- Define register conversion at the macro level to simplify the code.

## Version 0.6.0

- Add dxl XL330 support

## Version 0.5.0

- Add an post delay option to the read and write method.
- Add dxl XM motor device

## Version 0.4.0

- Add support for orbita-2dof-foc device.

### Version 0.3.1

- Patch torque limit conversion.

## Version 0.3.0

- Add support for orbita-foc device.

## Version 0.2.0

- Add support for timeout in sync read v1

## Version 0.1.0

- Support protocol v1 and v2
- Support read, sync read, write, sync write, ping
- Support mx and xl-320
