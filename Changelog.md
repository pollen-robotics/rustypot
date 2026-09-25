## Unreleased

- Register the model numbers of the X-series servos that share an existing definition:
  XC330-T181 and XC330-T288 under XL330; XC430-W150, XM430-W350, XM540-W270 and
  XH540-W150 under XL430, whose definition is the XM430 control table. On the Feetech
  side, STS3250 and SM8512BL join STS3215, and the STS3215 model number is corrected to
  777: bytes 9 and 3 at address 3 read little-endian, as the scan tool reads them; 2307
  was the same bytes swapped and never matched.
- STS3215 and SCS0009: add the firmware version bytes and the factory block (moving
  velocity threshold, DTs, velocity unit factor, Hts and maximum velocity limit on the
  STS3215; acceleration, sync write flag, PWM maximum step, and the velocity threshold and
  limits on the SCS0009). Registers that exist on Dynamixel servos under another name get
  that name as a second name: `model_number`, `baud_rate`, `min_position_limit`,
  `max_position_limit`, `homing_offset`, `operating_mode`, `goal_velocity`,
  `present_velocity`. Same address, size and access; the original names stay.

- Servo definitions can state what a raw-address caller needs to know beyond the
  registers, with `resolution:`, `word_order:`, `supports_sync_read:`, `baudrates:` and
  `encoding:` entries ahead of the `reg:` list, all optional. They fill a `ServoInfo`
  (`INFO` on each servo module) and an `encoding` on every `RegisterInfo`: unsigned,
  two's complement, or sign-magnitude with its sign bit, taken from the register's
  integer type unless the definition says otherwise. Each servo module also gets
  `MODELS`, the (name, model number) pairs its registry entry lists. Filled in for the
  Feetech STS3215 and SCS0009 (big-endian, no Sync Read) and the Dynamixel XL330 and
  XL430, resolution alone for MX, AX and XL320.
- Python: `resolution()`, `word_order()`, `supports_sync_read()`, `baudrates()` and
  `models()` on every controller class, static like `registers()`, and `encoding` /
  `sign_bit` on `RegisterInfo`.
- `set_baudrate(baudrate)` and `set_timeout(duration)` on every controller, and on the
  Python classes with the timeout in seconds like the constructor's. Both change the
  open port in place. A caller probing a bus at each rate a motor might be at, or
  shortening the timeout for an ID sweep, no longer has to close the controller and
  build a new one, which on Python could fail on a port still held by a traceback.
- Integer access to registers chosen by name: `read_register(id, name)`,
  `write_register(id, name, value)`, `sync_read_register(ids, name)` and
  `sync_write_register(ids, name, values)` on every controller, plus `_with_error`
  variants of the first three carrying the status packet's error field. They apply
  what the definition states about the register: the bytes go in the servo's word
  order (a big-endian servo swaps the two bytes of each 16-bit word, low word first)
  and the sign follows the register's encoding, so a Feetech sign-magnitude offset
  reads as `-709` and writes back as `0x0AC5`. A value that does not fit the register,
  or a name the servo does not define, fails before anything reaches the bus
  (`RegisterError`, a `ValueError` on Python). The same eight methods exist on the
  Python classes and release the GIL like the raw ones. `WordOrder::to_bytes` / `from_bytes`, `Encoding::encode` /
  `decode` and `RegisterInfo::encode` / `decode` are public for callers holding their
  own bytes.
- `scan(ids)` on every controller: which of `ids` answer, with their model number, as
  one Model Number read per id. The sweep runs under a read timeout sized to the baud
  rate (`servo::scan_timeout`: 320 bits of wire time, 5 ms at least) so that absent ids
  do not each cost the port's timeout, and puts the timeout back afterwards. On Python
  it returns `{id: model number}` and releases the GIL for the whole sweep. `scan_all()`,
  and `scan()` without ids on Python, sweep every id the protocol allows: 0 to 253 on v1,
  0 to 252 on v2, now given by `DynamixelProtocolHandler::max_id()`.
- `with_retries(retries, op)` on every controller runs a register access again, up to
  `retries` more times, while it fails on the bus: a timeout or a corrupted status packet.
  A bad name or value fails at once, and a motor answering with a fault is not retried,
  since the `_with_error` variants carry its error field. The Python by-name methods take
  it as a `retries=0` keyword, so a caller retrying a read no longer crosses back into
  Python between attempts.

## Version 1.9.0

- Feetech STS3215: `maximum_acceleration` (address 85) is one byte, not two, and
  `acceleration_multiplier` is added at 86. The two-byte size was inherited from an early
  LeRobot table that marked the register as not in the memory table; Feetech's memory
  table has two one-byte registers there, so a two-byte write clobbered the multiplier.
- XL430: the baud rate register is now spelled `baud_rate`, as on the XL330; it was
  `buad_rate`. The generated `read_buad_rate` / `write_buad_rate` accessors and their sync
  variants are renamed accordingly, in Rust and Python.
- Python: expose each servo's control table. Every controller class gets two static
  methods, `registers()` and `register(name)`, so a register name can be resolved to its
  address and size from Python without opening a serial port. The Rust side already had
  `REGISTERS` and `register()`; the Python side was left to copy the table by hand. Entries
  are `RegisterInfo` values (`name`, `addr`, `size`, `access`) and `access` is a
  `RegisterAccess` enum (`Read`, `Write`, `ReadWrite`); both types are new in the module.

## Version 1.8.0

- Expose each servo's control table: every servo module now has `REGISTERS: &[RegisterInfo]`
  and `register(name)`, giving each register's name, address, size and access (read, write
  or read-write). Code that picks a register at runtime no longer has to hand maintain a
  name to accessor match. `RegisterInfo` is `#[non_exhaustive]`.
- Add the `indirect_addressing` example, which points a block of indirect address slots at
  scattered registers so they can be read as one contiguous block, and compares it against
  one read per register and one wide read spanning the gaps. Registers are named on the
  command line and resolved through `REGISTERS`.
- Fix the documentation generated for the per-register accessors: a register with a
  conversion documented only its raw accessor, one without rendered the literal
  placeholder `Read register $name (addr: $addr, ...)`, and raw and converted labels were
  swapped on several controller and Python methods.
- Python: add `close()` and `is_open()` on the controllers, so the serial port is released
  at a known point instead of whenever the object happens to be dropped. Methods called on
  a closed controller raise `RuntimeError`.
- Python: release the GIL for the duration of every serial transaction. Other Python
  threads now keep running while the bus is busy, instead of being blocked for the whole
  read or write. Covers the raw-address bindings and every generated per-register
  accessor (`read_*`, `sync_read_*`, `write_*`, `sync_write_*`), plus `ping`, `reboot`
  and `factory_reset`. The Python API is unchanged -- the generated `rustypot.pyi` is
  byte-identical.
- Document `sys.setswitchinterval` in the README: it caps how much GIL time a competing
  thread gets back, and its 5 ms default dominates a millisecond-scale bus transaction.
- Surface the status packet's error field, which was parsed and then dropped, so a caller
  can tell a healthy motor from one answering while it reports a fault. Adds
  `read_with_error`, `write_with_error` and `sync_read_with_error` on the protocol handler
  (sync read included, since a control loop polls there), the matching
  `read_raw_data_with_error`, `write_raw_data_with_error` and
  `sync_read_raw_data_with_error` on the controllers, and all three in Python.
  `read`/`write`/`sync_read` are untouched.
- Add `StatusError`, a newtype over that byte with the two protocol readings named:
  `v1_conditions()` decodes the v1 bitfield, while `v2_instruction_error()` and
  `v2_alert()` read the v2 layout, where bits 0-6 are an error *number* rather than
  flags. `DynamixelErrorV1` is now public. Python receives the raw byte, as the vendor
  SDKs do.
- Fix `with_post_delay` being ignored by `sync_read`, `fast_sync_read` and `sync_write`.
  The delay is documented as applying after each communication, but only `read`, `write`
  and `write_fb` ever slept it, so a control loop polling with `sync_read` -- the common
  case on hardware that needs the gap -- ran without one. Every transaction now sleeps
  it, including one that failed, which is when an immediate retry would otherwise close
  the gap; `write` and `write_with_error` previously skipped it on error and no longer
  do.

## Version 1.7.0

- Add fast sync read (protocol v2 instruction 0x8A): every motor appends its answer to a
  single status packet returned from the broadcast id, instead of each sending its own.
  Available as `DynamixelProtocolHandler::fast_sync_read`, or by enabling it once with
  `with_fast_sync_read()` so all `sync_read_*` methods use it. Requires firmware new
  enough to implement it (XL330: v46+); older firmware does not answer and the read
  times out.
- Add the `fast_sync_read_bench` example, which checks both instructions return the same
  data and times them side by side.
- Python: add `set_fast_sync_read()` on the protocol v2 controllers, which reroutes the
  existing `sync_read_*` methods.

## Version 1.6.0

- Handle Dynamixel protocol 2.0 byte stuffing: status packets whose data contains FF FF FD are now de-stuffed on reception (previously returned oversized params, breaking fixed-size reads e.g. when present current = -1), and instruction packet params are stuffed on transmission.
- Bump pyo3 to 0.29.0 and pyo3-stub-gen to 0.23.0 (fixes RUSTSEC-2026-0176, an out-of-bounds read in the PyList/PyTuple iterators).
- Track Cargo.lock in the repository.
- Add a memory soak test example.

## Version 1.5.0

- Add support for feetech Scs0043 motor.
- Pin pyo3 to 0.27.1 and pyo3-stub-gen to 0.21.0.

### Version 1.4.1

- Make flush method more robust and replace potential panic with error.

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
