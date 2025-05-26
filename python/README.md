# Rustypot python bindings

## Installation

Rustypot bindings are available on PyPI. You can install them using pip:

```bash
pip install rustypot
```

### Building the bindings

If you want to build the bindings from source, you can clone the repository and run the following command:

```bash
maturin develop --release --features python
```

## Usage

The Python bindings exposes the same API as the Controller API in the rust crate. 

You first need to create a Controller object. For instance, to communicate with a serial port to Feetech STS3215 motors, you can do the following:

```python
from rustypot.servo import Sts3215SyncController

c = Sts3215SyncController(serial_port='/dev/ttyUSB0', baudrate=100000, timeout=0.1)
```

Then, you can directly read/write any register of the motor. For instance, to read the present position of the motors with id 1 and 2, you can do:

```python

pos = c.read_present_position([1, 2])
print(pos)
```

You can also write to the motors. For instance, to set the goal position of the motors with id 1 and 2 to 0.0 and 90° respectively, you can do:

```python
import numpy as np
c.write_goal_position([1, 2], [0.0, np.deg2rad(90.0)])
```