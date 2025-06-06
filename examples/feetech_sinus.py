import time
import numpy as np

from rustypot import Sts3215PyController

def main():
    c = Sts3215PyController(
        serial_port='/dev/tty.usbmodem58FA0822621',
        baudrate=1000000,
        timeout=0.1,
    )

    c.write_torque_enable([1, 2], [True, True])

    t0 = time.time()

    while True:
        t = time.time() - t0
        pos = np.sin(2 * np.pi * 0.25 * t) * np.deg2rad(45)

        c.write_goal_position([1, 2], [pos, pos])
        print(c.read_present_position([1, 2]))

        time.sleep(0.01)

if __name__ == '__main__':
    main()
