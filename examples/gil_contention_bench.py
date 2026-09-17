"""Measure what a serial transaction costs the rest of a Python process.

The bindings release the GIL around every transaction, so a second Python thread --
a camera loop, a policy -- can run while the bus is busy. This clocks both sides of
that: how fast the bus goes, and how much of its solo throughput the other thread
keeps while the bus runs.

Two numbers move together and the trade-off between them is the point:

  - Releasing the GIL costs bus throughput, because re-acquiring it after each
    transaction means waiting for the other thread to give it up.
  - How long that wait is comes from `sys.setswitchinterval`, 5 ms by default. On a
    transaction of a millisecond or two, that default dominates. Lowering it buys
    back most of the bus rate, so this sweeps a few values.

The competing thread here is a bare Python loop, which is the worst case: it wants
the GIL continuously. A real thread that spends its time in a native call (decoding
a frame, running a model) releases the GIL itself and interferes far less.

```sh
python examples/gil_contention_bench.py \
    --serialport /dev/tty.usbserial-XXXX --baudrate 1000000 --ids 1,2,3,4,5,6
```
"""

import argparse
import sys
import threading
import time

from rustypot import Sts3215PyController


def spin_rate(stop_flag):
    """Count bare-Python loop iterations per second until stop_flag is set."""
    spins = 0
    t0 = time.perf_counter()
    while not stop_flag[0]:
        spins += 1
    return spins / (time.perf_counter() - t0)


def measure_solo_spin(duration):
    """Spin rate of the competing thread with the bus idle, as the 100% reference."""
    stop = [False]
    result = {}

    def run():
        result["rate"] = spin_rate(stop)

    t = threading.Thread(target=run, daemon=True)
    t.start()
    time.sleep(duration)
    stop[0] = True
    t.join(timeout=1)
    return result["rate"]


def measure_contended(controller, ids, iterations):
    """Bus rate and competing-thread spin rate while both run."""
    stop = [False]
    result = {}

    def run():
        result["rate"] = spin_rate(stop)

    t = threading.Thread(target=run, daemon=True)
    t.start()
    t0 = time.perf_counter()
    for _ in range(iterations):
        controller.sync_read_present_position(ids)
    wall = time.perf_counter() - t0
    stop[0] = True
    t.join(timeout=1)
    return iterations / wall, result["rate"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serialport", required=True)
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--ids", default="1", help="comma-separated motor ids")
    parser.add_argument("--iterations", type=int, default=500)
    parser.add_argument(
        "--switch-intervals",
        default="0.005,0.001,0.0001",
        help="sys.setswitchinterval values to sweep, in seconds",
    )
    args = parser.parse_args()

    ids = [int(i) for i in args.ids.split(",")]
    intervals = [float(v) for v in args.switch_intervals.split(",")]

    c = Sts3215PyController(
        serial_port=args.serialport,
        baudrate=args.baudrate,
        timeout=0.5,
    )

    # Warm up the chain so the first timed transaction is not the first one ever.
    for _ in range(50):
        c.sync_read_present_position(ids)

    print(f"{len(ids)} motor(s) at {args.baudrate} baud, {args.iterations} iterations per row")

    t0 = time.perf_counter()
    for _ in range(args.iterations):
        c.sync_read_present_position(ids)
    uncontended = args.iterations / (time.perf_counter() - t0)
    print(f"\nbus alone, no competing thread: {uncontended:.1f} Hz")

    solo = measure_solo_spin(1.0)
    print(f"competing thread alone:        {solo / 1e6:.1f} M spins/s\n")

    print(f"{'switch interval':>16}{'bus':>10}{'vs alone':>10}{'other thread':>14}{'vs alone':>10}")
    for interval in intervals:
        sys.setswitchinterval(interval)
        bus, spins = measure_contended(c, ids, args.iterations)
        print(
            f"{interval * 1e3:>13.2f} ms{bus:>8.1f} Hz{bus / uncontended:>9.0%}"
            f"{spins / 1e6:>10.1f} M/s{spins / solo:>10.0%}"
        )

    print(
        "\nBoth columns matter: a higher bus rate bought by starving the other thread is "
        "not a win. Pick the interval that keeps both acceptable for your loop."
    )


if __name__ == "__main__":
    main()
