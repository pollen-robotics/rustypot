//! Compare sync read (0x82) and fast sync read (0x8A) on Dynamixel XL330/XL430 motors.
//!
//! Fast sync read sends the same instruction packet as sync read, apart from the
//! instruction byte itself (0x8A instead of 0x82, and therefore the CRC). It gathers the
//! same data, but every motor appends its answer to a single status packet returned from
//! the broadcast id instead of sending its own. That replaces each motor's 11 byte status
//! packet frame with a 4 byte block (error, id, crc) at the cost of one 8 byte frame for
//! the whole reply (a net 7n - 8 bytes for n motors) and leaves a single bus
//! turnaround, with its return delay time, instead of one per motor.
//!
//! This example first checks that both instructions return the same bytes, then clocks them
//! for 1..n motors so the per motor cost of each can be compared against the predicted value.
//!
//! ```sh
//! cargo run --release --example fast_sync_read_bench -- \
//!     --serialport /dev/tty.usbserial-XXXX --baudrate 1000000 --ids 1,2,3,4,5,6
//! ```
//!
//! Requires protocol v2 firmware implementing fast sync read (e.g. XL330: v46+).

use std::{error::Error, time::Duration, time::Instant};

use clap::Parser;
use rustypot::DynamixelProtocolHandler;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Serial port (e.g. /dev/ttyUSB0 or /dev/tty.usbmodemXXXX)
    #[arg(short, long)]
    serialport: String,
    /// Baudrate
    #[arg(short, long, default_value_t = 1_000_000)]
    baudrate: u32,

    /// Motor ids, comma separated (e.g. 1,2,3,4,5,6)
    #[arg(short, long, value_delimiter = ',')]
    ids: Vec<u8>,

    /// First register to read (default 132: present position)
    #[arg(short, long, default_value_t = 132)]
    addr: u8,

    /// Number of bytes to read per motor (default 4: one i32)
    #[arg(short, long, default_value_t = 4)]
    length: u8,

    /// Timed iterations per measurement
    #[arg(short = 'n', long, default_value_t = 1000)]
    iterations: usize,

    /// Untimed iterations before each measurement
    #[arg(short, long, default_value_t = 50)]
    warmup: usize,

    /// Only time the full set of ids, skip the 1..n sweep
    #[arg(long, default_value_t = false)]
    no_sweep: bool,

    /// Write this return delay time (unit: 2 us) to every motor first, to see how much of
    /// the difference comes from the cost of one bus turnaround per motor. Writes EEPROM,
    /// is not restored afterwards. The X series factory default is 250 (500 us).
    #[arg(long)]
    return_delay_time: Option<u8>,

    /// Check how both instructions behave when the data contains the FF FF FD pattern
    #[arg(long, default_value_t = false)]
    probe_stuffing: bool,
}

/// Timing of one batch of identical reads.
struct Stats {
    mean: f64,
    p50: f64,
    p95: f64,
    errors: usize,
}

impl Stats {
    fn of(mut samples: Vec<f64>, errors: usize) -> Self {
        samples.sort_by(f64::total_cmp);
        let pct = |p: f64| samples[((samples.len() - 1) as f64 * p) as usize];
        Stats {
            mean: samples.iter().sum::<f64>() / samples.len() as f64,
            p50: pct(0.50),
            p95: pct(0.95),
            errors,
        }
    }
}

/// Time `iterations` reads, discarding the ones that failed but counting them.
fn bench(
    dph: &DynamixelProtocolHandler,
    serial_port: &mut dyn serialport::SerialPort,
    ids: &[u8],
    args: &Args,
    fast: bool,
) -> Stats {
    let (addr, length) = (args.addr, args.length);
    let read = |port: &mut dyn serialport::SerialPort| {
        if fast {
            dph.fast_sync_read(port, ids, addr, length)
        } else {
            dph.sync_read(port, ids, addr, length)
        }
    };

    for _ in 0..args.warmup {
        let _ = read(&mut *serial_port);
    }

    let mut samples = Vec::with_capacity(args.iterations);
    let mut errors = 0;
    for _ in 0..args.iterations {
        let t = Instant::now();
        let res = read(&mut *serial_port);
        let dt = t.elapsed().as_secs_f64() * 1e3;
        match res {
            Ok(_) => samples.push(dt),
            Err(_) => errors += 1,
        }
    }

    if samples.is_empty() {
        samples.push(f64::NAN);
    }
    Stats::of(samples, errors)
}

/// Total bytes the bus carries, instruction plus answers, for `n` motors reading
/// `length` bytes each. The bus is half duplex, so the two directions cannot overlap
/// and their sizes simply add, giving the total bus traffic for one transaction
/// (not counting the return delay time, which is idle bus rather than bytes).
///
/// sync read:      instruction 14 + n, then n status packets of 11 + length
/// fast sync read: instruction 14 + n, then one status packet of 8 + n * (length + 4)
fn wire_bytes(n: usize, length: u8, fast: bool) -> usize {
    let instruction = 14 + n;
    let status = if fast {
        8 + n * (length as usize + 4)
    } else {
        n * (11 + length as usize)
    };
    instruction + status
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let ids = args.ids.clone();
    if ids.is_empty() {
        return Err("no ids given, pass e.g. --ids 1,2,3".into());
    }

    let mut serial_port = serialport::new(&args.serialport, args.baudrate)
        .timeout(Duration::from_millis(20))
        .open()?;
    let serial_port = serial_port.as_mut();

    let dph = DynamixelProtocolHandler::v2();

    // --- bus inventory -----------------------------------------------------------------
    println!("bus: {} at {} baud", args.serialport, args.baudrate);

    if let Some(rdt) = args.return_delay_time {
        for &id in &ids {
            if dph.read(serial_port, id, 64, 1)?[0] != 0 {
                return Err(format!("motor {id} has torque enabled, cannot write EEPROM").into());
            }
            dph.write(serial_port, id, 9, &[rdt])?;
        }
        println!(
            "return delay time set to {rdt} ({} us) on every motor",
            rdt as u32 * 2
        );
    }

    println!("{:<6}{:<12}{:<12}return delay", "id", "firmware", "model");
    for &id in &ids {
        if !dph.ping(serial_port, id)? {
            return Err(format!("motor {id} did not answer to a ping").into());
        }
        let firmware = dph.read(serial_port, id, 6, 1)?[0];
        let model = u16::from_le_bytes(dph.read(serial_port, id, 0, 2)?.try_into().unwrap());
        let rdt = dph.read(serial_port, id, 9, 1)?[0];
        println!(
            "{:<6}{:<12}{:<12}{} ({} us){}",
            id,
            firmware,
            model,
            rdt,
            rdt as u32 * 2,
            if firmware < 45 {
                "   <- too old for fast sync read"
            } else {
                ""
            }
        );
    }
    println!();

    // --- correctness -------------------------------------------------------------------
    // Compared on the first 10 registers, which never change, so both instructions can be
    // checked against each other without the values moving.
    let slow = dph.sync_read(serial_port, &ids, 0, 10)?;
    let fast = dph.fast_sync_read(serial_port, &ids, 0, 10)?;
    if slow != fast {
        println!("Mismatch between sync read and fast sync read");
        for (i, &id) in ids.iter().enumerate() {
            println!("  id {id}: sync {:02X?} / fast {:02X?}", slow[i], fast[i]);
        }
        return Err("fast sync read returned different data".into());
    }
    println!(
        "sync read and fast sync read agree on all {} motors",
        ids.len()
    );

    if args.probe_stuffing {
        probe_stuffing(&dph, serial_port, &ids)?;
    }
    println!();

    // --- timing ------------------------------------------------------------------------
    let byte_us = 10.0 / args.baudrate as f64 * 1e6; // 8N1: 10 bits per byte
    println!(
        "reading {} bytes at addr {} per motor, {} iterations each\n",
        args.length, args.addr, args.iterations
    );
    println!(
        "{:<4}{:>26}{:>26}{:>10}{:>18}",
        "n", "sync read (ms)", "fast sync read (ms)", "speedup", "wire bytes"
    );
    println!(
        "{:<4}{:>10}{:>8}{:>8}{:>10}{:>8}{:>8}{:>10}{:>18}",
        "", "mean", "p50", "p95", "mean", "p50", "p95", "mean", "sync -> fast"
    );

    let counts: Vec<usize> = if args.no_sweep {
        vec![ids.len()]
    } else {
        (1..=ids.len()).collect()
    };

    for n in counts {
        let subset = &ids[..n];
        let s = bench(&dph, serial_port, subset, &args, false);
        let f = bench(&dph, serial_port, subset, &args, true);

        println!(
            "{:<4}{:>10.3}{:>8.3}{:>8.3}{:>10.3}{:>8.3}{:>8.3}{:>9.2}x{:>11} -> {:<4}",
            n,
            s.mean,
            s.p50,
            s.p95,
            f.mean,
            f.p50,
            f.p95,
            s.mean / f.mean,
            wire_bytes(n, args.length, false),
            wire_bytes(n, args.length, true),
        );
        if s.errors + f.errors > 0 {
            println!("      errors: sync {} / fast {}", s.errors, f.errors);
        }
    }

    // Bus traffic prediction. Two effects add up: fewer bytes, and fewer status packets
    // (so fewer return delay times and fewer host side reads).
    let n = ids.len();
    let rdt_us = dph.read(serial_port, ids[0], 9, 1)?[0] as f64 * 2.0;
    let (slow_bytes, fast_bytes) = (
        wire_bytes(n, args.length, false),
        wire_bytes(n, args.length, true),
    );
    println!("\nwhat the wire format predicts for {n} motors:");
    println!(
        "  bytes           {slow_bytes} -> {fast_bytes} ({:.2}x), i.e. {:.3} ms -> {:.3} ms at {} baud",
        slow_bytes as f64 / fast_bytes as f64,
        slow_bytes as f64 * byte_us / 1e3,
        fast_bytes as f64 * byte_us / 1e3,
        args.baudrate,
    );
    println!(
        "  status packets  {n} -> 1, so {:.3} ms -> {:.3} ms of return delay time ({rdt_us} us each)",
        n as f64 * rdt_us / 1e3,
        rdt_us / 1e3,
    );
    println!(
        "  total           {:.3} ms -> {:.3} ms ({:.2}x)",
        (slow_bytes as f64 * byte_us + n as f64 * rdt_us) / 1e3,
        (fast_bytes as f64 * byte_us + rdt_us) / 1e3,
        (slow_bytes as f64 * byte_us + n as f64 * rdt_us) / (fast_bytes as f64 * byte_us + rdt_us),
    );
    println!(
        "  anything measured on top of that is host side: USB turnaround (the FTDI latency\n\
        \x20 timer, 16 ms unless lowered) and one read syscall per status packet."
    );

    Ok(())
}

/// Check what happens when the returned data contains FF FF FD.
///
/// A regular status packet is byte stuffed (the motor inserts an extra FD after FF FF FD)
/// and rustypot undoes that. The official SDK reads fast sync read answers with stuffing
/// removal explicitly skipped, so it assumes motors do not stuff there. This puts the
/// pattern in the data on purpose to find out which is true on this firmware.
///
/// Goal Velocity (addr 104) set to -1 gives FF FF FF FF and Profile Acceleration
/// (addr 108) set to 253 gives FD, so addr 104..110 reads FF FF FF FF FD 00. Both are RAM
/// registers, nothing touches EEPROM, torque stays off so no motor moves, and both are
/// restored afterwards.
///
/// Whether a motor stores those values verbatim depends on its operating mode: a mode that
/// treats the register as a magnitude or drives it itself will hand back something else.
/// So the probe reports which motors actually ended up carrying the pattern and only needs
/// one of them, while still comparing every motor against its own reference read.
fn probe_stuffing(
    dph: &DynamixelProtocolHandler,
    serial_port: &mut dyn serialport::SerialPort,
    ids: &[u8],
) -> Result<(), Box<dyn Error>> {
    println!("\nstuffing probe: putting FF FF FD in the data of every motor");

    for &id in ids {
        if dph.read(serial_port, id, 64, 1)?[0] != 0 {
            println!("  skipped: motor {id} has torque enabled, disable it first");
            return Ok(());
        }
    }

    // Goal velocity and profile acceleration, saved together so one write restores both.
    let saved: Vec<Vec<u8>> = ids
        .iter()
        .map(|&id| dph.read(serial_port, id, 104, 8))
        .collect::<Result<_, _>>()?;

    for &id in ids {
        dph.write(serial_port, id, 104, &(-1i32).to_le_bytes())?;
        dph.write(serial_port, id, 108, &253u32.to_le_bytes())?;
    }

    // One plain read per motor is the reference: it already handles byte
    // stuffing, and it isolates each motor from the others.
    let reference: Vec<Vec<u8>> = ids
        .iter()
        .map(|&id| dph.read(serial_port, id, 104, 6))
        .collect::<Result<_, _>>()?;
    let planted = |d: &Vec<u8>| d.windows(3).any(|w| w == [0xFF, 0xFF, 0xFD]);
    let carrying: Vec<u8> = ids
        .iter()
        .zip(&reference)
        .filter(|(_, d)| planted(d))
        .map(|(&id, _)| id)
        .collect();

    for (&id, d) in ids.iter().zip(&reference) {
        let mode = dph.read(serial_port, id, 11, 1)?[0];
        println!(
            "  id {id} (mode {mode}) reads {d:02X?}{}",
            if planted(d) { "" } else { "   <- no FF FF FD" }
        );
    }

    if !carrying.is_empty() {
        println!(
            "  {} of {} motors carry the pattern",
            carrying.len(),
            ids.len()
        );

        match dph.sync_read(serial_port, ids, 104, 6) {
            Ok(v) if v == reference => println!("  sync read      -> correct, stuffing removed"),
            Ok(v) => println!("  sync read      -> WRONG: {v:02X?}"),
            Err(e) => println!("  sync read      -> failed: {e}"),
        }
        match dph.fast_sync_read(serial_port, ids, 104, 6) {
            Ok(v) if v == reference => println!(
                "  fast sync read -> correct: the answer is not byte stuffed, matching\n\
                 \x20                 what the official SDK assumes"
            ),
            Ok(v) => println!("  fast sync read -> WRONG: {v:02X?}"),
            Err(e) => println!(
                "  fast sync read -> failed ({e}): the answer is byte stuffed after all,\n\
                 \x20                 which the fixed stride parsing does not expect"
            ),
        }
    } else {
        println!("  skipped: could not get FF FF FD into any motor's data");
    }

    for (&id, data) in ids.iter().zip(&saved) {
        // Restore even if something above went wrong, retrying once past a stale bus.
        if dph.write(serial_port, id, 104, data).is_err() {
            dph.write(serial_port, id, 104, data)?;
        }
    }
    println!("  goal velocity / profile acceleration restored");

    Ok(())
}
