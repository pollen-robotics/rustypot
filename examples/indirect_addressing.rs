//! Gather non-contiguous registers into one contiguous read using indirect addressing.
//!
//! A control loop may want a handful of registers that are not contiguous.
//! Reading them costs either one transaction per group, or one wide read that drags along
//! every byte in between. Indirect addressing solves this by pointing a block of
//! indirect address slots at the bytes you actually want, and they appear contiguously in
//! the indirect data region, so one short read returns exactly your registers.
//!
//! This example uses `realtime_tick` (120, 2 bytes) and `present_position` (132, 4 bytes)
//! as the per tick block, and `present_input_voltage` (144, 2 bytes) as a slower one, then
//! compares three ways of reading them. The slow registers are intentionally mapped after
//! the fast ones, so the same read can cover 6 bytes on a normal tick or 8 when the slow
//! values are wanted. In this example, the base address never changes, only the length,
//! although the slow block can be read separately if desired.
//!
//! ```sh
//! cargo run --release --example indirect_addressing -- \
//!     --serialport /dev/tty.usbserial-XXXX --ids 1,2,3,4,5,6
//! ```
//!
//! Two notes before considering indirect mapping:
//!
//! * An indirect address may only point at the RAM area (address 64 and up). Pointing one
//!   at EEPROM is rejected by the servo, and because rustypot discards the status packet's
//!   error byte the write returns `Ok` while quietly doing nothing -- so always read the
//!   table back, as this example does.
//! * Indirect addresses live in RAM themselves, so the map is lost on every power cycle
//!   and has to be reapplied at startup.
//!
//! Requires protocol v2 firmware with indirect data at 224 (XL330: v53+; earlier firmware
//! puts it at 208). This example restores whatever map it found when it exits.

use std::{error::Error, time::Duration, time::Instant};

use clap::Parser;
use rustypot::{servo::dynamixel::xl330, DynamixelProtocolHandler};

/// Where the two indirect blocks start: `indirect_address_1` is the first pointer slot,
/// `indirect_data_1` the first byte they gather into.
fn indirect_bases() -> Result<(u8, u8, u8), Box<dyn Error>> {
    let addr =
        xl330::register("indirect_address_1").ok_or("this servo defines no indirect_address_1")?;
    let data = xl330::register("indirect_data_1").ok_or("this servo defines no indirect_data_1")?;
    // Each pointer slot is one u16, which is what indirect_map writes per byte gathered.
    Ok((addr.addr, data.addr, addr.size))
}

/// Registers used to prove the gather is correct without racing live values: in RAM,
/// constant in practice, and not adjacent to each other.
const CONST_REGS: &[&str] = &["torque_enable", "status_return_level", "position_p_gain"];

/// Resolve register names to (address, size) through the servo's own register table, so
/// the map can be chosen at runtime instead of hardcoded.
fn resolve(names: &[impl AsRef<str>]) -> Result<Vec<(u8, u8)>, Box<dyn Error>> {
    names
        .iter()
        .map(|n| {
            let n = n.as_ref();
            let r = xl330::register(n).ok_or_else(|| {
                format!(
                    "unknown register {n:?} ({} are defined)",
                    xl330::REGISTERS.len()
                )
            })?;
            // Indirect addresses can only point at RAM. The servo rejects anything lower,
            // and the write reports Ok while doing nothing, so catch it here instead.
            if r.addr < 64 {
                return Err(format!(
                    "{n} is at {} in EEPROM; indirect addressing only reaches RAM (64+)",
                    r.addr
                )
                .into());
            }
            Ok((r.addr, r.size))
        })
        .collect()
}

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// tty
    #[arg(short, long, default_value = "/dev/ttyUSB0")]
    serialport: String,
    /// baud
    #[arg(short, long, default_value_t = 1_000_000)]
    baudrate: u32,
    /// Motor ids to read
    #[arg(short, long, value_delimiter = ',')]
    ids: Vec<u8>,
    /// Registers to gather every tick, by name
    #[arg(
        long,
        value_delimiter = ',',
        default_value = "realtime_tick,present_position"
    )]
    fast: Vec<String>,
    /// Registers appended for the occasional slower read
    #[arg(long, value_delimiter = ',', default_value = "present_input_voltage")]
    slow: Vec<String>,
    /// Timed iterations per measurement
    #[arg(short = 'n', long, default_value_t = 500)]
    iterations: usize,
    /// Untimed iterations before each measurement
    #[arg(short, long, default_value_t = 30)]
    warmup: usize,
}

/// Build the indirect address table for `entries`.
///
/// Each slot points at exactly ONE byte, so an n byte register needs n consecutive slots
/// pointing at addr, addr+1, ... This part is easy to get wrong by hand.
fn indirect_map(entries: &[(u8, u8)]) -> Vec<u8> {
    entries
        .iter()
        .flat_map(|&(addr, len)| (0..len).map(move |i| addr as u16 + i as u16))
        .flat_map(|a| a.to_le_bytes())
        .collect()
}

fn block_len(entries: &[(u8, u8)]) -> u8 {
    entries.iter().map(|&(_, len)| len).sum()
}

/// Bytes on the bus for one sync read of `len` bytes from `n` motors.
fn wire_bytes(n: usize, len: u8, fast: bool) -> usize {
    let instruction = 14 + n;
    let status = if fast {
        8 + n * (len as usize + 4)
    } else {
        n * (11 + len as usize)
    };
    instruction + status
}

/// One transaction per register group, then stitch each motor's bytes back together.
fn read_separate(
    dph: &DynamixelProtocolHandler,
    port: &mut dyn serialport::SerialPort,
    ids: &[u8],
    groups: &[(u8, u8)],
) -> Result<Vec<Vec<u8>>, Box<dyn Error>> {
    let mut out = vec![Vec::new(); ids.len()];
    for &(addr, len) in groups {
        for (slot, data) in out.iter_mut().zip(dph.sync_read(port, ids, addr, len)?) {
            slot.extend(data);
        }
    }
    Ok(out)
}

/// One wide read covering everything, including unwanted bytes.
fn read_wide(
    dph: &DynamixelProtocolHandler,
    port: &mut dyn serialport::SerialPort,
    ids: &[u8],
    groups: &[(u8, u8)],
    wide_addr: u8,
    wide_len: u8,
) -> Result<Vec<Vec<u8>>, Box<dyn Error>> {
    let blocks = dph.sync_read(port, ids, wide_addr, wide_len)?;
    Ok(blocks
        .iter()
        .map(|b| {
            groups
                .iter()
                .flat_map(|&(addr, len)| {
                    let off = (addr - wide_addr) as usize;
                    b[off..off + len as usize].to_vec()
                })
                .collect()
        })
        .collect())
}

fn mean_ms(
    mut f: impl FnMut() -> Result<(), Box<dyn Error>>,
    warmup: usize,
    iterations: usize,
) -> (f64, usize) {
    for _ in 0..warmup {
        let _ = f();
    }
    let mut errors = 0;
    let t = Instant::now();
    for _ in 0..iterations {
        if f().is_err() {
            errors += 1;
        }
    }
    (t.elapsed().as_secs_f64() * 1e3 / iterations as f64, errors)
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let ids = args.ids.clone();
    if ids.is_empty() {
        return Err("no ids given, pass e.g. --ids 1,2,3".into());
    }
    let n = ids.len();

    let mut serial_port = serialport::new(&args.serialport, args.baudrate)
        .timeout(Duration::from_millis(30))
        .open()?;
    let port = serial_port.as_mut();

    let dph = DynamixelProtocolHandler::v2();
    let fast_dph = DynamixelProtocolHandler::v2().with_fast_sync_read();

    let (indirect_addr, indirect_data, slot_size) = indirect_bases()?;

    let fast_regs = resolve(&args.fast)?;
    let slow_regs = resolve(&args.slow)?;
    let const_regs = resolve(CONST_REGS)?;
    let all: Vec<(u8, u8)> = fast_regs.iter().chain(&slow_regs).copied().collect();
    let map = indirect_map(&all);
    let (fast_len, all_len) = (block_len(&fast_regs), block_len(&all));

    // What a single contiguous read would have to span to reach every chosen register.
    let wide_addr = all.iter().map(|&(a, _)| a).min().unwrap();
    let wide_len = all.iter().map(|&(a, l)| a + l).max().unwrap() - wide_addr;

    println!(
        "bus: {} at {} baud, {n} motors",
        args.serialport, args.baudrate
    );
    for &id in &ids {
        let fw = dph.read(port, id, 6, 1)?[0];
        if fw < 53 {
            println!("  motor {id} firmware v{fw}: indirect data is at 208 below v53, not 224");
        }
    }
    println!("  indirect address slots at {indirect_addr}, gathered data at {indirect_data}");
    println!(
        "\nmapping {all_len} bytes into indirect data: fast {:?} then slow {:?}\n  \
         a single contiguous read would need {wide_len} bytes from {wide_addr}",
        args.fast, args.slow
    );

    // --- correctness, on registers that cannot change under us --------------------
    let saved: Vec<Vec<u8>> = ids
        .iter()
        .map(|&id| dph.read(port, id, indirect_addr, map.len() as u8))
        .collect::<Result<_, _>>()?;

    dph.sync_write(
        port,
        &ids,
        indirect_addr,
        &vec![indirect_map(&const_regs); n],
    )?;
    let direct = read_separate(&dph, port, &ids, &const_regs)?;
    let gathered = dph.sync_read(port, &ids, indirect_data, block_len(&const_regs))?;
    if direct != gathered {
        println!("  direct   {direct:02X?}\n  indirect {gathered:02X?}");
        return Err("indirect data does not match direct reads".into());
    }
    println!(
        "\ngathered {CONST_REGS:?} through indirect data: matches direct reads on all {n} motors"
    );

    // --- writing the real map -------------------------------------------------------
    // Indirect Address 1..N is contiguous, so the whole table for every motor fits in one
    // sync write. A slot at a time costs n * slots transactions, each awaiting a status
    // packet, which is what makes the difference so large.
    // Both paths finish with the same read back, so the timings are comparable: a sync
    // write gets no status packet, so without it the measurement would only capture the
    // host queueing bytes rather than the servos applying them.
    let confirm = |port: &mut dyn serialport::SerialPort| {
        dph.read(port, ids[n - 1], indirect_addr, map.len() as u8)
    };

    let t = Instant::now();
    for &id in &ids {
        for (slot, chunk) in map.chunks(2).enumerate() {
            dph.write(port, id, indirect_addr + slot_size * slot as u8, chunk)?;
        }
    }
    confirm(port)?;
    let per_slot = t.elapsed().as_secs_f64() * 1e3;

    // Put the table back so the batched write has the same work to do.
    dph.sync_write(
        port,
        &ids,
        indirect_addr,
        &vec![indirect_map(&const_regs); n],
    )?;
    confirm(port)?;

    let t = Instant::now();
    dph.sync_write(port, &ids, indirect_addr, &vec![map.clone(); n])?;
    let landed = confirm(port)?;
    let batched = t.elapsed().as_secs_f64() * 1e3;

    // The servo rejects an out of range target silently, so check rather than trust Ok.
    if landed != map {
        return Err(format!("map did not land: wrote {map:02X?}, read {landed:02X?}").into());
    }

    // A single register write is 14 bytes out and an 11 byte status back. A sync write
    // carries every motor's table in one packet and gets no status at all.
    let slots = map.len() / 2;
    let per_slot_bytes = n * slots * (14 + 11);
    let batched_bytes = 14 + n * (1 + map.len());

    println!("\nwriting the {slots}-slot map to {n} motors (each timing includes one read back):");
    println!(
        "{:<34}{:>10}{:>12}{:>14}",
        "", "mean (ms)", "wire bytes", "transactions"
    );
    println!(
        "{:<34}{per_slot:>10.2}{per_slot_bytes:>12}{:>14}",
        "  one write per slot",
        n * slots
    );
    println!(
        "{:<34}{batched:>10.2}{batched_bytes:>12}{:>14}",
        "  one sync write", 1
    );
    println!(
        "  -> {:.0}x faster, {:.0}x fewer bytes",
        per_slot / batched,
        per_slot_bytes as f64 / batched_bytes as f64
    );

    // Sanity check the live map landed, and show what it reads. The block layout follows
    // whichever registers were asked for, so walk it rather than assuming offsets.
    let indirect = dph.sync_read(port, &ids, indirect_data, all_len)?;
    let names = args.fast.iter().chain(&args.slow);
    let mut off = 0;
    print!("  id {} reads", ids[0]);
    for (name, &(_, len)) in names.zip(&all) {
        let bytes = &indirect[0][off..off + len as usize];
        let value = bytes
            .iter()
            .rev()
            .fold(0u32, |acc, &b| (acc << 8) | b as u32);
        print!("  {name} {value}");
        off += len as usize;
    }
    println!();

    // --- timing -----------------------------------------------------------------------
    println!(
        "\n{:<34}{:>10}{:>12}{:>14}",
        "", "mean (ms)", "wire bytes", "transactions"
    );
    let separate_bytes = |fast: bool| {
        all.iter()
            .map(|&(_, l)| wire_bytes(n, l, fast))
            .sum::<usize>()
    };

    for (label, fast) in [("sync read", false), ("fast sync read", true)] {
        let d = if fast { &fast_dph } else { &dph };
        println!("{label}:");

        let (ms, _) = mean_ms(
            || read_separate(d, port, &ids, &all).map(|_| ()),
            args.warmup,
            args.iterations,
        );
        println!(
            "{:<34}{ms:>10.3}{:>12}{:>14}",
            "  one read per register",
            separate_bytes(fast),
            all.len()
        );

        let (ms, _) = mean_ms(
            || read_wide(d, port, &ids, &all, wide_addr, wide_len).map(|_| ()),
            args.warmup,
            args.iterations,
        );
        println!(
            "{:<34}{ms:>10.3}{:>12}{:>14}",
            &format!("  one wide read ({wide_len} B)"),
            wire_bytes(n, wide_len, fast),
            1
        );

        let (ms, _) = mean_ms(
            || d.sync_read(port, &ids, indirect_data, all_len).map(|_| ()),
            args.warmup,
            args.iterations,
        );
        println!(
            "{:<34}{ms:>10.3}{:>12}{:>14}",
            &format!("  indirect, fast+slow ({all_len} B)"),
            wire_bytes(n, all_len, fast),
            1
        );

        let (ms, _) = mean_ms(
            || d.sync_read(port, &ids, indirect_data, fast_len).map(|_| ()),
            args.warmup,
            args.iterations,
        );
        println!(
            "{:<34}{ms:>10.3}{:>12}{:>14}",
            &format!("  indirect, fast only ({fast_len} B)"),
            wire_bytes(n, fast_len, fast),
            1
        );
    }

    // --- restore ----------------------------------------------------------------------
    for (&id, data) in ids.iter().zip(&saved) {
        if dph.write(port, id, indirect_addr, data).is_err() {
            dph.write(port, id, indirect_addr, data)?;
        }
    }
    println!("\nindirect address table restored");

    Ok(())
}
