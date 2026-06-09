//! Long-duration soak / endurance test for Dynamixel XL330/XL430 motors.
//!
//! Drives a sinusoidal goal position at a fixed control rate while sync-reading
//! present position every cycle, and monitors three things over many hours:
//!
//!   1. Memory leak        -- samples this process's own RSS each summary window.
//!   2. Communication errors -- buckets every failed sync_read/sync_write by
//!                              `CommunicationErrorKind` (Timeout / Checksum /
//!                              Parsing / IncorrectId / other). Never aborts on a
//!                              comm error: it counts it and keeps going.
//!   3. Jitter             -- streaming stats (Welford + a fixed-bucket histogram)
//!                              on loop-period error, read latency, and write
//!                              latency. All statistics are O(1) in memory so the
//!                              harness itself cannot masquerade as a leak.
//!
//! Output: one CSV row per summary window (to stdout and, optionally, a file),
//! plus a final cumulative report. SIGINT (Ctrl-C) shuts down gracefully and
//! disables torque before exiting.
//!
//! Example:
//!   cargo run --release --example soak_test -- \
//!       --serialport /dev/ttyUSB0 --ids 1,2,3 --rate-hz 100 \
//!       --amplitude-deg 20 --frequency 0.2 --csv soak.csv

use std::error::Error;
use std::f64::consts::PI;
use std::fs::File;
use std::io::Write;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use clap::Parser;
use signal_hook::flag;

use rustypot::servo::dynamixel::xl330::Xl330Controller;
use rustypot::CommunicationErrorKind;

#[derive(Parser, Debug)]
#[command(author, version, about = "XL330/XL430 endurance / soak test", long_about = None)]
struct Args {
    /// Serial port (e.g. /dev/ttyUSB0 or /dev/tty.usbmodemXXXX)
    #[arg(short, long)]
    serialport: String,

    /// Baudrate
    #[arg(short, long, default_value_t = 1_000_000)]
    baudrate: u32,

    /// Motor ids, comma separated (e.g. 1,2,3)
    #[arg(short, long, value_delimiter = ',')]
    ids: Vec<u8>,

    /// Control-loop rate in Hz
    #[arg(short, long, default_value_t = 100.0)]
    rate_hz: f64,

    /// Sine amplitude in degrees (peak, around center)
    #[arg(short, long, default_value_t = 20.0)]
    amplitude_deg: f64,

    /// Sine center in degrees
    #[arg(short, long, default_value_t = 0.0)]
    center_deg: f64,

    /// Sine frequency in Hz
    #[arg(short, long, default_value_t = 0.2)]
    frequency: f64,

    /// Total duration in seconds (0 = run until Ctrl-C)
    #[arg(short, long, default_value_t = 0)]
    duration_s: u64,

    /// Summary window in seconds
    #[arg(long, default_value_t = 10.0)]
    summary_s: f64,

    /// Read-only: never enable torque or write goal position (RX-path only)
    #[arg(long, default_value_t = false)]
    read_only: bool,

    /// Per-call serial timeout in milliseconds
    #[arg(long, default_value_t = 20)]
    timeout_ms: u64,

    /// Optional CSV output file (appends header + one row per window)
    #[arg(long)]
    csv: Option<String>,
}

/// Streaming statistics: mean/stddev via Welford, plus a fixed-bucket histogram
/// for approximate percentiles. Constant memory, no per-sample allocation.
struct Stats {
    count: u64,
    mean: f64,
    m2: f64,
    min: f64,
    max: f64,
    /// Buckets of `bucket_us` microseconds each; last bucket is overflow.
    hist: Vec<u64>,
    bucket_us: f64,
}

impl Stats {
    fn new() -> Self {
        // 2000 buckets * 50us = covers 0..100ms with 50us resolution; the final
        // bucket catches anything slower. Fixed size => O(1) memory.
        Stats {
            count: 0,
            mean: 0.0,
            m2: 0.0,
            min: f64::INFINITY,
            max: f64::NEG_INFINITY,
            hist: vec![0; 2001],
            bucket_us: 50.0,
        }
    }

    /// Record one sample, value in seconds.
    fn record(&mut self, secs: f64) {
        self.count += 1;
        let delta = secs - self.mean;
        self.mean += delta / self.count as f64;
        self.m2 += delta * (secs - self.mean);
        if secs < self.min {
            self.min = secs;
        }
        if secs > self.max {
            self.max = secs;
        }
        let us = secs * 1e6;
        let idx = (us / self.bucket_us) as usize;
        let idx = idx.min(self.hist.len() - 1);
        self.hist[idx] += 1;
    }

    fn stddev(&self) -> f64 {
        if self.count < 2 {
            0.0
        } else {
            (self.m2 / self.count as f64).sqrt()
        }
    }

    /// Approximate percentile (0.0..1.0), returned in seconds (bucket upper edge).
    fn percentile(&self, p: f64) -> f64 {
        if self.count == 0 {
            return 0.0;
        }
        let target = (p * self.count as f64).ceil() as u64;
        let mut cum = 0u64;
        for (i, &c) in self.hist.iter().enumerate() {
            cum += c;
            if cum >= target {
                return (i as f64 + 1.0) * self.bucket_us / 1e6;
            }
        }
        self.max
    }

    fn reset(&mut self) {
        self.count = 0;
        self.mean = 0.0;
        self.m2 = 0.0;
        self.min = f64::INFINITY;
        self.max = f64::NEG_INFINITY;
        for b in self.hist.iter_mut() {
            *b = 0;
        }
    }
}

/// Per-`CommunicationErrorKind` counters.
#[derive(Default, Clone, Copy)]
struct ErrCounts {
    timeout: u64,
    checksum: u64,
    parsing: u64,
    incorrect_id: u64,
    other: u64,
}

impl ErrCounts {
    fn total(&self) -> u64 {
        self.timeout + self.checksum + self.parsing + self.incorrect_id + self.other
    }

    fn record(&mut self, e: &(dyn Error + 'static)) {
        match e.downcast_ref::<CommunicationErrorKind>() {
            Some(CommunicationErrorKind::TimeoutError) => self.timeout += 1,
            Some(CommunicationErrorKind::ChecksumError) => self.checksum += 1,
            Some(CommunicationErrorKind::ParsingError) => self.parsing += 1,
            Some(CommunicationErrorKind::IncorrectId(_, _)) => self.incorrect_id += 1,
            _ => self.other += 1,
        }
    }
}

/// Current resident set size of this process, in kilobytes (0 if unavailable).
fn rss_kb() -> u64 {
    #[cfg(target_os = "linux")]
    {
        if let Ok(s) = std::fs::read_to_string("/proc/self/status") {
            for line in s.lines() {
                if let Some(rest) = line.strip_prefix("VmRSS:") {
                    return rest
                        .split_whitespace()
                        .next()
                        .and_then(|v| v.parse::<u64>().ok())
                        .unwrap_or(0);
                }
            }
        }
        0
    }
    #[cfg(not(target_os = "linux"))]
    {
        // macOS / BSD: ask ps for our own RSS (KB).
        let pid = std::process::id();
        std::process::Command::new("ps")
            .args(["-o", "rss=", "-p", &pid.to_string()])
            .output()
            .ok()
            .and_then(|o| String::from_utf8(o.stdout).ok())
            .and_then(|s| s.trim().parse::<u64>().ok())
            .unwrap_or(0)
    }
}

const CSV_HEADER: &str = "elapsed_s,iters,read_ops,write_ops,errs,err_rate,\
timeout,checksum,parsing,incorrect_id,other,max_consec_errs,\
read_mean_us,read_p50_us,read_p99_us,read_max_us,\
write_mean_us,write_p50_us,write_p99_us,write_max_us,\
period_err_mean_us,period_err_std_us,period_err_max_us,overruns,\
rss_kb,rss_delta_kb,temp_max_c,temp_mean_c";

fn main() -> Result<(), Box<dyn Error>> {
    env_logger::init();
    let args = Args::parse();

    if args.ids.is_empty() {
        return Err("no ids given; pass --ids 1,2,3".into());
    }

    let period = Duration::from_secs_f64(1.0 / args.rate_hz);
    let amplitude_rad = args.amplitude_deg.to_radians();
    let center_rad = args.center_deg.to_radians();

    println!("# rustypot soak test");
    println!("# port={} baud={} ids={:?}", args.serialport, args.baudrate, args.ids);
    println!(
        "# rate={} Hz  amp={}deg  center={}deg  freq={} Hz  read_only={}",
        args.rate_hz, args.amplitude_deg, args.center_deg, args.frequency, args.read_only
    );
    if args.duration_s == 0 {
        println!("# duration: until Ctrl-C");
    } else {
        println!("# duration: {} s", args.duration_s);
    }

    let term = Arc::new(AtomicBool::new(false));
    flag::register(signal_hook::consts::SIGINT, Arc::clone(&term))?;

    let serial_port = serialport::new(&args.serialport, args.baudrate)
        .timeout(Duration::from_millis(args.timeout_ms))
        .open()?;

    let mut c = Xl330Controller::new()
        .with_protocol_v2()
        .with_serial_port(serial_port);

    // Enable torque (unless read-only).
    if !args.read_only {
        for &id in &args.ids {
            if let Err(e) = c.write_torque_enable(id, true) {
                eprintln!("warning: torque enable failed for id {id}: {e}");
            }
        }
    }

    // CSV file (optional).
    let mut csv_file = match &args.csv {
        Some(path) => {
            let mut f = File::create(path)?;
            writeln!(f, "{CSV_HEADER}")?;
            Some(f)
        }
        None => None,
    };
    println!("{CSV_HEADER}");

    // Cumulative + windowed accumulators.
    let mut read_stats = Stats::new();
    let mut write_stats = Stats::new();
    let mut period_stats = Stats::new();
    let mut win_read = Stats::new();
    let mut win_write = Stats::new();
    let mut win_period = Stats::new();

    let mut errs = ErrCounts::default();
    let mut win_errs = ErrCounts::default();
    let mut iters: u64 = 0;
    let mut win_iters: u64 = 0;
    let mut read_ops: u64 = 0;
    let mut write_ops: u64 = 0;
    let mut consec_errs: u64 = 0;
    let mut max_consec: u64 = 0;
    let mut overruns: u64 = 0;
    let mut win_overruns: u64 = 0;
    // Thermal health (probed once per summary window, not per cycle).
    let mut last_temp_max: u8 = 0;
    let mut last_temp_mean: f64 = 0.0;
    let mut peak_temp: u8 = 0;

    // Reusable goal buffer -- allocate once, mutate in place (no per-cycle alloc).
    let mut goals: Vec<f64> = vec![center_rad; args.ids.len()];

    let rss0 = rss_kb();
    let start = Instant::now();
    let mut last_summary = start;
    let mut last_cycle = start;
    let summary_period = Duration::from_secs_f64(args.summary_s);

    let mut tick: u64 = 0;
    while !term.load(Ordering::Relaxed) {
        // Drift-free pacing: target = start + tick*period.
        let target_time = start + period * tick as u32;
        let now = Instant::now();
        if now < target_time {
            thread::sleep(target_time - now);
        } else if now - target_time > period {
            overruns += 1;
            win_overruns += 1;
        }
        tick += 1;

        let cycle_start = Instant::now();
        // Period error = how far the actual cycle start drifted from the target rate.
        let actual_period = cycle_start.duration_since(last_cycle).as_secs_f64();
        last_cycle = cycle_start;
        if iters > 0 {
            let perr = (actual_period - period.as_secs_f64()).abs();
            period_stats.record(perr);
            win_period.record(perr);
        }

        let t = cycle_start.duration_since(start).as_secs_f64();

        // --- Write goal position (sinusoid) ---
        if !args.read_only {
            let target = center_rad + amplitude_rad * (2.0 * PI * args.frequency * t).sin();
            for g in goals.iter_mut() {
                *g = target;
            }
            let w0 = Instant::now();
            let res = c.sync_write_goal_position(&args.ids, &goals);
            let dt = w0.elapsed().as_secs_f64();
            write_ops += 1;
            match res {
                Ok(()) => {
                    write_stats.record(dt);
                    win_write.record(dt);
                    consec_errs = 0;
                }
                Err(e) => {
                    errs.record(e.as_ref());
                    win_errs.record(e.as_ref());
                    consec_errs += 1;
                    max_consec = max_consec.max(consec_errs);
                }
            }
        }

        // --- Read present position ---
        let r0 = Instant::now();
        let res = c.sync_read_present_position(&args.ids);
        let dt = r0.elapsed().as_secs_f64();
        read_ops += 1;
        match res {
            Ok(_) => {
                read_stats.record(dt);
                win_read.record(dt);
                consec_errs = 0;
            }
            Err(e) => {
                errs.record(e.as_ref());
                win_errs.record(e.as_ref());
                consec_errs += 1;
                max_consec = max_consec.max(consec_errs);
            }
        }

        iters += 1;
        win_iters += 1;

        // --- Periodic summary ---
        if cycle_start.duration_since(last_summary) >= summary_period {
            let elapsed = cycle_start.duration_since(start).as_secs_f64();
            let rss = rss_kb();

            // Thermal probe: one extra sync_read per window (kept out of the
            // per-cycle latency/error stats so it can't skew jitter numbers).
            // On failure, keep the last known values rather than reporting 0.
            match c.sync_read_present_temperature(&args.ids) {
                Ok(temps) if !temps.is_empty() => {
                    last_temp_max = *temps.iter().max().unwrap();
                    last_temp_mean =
                        temps.iter().map(|&t| t as f64).sum::<f64>() / temps.len() as f64;
                    peak_temp = peak_temp.max(last_temp_max);
                }
                Ok(_) => {}
                Err(e) => eprintln!("# warning: temperature read failed: {e}"),
            }
            let win_total_ops = if args.read_only { win_iters } else { win_iters * 2 };
            let err_rate = if win_total_ops > 0 {
                win_errs.total() as f64 / win_total_ops as f64
            } else {
                0.0
            };
            let row = format!(
                "{:.1},{},{},{},{},{:.5},{},{},{},{},{},{},\
{:.1},{:.1},{:.1},{:.1},\
{:.1},{:.1},{:.1},{:.1},\
{:.1},{:.1},{:.1},{},\
{},{},{},{:.1}",
                elapsed,
                win_iters,
                win_read.count,
                win_write.count,
                win_errs.total(),
                err_rate,
                win_errs.timeout,
                win_errs.checksum,
                win_errs.parsing,
                win_errs.incorrect_id,
                win_errs.other,
                max_consec,
                win_read.mean * 1e6,
                win_read.percentile(0.50) * 1e6,
                win_read.percentile(0.99) * 1e6,
                win_read.max * 1e6,
                win_write.mean * 1e6,
                win_write.percentile(0.50) * 1e6,
                win_write.percentile(0.99) * 1e6,
                win_write.max * 1e6,
                win_period.mean * 1e6,
                win_period.stddev() * 1e6,
                win_period.max * 1e6,
                win_overruns,
                rss,
                rss as i64 - rss0 as i64,
                last_temp_max,
                last_temp_mean,
            );
            println!("{row}");
            if let Some(f) = csv_file.as_mut() {
                let _ = writeln!(f, "{row}");
                let _ = f.flush();
            }

            win_read.reset();
            win_write.reset();
            win_period.reset();
            win_errs = ErrCounts::default();
            win_iters = 0;
            win_overruns = 0;
            last_summary = cycle_start;
        }

        if args.duration_s != 0 && start.elapsed() >= Duration::from_secs(args.duration_s) {
            break;
        }
    }

    // --- Graceful shutdown ---
    println!("\n# shutting down...");
    if !args.read_only {
        for &id in &args.ids {
            if let Err(e) = c.write_torque_enable(id, false) {
                eprintln!("warning: torque disable failed for id {id}: {e}");
            }
        }
    }

    let elapsed = start.elapsed().as_secs_f64();
    let total_ops = read_ops + write_ops;
    let rss_end = rss_kb();
    println!("\n========== FINAL REPORT ==========");
    println!("duration            : {:.1} s ({:.2} h)", elapsed, elapsed / 3600.0);
    println!("iterations          : {iters}");
    println!("ops (read+write)    : {total_ops}  (read={read_ops} write={write_ops})");
    println!(
        "errors total        : {} ({:.4}%)",
        errs.total(),
        if total_ops > 0 { errs.total() as f64 / total_ops as f64 * 100.0 } else { 0.0 }
    );
    println!(
        "  timeout={} checksum={} parsing={} incorrect_id={} other={}",
        errs.timeout, errs.checksum, errs.parsing, errs.incorrect_id, errs.other
    );
    println!("max consecutive errs: {max_consec}");
    println!("overruns (> 1 period): {overruns}");
    println!(
        "read latency  us    : mean={:.1} p50={:.1} p99={:.1} max={:.1}",
        read_stats.mean * 1e6,
        read_stats.percentile(0.50) * 1e6,
        read_stats.percentile(0.99) * 1e6,
        read_stats.max * 1e6
    );
    if !args.read_only {
        println!(
            "write latency us    : mean={:.1} p50={:.1} p99={:.1} max={:.1}",
            write_stats.mean * 1e6,
            write_stats.percentile(0.50) * 1e6,
            write_stats.percentile(0.99) * 1e6,
            write_stats.max * 1e6
        );
    }
    println!(
        "period jitter us    : mean={:.1} std={:.1} max={:.1}",
        period_stats.mean * 1e6,
        period_stats.stddev() * 1e6,
        period_stats.max * 1e6
    );
    println!(
        "RSS                 : start={} kb  end={} kb  delta={} kb",
        rss0,
        rss_end,
        rss_end as i64 - rss0 as i64
    );
    println!(
        "temperature C       : peak={} last_max={} last_mean={:.1}",
        peak_temp, last_temp_max, last_temp_mean
    );
    println!("==================================");

    Ok(())
}
