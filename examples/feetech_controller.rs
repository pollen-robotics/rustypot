use std::{error::Error, thread, time::Duration};

use rustypot::servo::feetech::sts3215::Sts3215Controller;
fn main() -> Result<(), Box<dyn Error>> {
    let serialportname: String = "/dev/tty.usbmodem58FA0822621".to_string();
    let baudrate: u32 = 1_000_000;
    let ids = vec![1, 2];

    let serial_port = serialport::new(serialportname, baudrate)
        .timeout(Duration::from_millis(1000))
        .open()?;
    println!("serial port opened");

    let mut c = Sts3215Controller::new()
        .with_protocol_v1()
        .with_serial_port(serial_port);

    let mut times: Vec<f64> = Vec::new();
    let duration = Duration::new(5, 0);
    let start_overall = std::time::Instant::now();

    while start_overall.elapsed() < duration {
        let start_time = std::time::Instant::now();
        let x = c.sync_read_present_position(&ids);
        let elapsed_time = start_time.elapsed();

        println!("present pos: {:?}", x);
        let elapsed_secs = elapsed_time.as_secs_f64();
        println!("Time taken to read position: {:?}", elapsed_secs);

        times.push(elapsed_secs);
        thread::sleep(Duration::from_millis(10));
    }

    let mean = times.iter().sum::<f64>() / times.len() as f64;
    let std_dev =
        (times.iter().map(|&t| (t - mean).powf(2.0)).sum::<f64>() / times.len() as f64).sqrt();

    println!("Mean time: {}", mean);
    println!("Standard deviation time: {}", std_dev);

    Ok(())
}
