use std::{error::Error, thread, time::Duration};

use rustypot::{device::feetech_STS3215, DynamixelSerialIO};
fn main() -> Result<(), Box<dyn Error>> {
    let serialportname: String = "/dev/tty.usbmodem58FD0164681".to_string();
    let baudrate: u32 = 1_000_000;
    let ids = vec![1];

    let mut serial_port = serialport::new(serialportname, baudrate)
        .timeout(Duration::from_millis(1000))
        .open()?;
    println!("serial port opened");

    let io = DynamixelSerialIO::feetech();

    let mut times: Vec<f64> = Vec::new();
    let duration = Duration::new(5, 0);
    let start_overall = std::time::Instant::now();

    while start_overall.elapsed() < duration {
        let start_time = std::time::Instant::now();

        // let x: i16 = feetech_STS3215::read_present_position(&io, serial_port.as_mut(), ids[0])?;
        let x = feetech_STS3215::sync_read_present_position(&io, serial_port.as_mut(), &ids)?;
        let x: Vec<f64> = x
            .iter()
            .map(|p| feetech_STS3215::conv::dxl_pos_to_radians(*p))
            .map(f64::to_degrees)
            .collect();
        println!("present pos: {:?}", x);

        let elapsed_time = start_time.elapsed();
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
