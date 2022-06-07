use rustypot::DynamixelLikeIO;
use rustypot::grpc_io::DynamixelGrpcIO;

use std::time::Instant;


fn main() {
    let mut dxl_io = DynamixelGrpcIO::new("192.168.1.40", 38745);

    let start = Instant::now();

    for _ in 0..1000 {
        dxl_io.send_packet(vec![255, 255, 1, 2, 1, 251]);
        let _resp = dxl_io.read_packet();
    }
    let duration = start.elapsed();
    println!("Time elapsed: {:?}", duration);
}
