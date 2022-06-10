use tokio::runtime::Runtime;
use tokio::sync::mpsc;
use tokio_stream::StreamExt;

use message::message_service_client::MessageServiceClient;
use message::InstructionPacket;

use crate::DynamixelLikeIO;

pub mod message {
    tonic::include_proto!("message");
}

pub struct DynamixelGrpcIO {
    rt: Runtime,

    out_tx: mpsc::Sender<Vec<u8>>,
    in_rx: mpsc::Receiver<Vec<u8>>,
}

impl DynamixelGrpcIO {
    pub fn new(host: &str, port: u32) -> Self {
        let rt = Runtime::new().unwrap();

        let (out_tx, mut out_rx): (mpsc::Sender<Vec<u8>>, mpsc::Receiver<Vec<u8>>) = mpsc::channel(1);
        let (in_tx, in_rx): (mpsc::Sender<Vec<u8>>, mpsc::Receiver<Vec<u8>>) = mpsc::channel(1);

        let host = String::from(host);

        rt.spawn(async move {
            let url = format!("http://{}:{}", host, port);
            let mut client = MessageServiceClient::connect(url).await.unwrap();

            let outbound = async_stream::stream! {
                loop {
                    let data = out_rx.recv().await.unwrap();
                    let packet = InstructionPacket { packet: data };
                    yield packet;
                }
            };

            let response = client.send(tonic::Request::new(outbound)).await.unwrap();
            let mut resp_stream = response.into_inner();
        
            while let Some(received) = resp_stream.next().await {
                let received = received.unwrap();

                if let Some(x) = received.response {
                    match x {
                        message::status_packet::Response::Packet(data) => in_tx.send(data).await.unwrap(),
                        message::status_packet::Response::Error(e) => panic!("TIMEOUT {:?}", e),
                    }
                }
            }
        });

        DynamixelGrpcIO { rt: rt, out_tx: out_tx, in_rx: in_rx }
    }
}

impl DynamixelLikeIO for DynamixelGrpcIO {
    fn send_packet(&self, bytes: Vec<u8>) {
        self.rt.block_on(async {
            self.out_tx.send(bytes).await
        }).unwrap();
    }

    fn read_packet(&mut self) -> Vec<u8> {
        self.rt.block_on(async {
            self.in_rx.recv().await
        }).unwrap()
    }
}