use log::warn;
use std::time::Duration;

use tokio::runtime::Runtime;
use tokio::sync::mpsc;
use tokio_stream::StreamExt;

use message::message_service_client::MessageServiceClient;
use message::InstructionPacket;

use crate::{CommunicationErrorKind, DynamixelLikeIO};

pub mod message {
    tonic::include_proto!("message");
}

type ByteResponse = Result<Vec<u8>, CommunicationErrorKind>;

pub struct DynamixelGrpcIO {
    rt: Runtime,

    out_tx: mpsc::Sender<Vec<u8>>,
    in_rx: mpsc::Receiver<ByteResponse>,
}

impl DynamixelGrpcIO {
    pub fn new(host: &str, port: u32) -> Self {
        let rt = Runtime::new().unwrap();

        let (out_tx, mut out_rx): (mpsc::Sender<Vec<u8>>, mpsc::Receiver<Vec<u8>>) =
            mpsc::channel(1);
        let (in_tx, in_rx): (mpsc::Sender<ByteResponse>, mpsc::Receiver<ByteResponse>) =
            mpsc::channel(1);

        let host = String::from(host);

        rt.spawn(async move {
            let url = format!("http://{}:{}", host, port);

            const MAX_RETRY: u32 = 5;
            let mut trial = 0;

            let mut client = loop {
                trial += 1;

                if trial == MAX_RETRY {
                    panic!("Could not connect to {url}, aborting!")
                }

                match MessageServiceClient::connect(url.clone()).await {
                    Ok(client) => break client,
                    Err(_) => {
                        warn!("Unable to connect to {url}! Will retry in 1s.");
                        tokio::time::sleep(Duration::from_secs(1)).await;
                    }
                }
            };

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
                let resp = match received {
                    Ok(status_packet) => match status_packet.response {
                        Some(resp) => match resp {
                            message::status_packet::Response::Packet(data) => Ok(data),
                            message::status_packet::Response::Error(_) => {
                                Err(CommunicationErrorKind::TimeoutError)
                            }
                        },
                        None => Err(CommunicationErrorKind::TimeoutError),
                    },
                    Err(_) => Err(CommunicationErrorKind::TimeoutError),
                };

                in_tx.send(resp).await.unwrap();
            }
        });

        DynamixelGrpcIO { rt, out_tx, in_rx }
    }
}

impl DynamixelLikeIO for DynamixelGrpcIO {
    fn send_packet(&mut self, bytes: Vec<u8>) {
        self.rt
            .block_on(async { self.out_tx.send(bytes).await })
            .unwrap();
    }

    fn read_packet(&mut self) -> ByteResponse {
        self.rt.block_on(async { self.in_rx.recv().await }).unwrap()
    }
}
