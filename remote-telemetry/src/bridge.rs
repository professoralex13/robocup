use futures_util::{SinkExt, StreamExt};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::{TcpListener, TcpStream},
    select,
};
use tokio_serial::SerialPortBuilderExt;
use tokio_tungstenite::{accept_async, tungstenite::Message};

use crate::BAUD_RATE;

/// Starts the program in bridge mode, connecting a given serial port to a websocket at a given address
pub fn run_bridge(serial_port: &str, listen_addr: &str) -> Result<(), Box<dyn std::error::Error>> {
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?;

    runtime.block_on(run_bridge_async(serial_port, listen_addr))
}

async fn run_bridge_async(
    serial_port: &str,
    listen_addr: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let listener = TcpListener::bind(listen_addr).await?;

    println!("Telemetry bridge listening on ws://{listen_addr}");

    loop {
        // Wait for a client to connect
        let (stream, addr) = listener.accept().await?;

        println!("Client connected: {addr}");

        // Once a client is connected, stream data both ways until the client disconnects, then wait again for another connection

        if let Err(error) = stream_serial_to_client(stream, serial_port).await {
            eprintln!("Client disconnected: {error}");
        }
    }
}

async fn stream_serial_to_client(
    stream: TcpStream,
    serial_port: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let websocket = accept_async(stream).await?;
    let serial = tokio_serial::new(serial_port, BAUD_RATE).open_native_async()?;

    let (mut websocket_writer, mut websocket_reader) = websocket.split();
    let (mut serial_reader, mut serial_writer) = tokio::io::split(serial);

    let mut serial_buf = [0u8; 4096];

    loop {
        select! {
            // Handle the websocket receiving a message by sending it to the serial port
            websocket_message = websocket_reader.next() => {
                match websocket_message.ok_or("No Message")?? {
                    Message::Binary(payload) => {
                        if !payload.is_empty() {
                            serial_writer.write_all(payload.as_ref()).await?;

                            serial_writer.flush().await?;
                        }
                    },
                    Message::Close(_) => return Ok(()), // If the message is requesting closer, we exit
                    _ => {}, // Do nothing on any other message type
                }
            }

            // Handle the serial port receiving content by sending it to the websocket
            serial_result = serial_reader.read(&mut serial_buf) => {
                let read_count = serial_result?;

                if read_count > 0 {
                    websocket_writer
                        .send(Message::Binary(serial_buf[..read_count].to_vec().into()))
                        .await?;
                }
            }
        }
    }
}
