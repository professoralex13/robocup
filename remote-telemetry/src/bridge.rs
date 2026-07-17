use futures_util::{SinkExt, StreamExt};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::{TcpListener, TcpStream},
    select,
};
use tokio_serial::SerialPortBuilderExt;
use tokio_tungstenite::{accept_async, tungstenite::Message};

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
        let (stream, addr) = listener.accept().await?;
        println!("Client connected: {addr}");

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
    let serial = tokio_serial::new(serial_port, 921600).open_native_async()?;

    let (mut websocket_writer, mut websocket_reader) = websocket.split();
    let (mut serial_reader, mut serial_writer) = tokio::io::split(serial);

    let mut serial_buf = [0u8; 4096];

    loop {
        select! {
            message = websocket_reader.next() => {
                match message {
                    Some(Ok(Message::Binary(payload))) => {
                        if !payload.is_empty() {
                            serial_writer.write_all(payload.as_ref()).await?;
                            serial_writer.flush().await?;
                        }
                    }
                    Some(Ok(Message::Close(_))) => return Ok(()),
                    Some(Ok(_)) => {}
                    Some(Err(error)) => return Err(Box::new(error)),
                    None => return Ok(()),
                }
            }
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
