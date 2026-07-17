use std::{
    io::BufReader,
    net::{TcpListener, TcpStream},
    time::Duration,
};

use serialport::SerialPort;
use tungstenite::{Message, accept};
use zerocopy::IntoBytes;

use crate::data_source::read_telemetry_packet;

pub fn run_bridge(serial_port: &str, listen_addr: &str) -> Result<(), Box<dyn std::error::Error>> {
    let port = serialport::new(serial_port, 921600)
        .timeout(Duration::from_secs(100))
        .open()?;
    let mut reader = BufReader::new(port);

    let listener = TcpListener::bind(listen_addr)?;
    println!("Telemetry bridge listening on ws://{listen_addr}");

    loop {
        let (stream, addr) = listener.accept()?;
        println!("Client connected: {addr}");

        if let Err(error) = stream_telemetry_to_client(stream, &mut reader) {
            eprintln!("Client disconnected: {error}");
        }
    }
}

fn stream_telemetry_to_client(
    stream: TcpStream,
    reader: &mut BufReader<Box<dyn SerialPort>>,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut websocket = accept(stream)?;

    loop {
        let telemetry = read_telemetry_packet(reader)?;
        websocket.send(Message::Binary(telemetry.as_bytes().to_vec()))?;
    }
}
