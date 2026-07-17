use std::{
    io::{BufReader, BufWriter, Write},
    net::{TcpListener, TcpStream},
    time::Duration,
};

use serialport::SerialPort;
use tungstenite::{Message, accept};
use zerocopy::IntoBytes;

use crate::{
    data_source::read_telemetry_packet,
    protocol::{COMMAND_HEADER, CommandPacket},
};

pub fn run_bridge(serial_port: &str, listen_addr: &str) -> Result<(), Box<dyn std::error::Error>> {
    let port = serialport::new(serial_port, 921600)
        .timeout(Duration::from_secs(100))
        .open()?;
    let write_port = port.try_clone().expect("Failed to clone port");

    let mut reader = BufReader::new(port);
    let mut writer = BufWriter::new(write_port);

    let listener = TcpListener::bind(listen_addr)?;
    println!("Telemetry bridge listening on ws://{listen_addr}");

    loop {
        let (stream, addr) = listener.accept()?;
        println!("Client connected: {addr}");

        if let Err(error) = stream_telemetry_to_client(stream, &mut reader, &mut writer) {
            eprintln!("Client disconnected: {error}");
        }
    }
}

fn stream_telemetry_to_client(
    stream: TcpStream,
    reader: &mut BufReader<Box<dyn SerialPort>>,
    writer: &mut BufWriter<Box<dyn SerialPort>>,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut websocket = accept(stream)?;
    websocket.get_mut().set_nonblocking(true)?;

    loop {
        loop {
            match websocket.read() {
                Ok(Message::Binary(payload)) => {
                    if payload.len() == COMMAND_HEADER.len() + std::mem::size_of::<CommandPacket>()
                        && payload.starts_with(&COMMAND_HEADER)
                    {
                        writer.write_all(payload.as_slice())?;
                        writer.flush()?;
                    } else if payload.len() == std::mem::size_of::<CommandPacket>() {
                        writer.write_all(&COMMAND_HEADER)?;
                        writer.write_all(payload.as_slice())?;
                        writer.flush()?;
                    }
                }
                Ok(Message::Close(_)) => return Ok(()),
                Ok(_) => {}
                Err(tungstenite::Error::Io(error))
                    if error.kind() == std::io::ErrorKind::WouldBlock =>
                {
                    break;
                }
                Err(error) => return Err(Box::new(error)),
            }
        }

        let telemetry = read_telemetry_packet(reader)?;
        websocket.send(Message::Binary(telemetry.as_bytes().to_vec()))?;
    }
}
