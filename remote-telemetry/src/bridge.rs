use std::{
    io::{Read, Write},
    net::{TcpListener, TcpStream},
    time::Duration,
};

use serialport::SerialPort;
use tungstenite::{Message, accept};

pub fn run_bridge(serial_port: &str, listen_addr: &str) -> Result<(), Box<dyn std::error::Error>> {
    let port = serialport::new(serial_port, 921600)
        .timeout(Duration::from_millis(20))
        .open()?;
    let write_port = port.try_clone().expect("Failed to clone port");

    let mut reader = port;
    let mut writer = write_port;

    let listener = TcpListener::bind(listen_addr)?;
    println!("Telemetry bridge listening on ws://{listen_addr}");

    loop {
        let (stream, addr) = listener.accept()?;
        println!("Client connected: {addr}");

        if let Err(error) = stream_serial_to_client(stream, &mut reader, &mut writer) {
            eprintln!("Client disconnected: {error}");
        }
    }
}

fn stream_serial_to_client(
    stream: TcpStream,
    reader: &mut Box<dyn SerialPort>,
    writer: &mut Box<dyn SerialPort>,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut websocket = accept(stream)?;
    websocket.get_mut().set_nonblocking(true)?;
    let mut serial_buf = [0u8; 4096];

    loop {
        loop {
            match websocket.read() {
                Ok(Message::Binary(payload)) => {
                    if !payload.is_empty() {
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

        match reader.read(&mut serial_buf) {
            Ok(0) => {}
            Ok(read_count) => {
                websocket.send(Message::Binary(serial_buf[..read_count].to_vec().into()))?;
            }
            Err(error)
                if error.kind() == std::io::ErrorKind::TimedOut
                    || error.kind() == std::io::ErrorKind::WouldBlock => {}
            Err(error) => return Err(Box::new(error)),
        }
    }
}
