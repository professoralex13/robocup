use std::{
    io::{BufRead, BufReader, BufWriter, Read},
    thread,
    time::Duration,
};

use serialport::SerialPort;
use tungstenite::{Message, connect};
use zerocopy::FromBytes;

use crate::{
    protocol::{TELEMETRY_HEADER, TelemetryPacket},
    telemetry_state::TELEMETRY,
};

pub enum DataSource {
    Serial(String),
    WebSocket(String),
}

pub enum CommandSink {
    Serial(BufWriter<Box<dyn SerialPort>>),
    None,
}

pub fn initialize_data_source(
    data_source: DataSource,
) -> Result<CommandSink, Box<dyn std::error::Error>> {
    match data_source {
        DataSource::Serial(serial_port) => start_serial_mode(&serial_port),
        DataSource::WebSocket(url) => {
            start_websocket_mode(&url)?;
            Ok(CommandSink::None)
        }
    }
}

pub fn read_telemetry_packet(
    reader: &mut BufReader<Box<dyn SerialPort>>,
) -> Result<TelemetryPacket, Box<dyn std::error::Error>> {
    let mut buf = [0u8; std::mem::size_of::<TelemetryPacket>()];

    for x in TELEMETRY_HEADER {
        reader.skip_until(x)?;
    }

    reader.read_exact(&mut buf)?;

    let telemetry = TelemetryPacket::read_from_bytes(&buf).unwrap();

    Ok(telemetry)
}

fn start_serial_mode(serial_port: &str) -> Result<CommandSink, Box<dyn std::error::Error>> {
    let port = serialport::new(serial_port, 921600)
        .timeout(Duration::from_secs(100))
        .open()?;

    let write_port = port.try_clone().expect("Failed to clone port");
    let mut reader = BufReader::new(port);

    thread::spawn(move || {
        loop {
            match read_telemetry_packet(&mut reader) {
                Ok(telemetry) => {
                    let mut lock = TELEMETRY.lock().unwrap();
                    *lock = Some(telemetry);
                }
                Err(error) => {
                    eprintln!("Serial read error: {error}");
                    thread::sleep(Duration::from_millis(100));
                }
            }
        }
    });

    Ok(CommandSink::Serial(BufWriter::new(write_port)))
}

fn start_websocket_mode(url: &str) -> Result<(), Box<dyn std::error::Error>> {
    let url = url.to_string();

    thread::spawn(move || {
        loop {
            match connect(url.as_str()) {
                Ok((mut websocket, _)) => {
                    println!("Connected to telemetry bridge at {url}");

                    loop {
                        match websocket.read() {
                            Ok(Message::Binary(payload)) => {
                                if payload.len() == std::mem::size_of::<TelemetryPacket>() {
                                    if let Ok(packet) =
                                        TelemetryPacket::ref_from_bytes(payload.as_slice())
                                    {
                                        let mut lock = TELEMETRY.lock().unwrap();
                                        *lock = Some(*packet);
                                    }
                                }
                            }
                            Ok(_) => {}
                            Err(error) => {
                                eprintln!("Websocket receive error: {error}");
                                break;
                            }
                        }
                    }
                }
                Err(error) => {
                    eprintln!("Websocket connect error: {error}");
                }
            }

            thread::sleep(Duration::from_millis(500));
        }
    });

    Ok(())
}