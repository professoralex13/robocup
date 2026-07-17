use std::{
    io::{BufRead, BufReader, BufWriter, Read},
    sync::mpsc::{self, Sender, TryRecvError},
    thread,
    time::Duration,
};

use serialport::SerialPort;
use tungstenite::stream::MaybeTlsStream;
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
    WebSocket(Sender<Vec<u8>>),
    None,
}

pub fn initialize_data_source(
    data_source: DataSource,
) -> Result<CommandSink, Box<dyn std::error::Error>> {
    match data_source {
        DataSource::Serial(serial_port) => start_serial_mode(&serial_port),
        DataSource::WebSocket(url) => start_websocket_mode(&url),
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

fn start_websocket_mode(url: &str) -> Result<CommandSink, Box<dyn std::error::Error>> {
    let url = url.to_string();
    let (command_tx, command_rx) = mpsc::channel::<Vec<u8>>();

    thread::spawn(move || {
        let mut incoming_serial_stream = Vec::<u8>::with_capacity(16 * 1024);

        loop {
            match connect(url.as_str()) {
                Ok((mut websocket, _)) => {
                    if let MaybeTlsStream::Plain(stream) = websocket.get_mut() {
                        if let Err(error) = stream.set_nonblocking(true) {
                            eprintln!("Websocket nonblocking setup error: {error}");
                            thread::sleep(Duration::from_millis(500));
                            continue;
                        }
                    }

                    println!("Connected to telemetry bridge at {url}");

                    loop {
                        loop {
                            match command_rx.try_recv() {
                                Ok(payload) => {
                                    if let Err(error) =
                                        websocket.send(Message::Binary(payload.into()))
                                    {
                                        eprintln!("Websocket send error: {error}");
                                        break;
                                    }
                                }
                                Err(TryRecvError::Empty) => break,
                                Err(TryRecvError::Disconnected) => return,
                            }
                        }

                        match websocket.read() {
                            Ok(Message::Binary(payload)) => {
                                incoming_serial_stream.extend_from_slice(payload.as_slice());

                                while let Some(packet) =
                                    extract_telemetry_packet(&mut incoming_serial_stream)
                                {
                                    let mut lock = TELEMETRY.lock().unwrap();
                                    *lock = Some(packet);
                                }
                            }
                            Ok(Message::Close(_)) => {
                                eprintln!("Websocket closed by peer");
                                break;
                            }
                            Ok(_) => {}
                            Err(tungstenite::Error::Io(error))
                                if error.kind() == std::io::ErrorKind::WouldBlock => {}
                            Err(error) => {
                                eprintln!("Websocket receive error: {error}");
                                break;
                            }
                        }

                        thread::sleep(Duration::from_millis(5));
                    }
                }
                Err(error) => {
                    eprintln!("Websocket connect error: {error}");
                }
            }

            thread::sleep(Duration::from_millis(500));
        }
    });

    Ok(CommandSink::WebSocket(command_tx))
}

fn extract_telemetry_packet(stream: &mut Vec<u8>) -> Option<TelemetryPacket> {
    let header_len = TELEMETRY_HEADER.len();
    let payload_len = std::mem::size_of::<TelemetryPacket>();
    let frame_len = header_len + payload_len;

    let header_pos = stream
        .windows(header_len)
        .position(|window| window == TELEMETRY_HEADER.as_slice());

    let header_pos = match header_pos {
        Some(position) => position,
        None => {
            let keep_len = header_len.saturating_sub(1);
            if stream.len() > keep_len {
                let start = stream.len() - keep_len;
                stream.drain(..start);
            }
            return None;
        }
    };

    if header_pos > 0 {
        stream.drain(..header_pos);
    }

    if stream.len() < frame_len {
        return None;
    }

    let packet = TelemetryPacket::ref_from_bytes(&stream[header_len..frame_len])
        .ok()
        .copied();

    stream.drain(..frame_len);

    packet
}
