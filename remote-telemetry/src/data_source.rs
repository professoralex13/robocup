use std::{
    io::{BufWriter, Read},
    sync::mpsc::{self, Sender, TryRecvError},
    thread,
    time::Duration,
};

use crate::{
    protocol::{
        TELEMETRY_FRAME_HEADER_LEN, TELEMETRY_PREAMBLE, TELEMETRY_VERSION, TelemetryFrame,
        parse_frame,
    },
    telemetry_state::{TELEMETRY, TypedValue},
};
use serialport::SerialPort;
use tungstenite::stream::MaybeTlsStream;
use tungstenite::{Message, connect};

const MAX_INCOMING_STREAM_BYTES: usize = 256 * 1024;

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

fn start_serial_mode(serial_port: &str) -> Result<CommandSink, Box<dyn std::error::Error>> {
    let port = serialport::new(serial_port, 921600)
        .timeout(Duration::from_millis(20))
        .open()?;

    let write_port = port.try_clone().expect("Failed to clone port");
    let mut reader = port;

    thread::spawn(move || {
        let mut incoming_stream = Vec::<u8>::with_capacity(32 * 1024);
        let mut read_buf = [0u8; 4096];

        loop {
            match reader.read(&mut read_buf) {
                Ok(0) => {}
                Ok(read_count) => {
                    incoming_stream.extend_from_slice(&read_buf[..read_count]);
                    consume_stream_frames(&mut incoming_stream);
                }
                Err(error) => {
                    if error.kind() != std::io::ErrorKind::TimedOut
                        && error.kind() != std::io::ErrorKind::WouldBlock
                    {
                        eprintln!("Serial read error: {error}");
                        thread::sleep(Duration::from_millis(100));
                    }
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
                        let mut should_reconnect = false;
                        let mut had_receive_data = false;

                        loop {
                            match command_rx.try_recv() {
                                Ok(payload) => {
                                    if let Err(error) =
                                        websocket.send(Message::Binary(payload.into()))
                                    {
                                        eprintln!("Websocket send error: {error}");
                                        should_reconnect = true;
                                        break;
                                    }
                                }
                                Err(TryRecvError::Empty) => break,
                                Err(TryRecvError::Disconnected) => return,
                            }
                        }

                        if should_reconnect {
                            break;
                        }

                        loop {
                            match websocket.read() {
                                Ok(Message::Binary(payload)) => {
                                    had_receive_data = true;
                                    incoming_serial_stream.extend_from_slice(payload.as_slice());

                                    if incoming_serial_stream.len() > MAX_INCOMING_STREAM_BYTES {
                                        let keep =
                                            TELEMETRY_FRAME_HEADER_LEN + TELEMETRY_PREAMBLE.len();
                                        let drop_len =
                                            incoming_serial_stream.len().saturating_sub(keep);
                                        incoming_serial_stream.drain(..drop_len);
                                    }

                                    consume_stream_frames(&mut incoming_serial_stream);
                                }
                                Ok(Message::Close(_)) => {
                                    eprintln!("Websocket closed by peer");
                                    should_reconnect = true;
                                    break;
                                }
                                Ok(_) => {}
                                Err(tungstenite::Error::Io(error))
                                    if error.kind() == std::io::ErrorKind::WouldBlock =>
                                {
                                    break;
                                }
                                Err(error) => {
                                    eprintln!("Websocket receive error: {error}");
                                    should_reconnect = true;
                                    break;
                                }
                            }
                        }

                        if should_reconnect {
                            break;
                        }

                        if !had_receive_data {
                            thread::sleep(Duration::from_millis(1));
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

    Ok(CommandSink::WebSocket(command_tx))
}

fn consume_stream_frames(stream: &mut Vec<u8>) {
    while let Some(frame) = extract_telemetry_frame(stream) {
        apply_frame(frame);
    }
}

fn extract_telemetry_frame(stream: &mut Vec<u8>) -> Option<TelemetryFrame> {
    let preamble_len = TELEMETRY_PREAMBLE.len();

    let preamble_pos = stream
        .windows(preamble_len)
        .position(|window| window == TELEMETRY_PREAMBLE.as_slice());

    let preamble_pos = match preamble_pos {
        Some(position) => position,
        None => {
            let keep_len = preamble_len.saturating_sub(1);
            if stream.len() > keep_len {
                let start = stream.len() - keep_len;
                stream.drain(..start);
            }
            return None;
        }
    };

    if preamble_pos > 0 {
        stream.drain(..preamble_pos);
    }

    if stream.len() < TELEMETRY_FRAME_HEADER_LEN {
        return None;
    }

    let frame_type = stream[4];
    let version = stream[5];

    if version != TELEMETRY_VERSION {
        stream.drain(..1);
        return None;
    }

    let payload_len = u16::from_le_bytes([stream[6], stream[7]]) as usize;
    let frame_len = TELEMETRY_FRAME_HEADER_LEN + payload_len;

    if stream.len() < frame_len {
        return None;
    }

    let frame = parse_frame(frame_type, &stream[TELEMETRY_FRAME_HEADER_LEN..frame_len]);
    stream.drain(..frame_len);
    frame
}

fn apply_frame(frame: TelemetryFrame) {
    let mut telemetry = TELEMETRY.lock().unwrap();

    match frame {
        TelemetryFrame::Values(entries) => {
            for entry in entries {
                telemetry.values.insert(
                    entry.key,
                    TypedValue {
                        value_type: entry.value_type,
                        payload: entry.payload,
                    },
                );
            }
        }
        TelemetryFrame::Lidar(points) => {
            for point in points {
                if telemetry.lidar_points.len() >= 1500 {
                    telemetry.lidar_points.pop_front();
                }
                telemetry.lidar_points.push_back(point);
            }
        }
    }
}
