use std::{
    env,
    f32::consts::PI,
    io::{BufRead, BufReader, BufWriter, Read, Write},
    net::TcpListener,
    net::TcpStream,
    thread,
    time::Duration,
};

use std::sync::Mutex;

use raylib::prelude::*;
use serialport::SerialPort;
use tungstenite::{Message, accept, connect};
use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout};

const WIDTH: i32 = 1280;
const HEIGHT: i32 = 720;

#[repr(C, packed)]
#[derive(Copy, Clone, IntoBytes, FromBytes, Debug, Immutable)]
struct LidarPoint {
    x: i16,
    y: i16,
    intensity: u8,
}

#[repr(C, packed)]
#[derive(Copy, Clone, IntoBytes, FromBytes, Debug, KnownLayout, Immutable)]
struct TelemetryPacket {
    points: [LidarPoint; 500],
    heading: f32,
    pitch: f32,

    left_wheel_velocity: f32,
    right_wheel_velocity: f32,

    position_x: f32,
    position_y: f32,
}

#[repr(C, packed)]
#[derive(Copy, Clone, FromBytes, IntoBytes, Debug, KnownLayout, Immutable)]
struct CommandPacket {
    left_command: i8,
    right_command: i8,
}

const TELEMETRY_HEADER: [u8; 8] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
const COMMAND_HEADER: [u8; 10] = [0x99, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00];

static TELEMETRY: Mutex<Option<TelemetryPacket>> = Mutex::new(None);

enum DataSource {
    Serial(String),
    WebSocket(String),
}

enum CommandSink {
    Serial(BufWriter<Box<dyn SerialPort>>),
    None,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    let serial_port = get_arg_value(&args, "--serial").unwrap_or("/dev/ttyACM0".to_string());

    if has_flag(&args, "--bridge") {
        let listen_addr = get_arg_value(&args, "--listen").unwrap_or("0.0.0.0:9002".to_string());
        return run_bridge(&serial_port, &listen_addr);
    }

    let data_source = match get_arg_value(&args, "--ws") {
        Some(url) => DataSource::WebSocket(url),
        None => DataSource::Serial(serial_port),
    };

    let mut command_sink = initialize_data_source(data_source)?;
    run_ui(&mut command_sink)
}

fn has_flag(args: &[String], flag: &str) -> bool {
    args.iter().any(|arg| arg == flag)
}

fn get_arg_value(args: &[String], name: &str) -> Option<String> {
    args.windows(2)
        .find(|window| window[0] == name)
        .map(|window| window[1].clone())
}

fn initialize_data_source(
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

fn read_telemetry_packet(
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

fn run_ui(command_sink: &mut CommandSink) -> Result<(), Box<dyn std::error::Error>> {
    let (mut rl, thread) = raylib::init()
        .size(WIDTH, HEIGHT)
        .title("Hello, World")
        .build();

    rl.set_target_fps(30);

    while !rl.window_should_close() {
        let mut y = 0;
        if rl.is_key_down(KeyboardKey::KEY_W) {
            y = 1;
        } else if rl.is_key_down(KeyboardKey::KEY_S) {
            y = -1;
        }

        let mut x = 0;

        if rl.is_key_down(KeyboardKey::KEY_A) {
            x = -1;
        } else if rl.is_key_down(KeyboardKey::KEY_D) {
            x = 1;
        }

        let left_command = (y + x).clamp(-1, 1) * 100;
        let right_command = (y - x).clamp(-1, 1) * 100;

        let packet = CommandPacket {
            left_command,
            right_command,
        };

        if let CommandSink::Serial(writer) = command_sink {
            writer.write_all(&COMMAND_HEADER)?;
            writer.write_all(packet.as_bytes())?;
            writer.flush()?;
        }

        let mut d = rl.begin_drawing(&thread);

        d.clear_background(Color::WHITE);

        d.draw_text(
            format!("Left: {left_command}, Right: {right_command}").as_str(),
            20,
            60,
            20,
            Color::BLACK,
        );

        if let Some(telemetry) = TELEMETRY.lock().unwrap().clone() {
            for point in telemetry.points {
                d.draw_circle(
                    (point.x as i32) / 4 + WIDTH / 2,
                    -(point.y as i32) / 4 + HEIGHT / 2,
                    2.0,
                    Color::color_from_hsv(360.0 / 256.0 * point.intensity as f32, 1.0, 1.0),
                );
            }

            d.draw_text(
                format!(
                    "Heading: {:.2}°, Pitch: {:.2}°",
                    telemetry.heading as f64 * RAD2DEG,
                    telemetry.pitch as f64 * RAD2DEG,
                )
                .as_str(),
                20,
                20,
                20,
                Color::BLACK,
            );

            d.draw_text(
                format!(
                    "Left: {:.1}rpm, Right: {:.1}rpm",
                    60.0 * telemetry.left_wheel_velocity / (2.0 * PI),
                    60.0 * telemetry.right_wheel_velocity / (2.0 * PI)
                )
                .as_str(),
                20,
                40,
                20,
                Color::BLACK,
            );

            d.draw_text(
                format!(
                    "X: {:.1}nn, Y: {:.1}mm",
                    1.0 * telemetry.position_x,
                    1.0 * telemetry.position_y,
                )
                .as_str(),
                20,
                80,
                20,
                Color::BLACK,
            );
        } else {
            d.draw_text("No Telemetry", 20, 20, 20, Color::BLACK);
        }
    }

    Ok(())
}

fn run_bridge(serial_port: &str, listen_addr: &str) -> Result<(), Box<dyn std::error::Error>> {
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
