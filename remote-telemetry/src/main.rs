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
const FIELD_WIDTH_X_METERS: f32 = 2.4;
const FIELD_HEIGHT_Y_METERS: f32 = 4.9;
const VIEW_MARGIN_PX: f32 = 60.0;

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
    position_uncertainty: f32,
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

        let screen_w = WIDTH as f32;
        let screen_h = HEIGHT as f32;

        let usable_w = (screen_w - 2.0 * VIEW_MARGIN_PX).max(1.0);
        let usable_h = (screen_h - 2.0 * VIEW_MARGIN_PX).max(1.0);

        let pixels_per_meter =
            (usable_w / FIELD_WIDTH_X_METERS).min(usable_h / FIELD_HEIGHT_Y_METERS);

        let field_px_w = FIELD_WIDTH_X_METERS * pixels_per_meter;
        let field_px_h = FIELD_HEIGHT_Y_METERS * pixels_per_meter;

        let field_origin_x = (screen_w - field_px_w) * 0.5;
        let field_origin_y = (screen_h + field_px_h) * 0.5;

        let world_to_screen = |x_m: f32, y_m: f32| -> Vector2 {
            Vector2::new(
                field_origin_x + x_m * pixels_per_meter,
                field_origin_y - y_m * pixels_per_meter,
            )
        };

        let bl = world_to_screen(0.0, 0.0);
        let br = world_to_screen(FIELD_WIDTH_X_METERS, 0.0);
        let tr = world_to_screen(FIELD_WIDTH_X_METERS, FIELD_HEIGHT_Y_METERS);
        let tl = world_to_screen(0.0, FIELD_HEIGHT_Y_METERS);

        d.draw_line_ex(bl, br, 2.0, Color::BLACK);
        d.draw_line_ex(br, tr, 2.0, Color::BLACK);
        d.draw_line_ex(tr, tl, 2.0, Color::BLACK);
        d.draw_line_ex(tl, bl, 2.0, Color::BLACK);

        if let Some(telemetry) = TELEMETRY.lock().unwrap().clone() {
            let heading = telemetry.heading;
            let heading_sin = heading.sin();
            let heading_cos = heading.cos();
            let pitch = telemetry.pitch;
            let left_wheel_velocity = telemetry.left_wheel_velocity;
            let right_wheel_velocity = telemetry.right_wheel_velocity;
            let position_x = telemetry.position_x;
            let position_y = telemetry.position_y;
            let position_uncertainty = telemetry.position_uncertainty;

            // Heading convention used everywhere in this viewer:
            // 0 rad means robot-forward points toward +Y on the field.
            // Positive heading is clockwise, like a compass.
            let robot_frame_to_world = |robot_x_m: f32, robot_y_m: f32| -> (f32, f32) {
                // Compass-style clockwise-positive rotation.
                let world_x = position_x + heading_cos * robot_x_m + heading_sin * robot_y_m;
                let world_y = position_y - heading_sin * robot_x_m + heading_cos * robot_y_m;

                (world_x, world_y)
            };

            for point in telemetry.points {
                let lidar_x_robot_m = point.x as f32 / 1000.0;
                let lidar_y_robot_m = point.y as f32 / 1000.0;

                let (lidar_x_world, lidar_y_world) =
                    robot_frame_to_world(lidar_x_robot_m, lidar_y_robot_m);

                let screen = world_to_screen(lidar_x_world, lidar_y_world);

                d.draw_circle_v(
                    screen,
                    2.0,
                    Color::color_from_hsv(360.0 / 256.0 * point.intensity as f32, 1.0, 1.0),
                );
            }

            let robot_screen = world_to_screen(position_x, position_y);

            let uncertainty_m = position_uncertainty.max(0.01);
            let uncertainty_radius_px = uncertainty_m * pixels_per_meter;
            d.draw_circle_lines_v(robot_screen, uncertainty_radius_px, Color::RED);

            let heading_length_m = 0.22f32;
            let (heading_end_x, heading_end_y) = robot_frame_to_world(0.0, heading_length_m);
            let heading_end = world_to_screen(heading_end_x, heading_end_y);
            d.draw_line_ex(robot_screen, heading_end, 3.0, Color::BLUE);

            d.draw_text(
                format!(
                    "Heading: {:.2}°, Pitch: {:.2}°",
                    heading as f64 * RAD2DEG,
                    pitch as f64 * RAD2DEG,
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
                    60.0 * left_wheel_velocity / (2.0 * PI),
                    60.0 * right_wheel_velocity / (2.0 * PI)
                )
                .as_str(),
                20,
                40,
                20,
                Color::BLACK,
            );

            d.draw_text(
                format!("X: {:.3}m, Y: {:.3}m", 1.0 * position_x, 1.0 * position_y,).as_str(),
                20,
                80,
                20,
                Color::BLACK,
            );

            d.draw_text(
                format!("Position Uncertainty: {:.3}m", position_uncertainty).as_str(),
                20,
                100,
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
