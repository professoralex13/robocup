use std::{
    f32::consts::PI,
    io::{BufRead, BufReader, BufWriter, Read, Write},
    thread,
    time::Duration,
};

use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout};

use firmware::{TELEMETRY_HEADER, TelemetryPacket};

use std::sync::Mutex;

use raylib::prelude::*;

const WIDTH: i32 = 1280;
const HEIGHT: i32 = 720;

#[repr(C, packed)]
#[derive(Copy, Clone, IntoBytes, Debug, KnownLayout, Immutable)]
struct CommandPacket {
    left_command: i8,
    right_command: i8,
}

const COMMAND_HEADER: [u8; 10] = [0x99, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00];

static TELEMETRY: Mutex<Option<TelemetryPacket>> = Mutex::new(None);

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let port = serialport::new("/dev/ttyACM1", 115200)
        .timeout(Duration::from_secs(100))
        .open()
        .unwrap();

    let write_port = port.try_clone().expect("Failed to clone port");

    let mut reader = BufReader::new(port);
    let mut writer = BufWriter::new(write_port);

    thread::spawn(move || {
        loop {
            let mut buf = [0u8; std::mem::size_of::<TelemetryPacket>()];

            for x in TELEMETRY_HEADER {
                reader.skip_until(x).unwrap();
            }

            reader.read_exact(&mut buf).unwrap();

            let telemetry = TelemetryPacket::ref_from_bytes(&buf).unwrap();

            let mut lock = TELEMETRY.lock().unwrap();

            *lock = Some(*telemetry);
        }
    });

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

        writer.write_all(&COMMAND_HEADER)?;
        writer.write_all(packet.as_bytes())?;
        writer.flush()?;

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
        } else {
            d.draw_text("No Telemetry", 20, 20, 20, Color::BLACK);
        }
    }

    Ok(())
}
