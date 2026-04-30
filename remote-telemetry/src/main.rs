use std::{
    io::{BufRead, BufReader, Read},
    time::Duration,
};

use raylib::prelude::*;
use zerocopy::{FromBytes, Immutable, KnownLayout};

const WIDTH: i32 = 1280;
const HEIGHT: i32 = 720;

#[repr(C, packed)]
#[derive(Copy, Clone, FromBytes, Debug, Immutable)]
struct LidarPoint {
    x: i16,
    y: i16,
    intensity: u8,
}

#[repr(C, packed)]
#[derive(Copy, Clone, FromBytes, Debug, KnownLayout, Immutable)]
struct TelemetryPacket {
    points: [LidarPoint; 500],
    heading: f32,
    pitch: f32,
}

const HEADER: [u8; 8] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let port = serialport::new("/dev/ttyCH343USB0", 115200)
        .timeout(Duration::from_secs(10))
        .open()
        .unwrap();

    let mut reader = BufReader::new(port);

    let (mut rl, thread) = raylib::init()
        .size(WIDTH, HEIGHT)
        .title("Hello, World")
        .build();

    while !rl.window_should_close() {
        let mut d = rl.begin_drawing(&thread);

        d.clear_background(Color::WHITE);

        let mut buf = [0u8; std::mem::size_of::<TelemetryPacket>()];

        for x in HEADER {
            reader.skip_until(x)?;
        }

        reader.read_exact(&mut buf)?;

        let telemetry = TelemetryPacket::ref_from_bytes(&buf).unwrap();

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
        )
    }

    Ok(())
}
