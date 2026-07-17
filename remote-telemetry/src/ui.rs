use std::{f32::consts::PI, io::Write};

use raylib::prelude::*;
use zerocopy::IntoBytes;

use crate::{
    data_source::CommandSink,
    protocol::{COMMAND_HEADER, CommandPacket},
    telemetry_state::TELEMETRY,
};

const WIDTH: i32 = 1280;
const HEIGHT: i32 = 720;
const FIELD_WIDTH_X_METERS: f32 = 2.4;
const FIELD_HEIGHT_Y_METERS: f32 = 4.9;
const VIEW_MARGIN_PX: f32 = 60.0;

pub fn run_ui(command_sink: &mut CommandSink) -> Result<(), Box<dyn std::error::Error>> {
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

            let robot_frame_to_world = |robot_x_m: f32, robot_y_m: f32| -> (f32, f32) {
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
                format!("X: {:.3}m, Y: {:.3}m", 1.0 * position_x, 1.0 * position_y).as_str(),
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