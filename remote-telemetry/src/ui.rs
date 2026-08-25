use std::f32::consts::PI;

use raylib::{ffi::EndScissorMode, prelude::*};
use zerocopy::IntoBytes;

use crate::{
    data_source::CommandSink,
    protocol::{
        COMMAND_HEADER, CommandPacket, KEY_DRIVE_ERROR, KEY_HEADING, KEY_LEFT_COMMAND,
        KEY_LEFT_WHEEL_VELOCITY, KEY_LOOKAHEAD_X, KEY_LOOKAHEAD_Y, KEY_NEXTPOINT_X,
        KEY_NEXTPOINT_Y, KEY_PITCH, KEY_POSITION_UNCERTAINTY, KEY_POSITION_X, KEY_POSITION_Y,
        KEY_RIGHT_COMMAND, KEY_RIGHT_WHEEL_VELOCITY, KEY_TURN_ERROR, VALUE_TYPE_FLOAT32,
    },
    telemetry_state::{TELEMETRY, TypedValue},
};

const WIDTH: i32 = 1280;
const HEIGHT: i32 = 720;
const FIELD_WIDTH_X_METERS: f32 = 2.425;
const FIELD_HEIGHT_Y_METERS: f32 = 4.85;
const VIEW_MARGIN_PX: f32 = 60.0;

fn value_as_f32(value: Option<&TypedValue>) -> Option<f32> {
    match value {
        Some(v) if v.value_type == VALUE_TYPE_FLOAT32 => Some(f32::from_bits(v.payload)),
        _ => None,
    }
}

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

        let packet = CommandPacket {
            left_command: (y + x).clamp(-1, 1) * 100,
            right_command: (y - x).clamp(-1, 1) * 100,
        };

        let mut payload = Vec::with_capacity(COMMAND_HEADER.len() + packet.as_bytes().len());
        payload.extend_from_slice(&COMMAND_HEADER);
        payload.extend_from_slice(packet.as_bytes());

        // Sending commands is causing pipe halts

        // match command_sink {
        //     CommandSink::Serial(sender) => {
        //         if sender.send(payload).is_err() {
        //             *command_sink = CommandSink::None;
        //         }
        //     }
        //     CommandSink::WebSocket(sender) => {
        //         if sender.send(payload).is_err() {
        //             *command_sink = CommandSink::None;
        //         }
        //     }
        //     CommandSink::None => {}
        // }

        let mut d = rl.begin_drawing(&thread);
        d.clear_background(Color::WHITE);

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

        let telemetry = TELEMETRY.lock().unwrap().clone();

        if !telemetry.values.is_empty() || !telemetry.lidar_points.is_empty() {
            let heading = value_as_f32(telemetry.values.get(&KEY_HEADING)).unwrap_or(0.0);
            let heading_sin = heading.sin();
            let heading_cos = heading.cos();
            let pitch = value_as_f32(telemetry.values.get(&KEY_PITCH)).unwrap_or(0.0);
            let left_wheel_velocity =
                value_as_f32(telemetry.values.get(&KEY_LEFT_WHEEL_VELOCITY)).unwrap_or(0.0);
            let right_wheel_velocity =
                value_as_f32(telemetry.values.get(&KEY_RIGHT_WHEEL_VELOCITY)).unwrap_or(0.0);
            let position_x = value_as_f32(telemetry.values.get(&KEY_POSITION_X)).unwrap_or(0.0);
            let position_y = value_as_f32(telemetry.values.get(&KEY_POSITION_Y)).unwrap_or(0.0);
            let position_uncertainty =
                value_as_f32(telemetry.values.get(&KEY_POSITION_UNCERTAINTY)).unwrap_or(0.0);

            let drive_error = value_as_f32(telemetry.values.get(&KEY_DRIVE_ERROR)).unwrap_or(0.0);
            let turn_error = value_as_f32(telemetry.values.get(&KEY_TURN_ERROR)).unwrap_or(0.0);

            let left_command = value_as_f32(telemetry.values.get(&KEY_LEFT_COMMAND)).unwrap_or(0.0);
            let right_command =
                value_as_f32(telemetry.values.get(&KEY_RIGHT_COMMAND)).unwrap_or(0.0);

            let nextpoint_x = value_as_f32(telemetry.values.get(&KEY_NEXTPOINT_X)).unwrap_or(0.0);
            let nextpoint_y = value_as_f32(telemetry.values.get(&KEY_NEXTPOINT_Y)).unwrap_or(0.0);

            let lookahead_x = value_as_f32(telemetry.values.get(&KEY_LOOKAHEAD_X)).unwrap_or(0.0);
            let lookahead_y = value_as_f32(telemetry.values.get(&KEY_LOOKAHEAD_Y)).unwrap_or(0.0);

            let robot_frame_to_world = |robot_x_m: f32, robot_y_m: f32| -> (f32, f32) {
                let world_x = position_x + heading_cos * robot_x_m + heading_sin * robot_y_m;
                let world_y = position_y - heading_sin * robot_x_m + heading_cos * robot_y_m;

                (world_x, world_y)
            };

            d.draw_text(
                format!("Num Points: {}", telemetry.lidar_points.len()).as_str(),
                20,
                200,
                20,
                Color::BLACK,
            );

            for point in telemetry.lidar_points {
                let lidar_x_robot_m = point.x_mm as f32 / 1000.0;
                let lidar_y_robot_m = point.y_mm as f32 / 1000.0;

                let (lidar_x_world, lidar_y_world) =
                    robot_frame_to_world(lidar_x_robot_m, lidar_y_robot_m);

                let screen = world_to_screen(lidar_x_world, lidar_y_world);

                d.draw_circle_v(screen, 2.0, Color::RED);
            }

            for line in telemetry.lidar_processing.line_fits {
                let (x1_world, y1_world) = robot_frame_to_world(line.x1, line.y1);
                let (x2_world, y2_world) = robot_frame_to_world(line.x2, line.y2);

                let start_screen = world_to_screen(x1_world, y1_world);
                let end_screen = world_to_screen(x2_world, y2_world);

                d.draw_line_ex(start_screen, end_screen, 2.0, Color::GREEN);
            }

            for circle in telemetry.lidar_processing.circle_fits {
                let (cx_world, cy_world) = robot_frame_to_world(circle.cx, circle.cy);

                let c_screen = world_to_screen(cx_world, cy_world);

                d.draw_circle_v(c_screen, circle.r * pixels_per_meter, Color::GREEN);
                d.draw_circle_v(c_screen, circle.r * pixels_per_meter - 5.0, Color::WHITE);
            }

            let robot_screen = world_to_screen(position_x, position_y);

            let uncertainty_m = position_uncertainty.max(0.01);
            let uncertainty_radius_px = uncertainty_m * pixels_per_meter;
            d.draw_circle_lines_v(robot_screen, uncertainty_radius_px, Color::RED);

            let heading_length_m = 0.22f32;
            let (heading_end_x, heading_end_y) = robot_frame_to_world(0.0, heading_length_m);
            let heading_end = world_to_screen(heading_end_x, heading_end_y);
            d.draw_line_ex(robot_screen, heading_end, 3.0, Color::BLUE);

            d.draw_circle_v(world_to_screen(nextpoint_x, nextpoint_y), 3.0, Color::RED);
            d.draw_circle_v(world_to_screen(lookahead_x, lookahead_y), 3.0, Color::BLUE);

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

            d.draw_text(
                format!(
                    "Drive Error: {drive_error:.3}m, Turn Error: {:.3}deg",
                    turn_error * 180.0 / PI
                )
                .as_str(),
                20,
                120,
                20,
                Color::BLACK,
            );

            d.draw_text(
                format!("Left Command: {left_command:.3}, Right Command: {right_command:.3}",)
                    .as_str(),
                20,
                140,
                20,
                Color::BLACK,
            );

            d.draw_text(
                format!("Lookahead: ({lookahead_x:.3}, {lookahead_y:.3})").as_str(),
                20,
                160,
                20,
                Color::BLACK,
            );

            d.draw_text(
                format!("Nextpoint: ({nextpoint_x:.3}, {nextpoint_y:.3})").as_str(),
                20,
                180,
                20,
                Color::BLACK,
            );
        } else {
            d.draw_text("No Telemetry", 20, 20, 20, Color::BLACK);
        }
    }

    Ok(())
}
