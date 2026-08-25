use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout};

pub const TELEMETRY_PREAMBLE: [u8; 4] = [0xAB, 0xCD, 0xEF, 0x42];
pub const TELEMETRY_VERSION: u8 = 1;

pub const FRAME_TYPE_VALUES: u8 = 1;
pub const FRAME_TYPE_LIDAR: u8 = 2;
pub const FRAME_TYPE_LIDAR_PROCESSING: u8 = 3;

pub const VALUE_TYPE_FLOAT32: u8 = 1;
pub const VALUE_TYPE_INT32: u8 = 2;
pub const VALUE_TYPE_UINT32: u8 = 3;
pub const VALUE_TYPE_BOOL: u8 = 4;

pub const KEY_HEADING: u16 = 1;
pub const KEY_PITCH: u16 = 2;
pub const KEY_LEFT_WHEEL_VELOCITY: u16 = 3;
pub const KEY_RIGHT_WHEEL_VELOCITY: u16 = 4;
pub const KEY_POSITION_X: u16 = 5;
pub const KEY_POSITION_Y: u16 = 6;
pub const KEY_POSITION_UNCERTAINTY: u16 = 7;
pub const KEY_DRIVE_ERROR: u16 = 8;
pub const KEY_TURN_ERROR: u16 = 9;
pub const KEY_LEFT_COMMAND: u16 = 10;
pub const KEY_RIGHT_COMMAND: u16 = 11;
pub const KEY_LOOKAHEAD_X: u16 = 12;
pub const KEY_LOOKAHEAD_Y: u16 = 13;
pub const KEY_NEXTPOINT_X: u16 = 14;
pub const KEY_NEXTPOINT_Y: u16 = 15;

pub const TELEMETRY_FRAME_HEADER_LEN: usize = 8;

#[derive(Copy, Clone, Debug)]
pub struct ValueEntry {
    pub key: u16,
    pub value_type: u8,
    pub payload: u32,
}

#[derive(Copy, Clone, Debug)]
pub struct LidarPoint {
    pub x_mm: i16,
    pub y_mm: i16,
    pub intensity: u8,
    pub flags: u8,
}

#[derive(Copy, Clone, Debug, FromBytes, KnownLayout, Immutable)]
pub struct LineFit {
    pub x1: f32,
    pub x2: f32,
    pub y1: f32,
    pub y2: f32,

    pub slope: f32,
    pub intercept: f32,
}

#[derive(Copy, Clone, Debug, FromBytes, KnownLayout, Immutable)]
pub struct CircleFit {
    pub cx: f32,
    pub cy: f32,
    pub r: f32,

    pub radius_deviation: f32,
}

#[derive(Clone, Debug)]
pub struct LidarProcessing {
    pub line_fits: Vec<LineFit>,
    pub circle_fits: Vec<CircleFit>,
}

#[derive(Clone, Debug)]
pub enum TelemetryFrame {
    Values(Vec<ValueEntry>),
    Lidar(Vec<LidarPoint>),
    LidarProcessing(LidarProcessing),
}

pub fn parse_frame(frame_type: u8, payload: &[u8]) -> Option<TelemetryFrame> {
    if payload.len() < 2 {
        return None;
    }

    match frame_type {
        FRAME_TYPE_VALUES => {
            let count = u16::from_le_bytes([payload[0], payload[1]]) as usize;
            let body = &payload[2..];

            let entry_size = 8usize;
            if body.len() < count * entry_size {
                return None;
            }

            let mut entries = Vec::with_capacity(count);
            for i in 0..count {
                let base = i * entry_size;
                let key = u16::from_le_bytes([body[base], body[base + 1]]);
                let value_type = body[base + 2];
                let payload = u32::from_le_bytes([
                    body[base + 4],
                    body[base + 5],
                    body[base + 6],
                    body[base + 7],
                ]);

                entries.push(ValueEntry {
                    key,
                    value_type,
                    payload,
                });
            }

            Some(TelemetryFrame::Values(entries))
        }
        FRAME_TYPE_LIDAR => {
            let count = u16::from_le_bytes([payload[0], payload[1]]) as usize;
            let body = &payload[2..];

            let point_size = 6usize;
            if body.len() < count * point_size {
                return None;
            }

            let mut points = Vec::with_capacity(count);
            for i in 0..count {
                let base = i * point_size;
                let x_mm = i16::from_le_bytes([body[base], body[base + 1]]);
                let y_mm = i16::from_le_bytes([body[base + 2], body[base + 3]]);
                let intensity = body[base + 4];
                let flags = body[base + 5];

                points.push(LidarPoint {
                    x_mm,
                    y_mm,
                    intensity,
                    flags,
                });
            }

            Some(TelemetryFrame::Lidar(points))
        }
        FRAME_TYPE_LIDAR_PROCESSING => {
            const line_size: usize = size_of::<LineFit>();

            let lines_count = u16::from_le_bytes([payload[0], payload[1]]) as usize;
            let lines_body = &payload[2..2 + lines_count * line_size];

            let mut lines = Vec::with_capacity(lines_count);

            for i in 0..lines_count {
                let data =
                    LineFit::read_from_bytes(&lines_body[i * line_size..i * line_size + line_size])
                        .unwrap();

                lines.push(data);
            }

            const circle_size: usize = size_of::<CircleFit>();

            let circles_count = u16::from_le_bytes([
                payload[2 + lines_count * line_size],
                payload[2 + lines_count * line_size + 1],
            ]) as usize;
            let circles_body = &payload[4 + lines_count * line_size..];

            let mut circles = Vec::with_capacity(circles_count);

            for i in 0..circles_count {
                let data = CircleFit::read_from_bytes(
                    &circles_body[i * circle_size..(i + 1) * circle_size],
                )
                .unwrap();

                circles.push(data);
            }

            Some(TelemetryFrame::LidarProcessing(LidarProcessing {
                line_fits: lines,
                circle_fits: circles,
            }))
        }
        _ => None,
    }
}

#[repr(C, packed)]
#[derive(Copy, Clone, FromBytes, IntoBytes, Debug, KnownLayout, Immutable)]
pub struct CommandPacket {
    pub left_command: i8,
    pub right_command: i8,
}

pub const COMMAND_HEADER: [u8; 10] = [0x99, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00];
