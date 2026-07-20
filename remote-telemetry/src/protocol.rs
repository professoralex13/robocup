use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout};

pub const TELEMETRY_PREAMBLE: [u8; 4] = [0xAB, 0xCD, 0xEF, 0x42];
pub const TELEMETRY_VERSION: u8 = 1;

pub const FRAME_TYPE_VALUES: u8 = 1;
pub const FRAME_TYPE_LIDAR: u8 = 2;

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
}

#[derive(Clone, Debug)]
pub enum TelemetryFrame {
    Values(Vec<ValueEntry>),
    Lidar(Vec<LidarPoint>),
}

pub fn parse_frame(frame_type: u8, payload: &[u8]) -> Option<TelemetryFrame> {
    if payload.len() < 2 {
        return None;
    }

    let count = u16::from_le_bytes([payload[0], payload[1]]) as usize;
    let body = &payload[2..];

    match frame_type {
        FRAME_TYPE_VALUES => {
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

                points.push(LidarPoint {
                    x_mm,
                    y_mm,
                    intensity,
                });
            }

            Some(TelemetryFrame::Lidar(points))
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
