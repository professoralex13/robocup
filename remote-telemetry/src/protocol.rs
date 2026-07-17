use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout};

#[repr(C, packed)]
#[derive(Copy, Clone, IntoBytes, FromBytes, Debug, Immutable)]
pub struct LidarPoint {
    pub x: i16,
    pub y: i16,
    pub intensity: u8,
}

#[repr(C, packed)]
#[derive(Copy, Clone, IntoBytes, FromBytes, Debug, KnownLayout, Immutable)]
pub struct TelemetryPacket {
    pub points: [LidarPoint; 500],
    pub heading: f32,
    pub pitch: f32,

    pub left_wheel_velocity: f32,
    pub right_wheel_velocity: f32,

    pub position_x: f32,
    pub position_y: f32,
    pub position_uncertainty: f32,
}

#[repr(C, packed)]
#[derive(Copy, Clone, FromBytes, IntoBytes, Debug, KnownLayout, Immutable)]
pub struct CommandPacket {
    pub left_command: i8,
    pub right_command: i8,
}

pub const TELEMETRY_HEADER: [u8; 8] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
pub const COMMAND_HEADER: [u8; 10] = [0x99, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00];
