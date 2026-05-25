#![no_std]

use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout};

pub const TELEMETRY_HEADER: [u8; 8] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, FromBytes, IntoBytes, Immutable)]
pub struct LidarPoint {
    pub x: i16,
    pub y: i16,
    pub intensity: u8,
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug, FromBytes, IntoBytes, KnownLayout, Immutable)]
pub struct TelemetryPacket {
    pub points: [LidarPoint; 512],
    pub heading: f32,
    pub pitch: f32,

    pub left_wheel_velocity: f32,
    pub right_wheel_velocity: f32,
}
