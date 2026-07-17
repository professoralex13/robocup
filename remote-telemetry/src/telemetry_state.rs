use std::{
    collections::{HashMap, VecDeque},
    sync::{LazyLock, Mutex},
};

use crate::protocol::LidarPoint;

#[derive(Copy, Clone, Debug)]
pub struct TypedValue {
    pub value_type: u8,
    pub payload: u32,
}

#[derive(Clone, Debug)]
pub struct TelemetrySnapshot {
    pub values: HashMap<u16, TypedValue>,
    pub lidar_points: VecDeque<LidarPoint>,
}

impl TelemetrySnapshot {
    pub fn new() -> Self {
        Self {
            values: HashMap::new(),
            lidar_points: VecDeque::with_capacity(1500),
        }
    }
}

pub static TELEMETRY: LazyLock<Mutex<TelemetrySnapshot>> =
    LazyLock::new(|| Mutex::new(TelemetrySnapshot::new()));
