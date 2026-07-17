use std::sync::Mutex;

use crate::protocol::TelemetryPacket;

pub static TELEMETRY: Mutex<Option<TelemetryPacket>> = Mutex::new(None);
