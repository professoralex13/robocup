use core::array;

use firmware::{LidarPoint, TELEMETRY_HEADER, TelemetryPacket};
use rtic::Mutex;
use teensy4_bsp::hal::dma::Error;
use uom::si::length::millimeter;
use zerocopy::IntoBytes;

pub async fn entrpoint(mut cx: crate::app::telemetry_task::Context<'_>) -> Result<(), Error> {
    let serial = cx.local.telemetry_serial;

    loop {
        let orientation = cx.shared.robot_orientation.lock(|x| x.clone());
        let lidar_points = cx.shared.lidar_points.lock(|x| x.clone());

        let packet = TelemetryPacket {
            pitch: orientation.pitch().value,
            heading: orientation.yaw().value,
            left_wheel_velocity: 0.0,
            right_wheel_velocity: 0.0,
            points: array::from_fn(|i| {
                let point = lidar_points.iter().nth(i).unwrap();

                LidarPoint {
                    x: point.0.x().get::<millimeter>() as i16,
                    y: point.0.y().get::<millimeter>() as i16,
                    intensity: point.1,
                }
            }),
        };

        serial
            .dma_write(cx.local.telemetry_write_dma, &TELEMETRY_HEADER)
            .await?;

        serial
            .dma_write(cx.local.telemetry_write_dma, packet.as_bytes())
            .await?;
    }
}
