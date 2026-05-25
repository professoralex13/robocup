use rtic::Mutex;

use lidar_lib::data::{LidarDataReader, LidarPacket};

pub async fn entrypoint(cx: crate::app::lidar_task::Context<'_>) {
    let serial = cx.local.lidar_serial;
    let dma = cx.local.lidar_dma;

    let mut lidar_points = cx.shared.lidar_points;

    let mut reader: LidarDataReader = LidarDataReader::new();

    loop {
        let mut buffer = [0u8; LidarPacket::SIZE];

        serial.dma_read(dma, &mut buffer).await.unwrap();

        if let Some(packet) = reader.read_slice(&buffer).unwrap() {
            lidar_points.lock(|points| {
                for point in packet.points {
                    if points.is_full() {
                        points.dequeue();
                    }

                    points.enqueue(point).unwrap();
                }
            })
        }
    }
}
