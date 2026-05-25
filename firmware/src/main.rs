#![no_std]
#![no_main]

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [KPP])]
mod app {
    use bsp::board;
    use heapless::spsc::Queue;
    use lidar_lib::data::{LidarDataReader, LidarPacket, LidarPoint};
    use teensy4_bsp::{self as bsp, hal::dma::channel::Channel};

    use imxrt_log as logging;

    use rtic_monotonics::systick::Systick;

    /// There are no resources shared across tasks.
    #[shared]
    struct Shared {
        lidar_points: Queue<LidarPoint, 512>,
    }

    /// These resources are local to individual tasks.
    #[local]
    struct Local {
        serial1: board::Lpuart6,
        dma_ch0: Channel,
        /// A poller to control USB logging.
        poller: logging::Poller,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins,
            usb,
            lpuart6,
            mut dma,
            ..
        } = board::t40(cx.device);

        let mut serial1 = board::lpuart(lpuart6, pins.p1, pins.p0, 230400);
        serial1.enable_dma_receive();

        let dma_ch0 = dma[0].take().unwrap();

        let poller = logging::log::usbd(usb, logging::Interrupts::Enabled).unwrap();

        Systick::start(
            cx.core.SYST,
            board::ARM_FREQUENCY,
            rtic_monotonics::create_systick_token!(),
        );

        lidar::spawn().unwrap();

        (
            Shared {
                lidar_points: Queue::new(),
            },
            Local {
                serial1,
                dma_ch0,
                poller,
            },
        )
    }

    #[task(local=[serial1, dma_ch0], shared=[lidar_points])]
    async fn lidar(cx: lidar::Context) {
        let serial1 = cx.local.serial1;
        let dma_ch0 = cx.local.dma_ch0;

        let mut lidar_points = cx.shared.lidar_points;

        let mut reader: LidarDataReader = LidarDataReader::new();

        loop {
            let mut buffer = [0u8; LidarPacket::SIZE];

            serial1.dma_read(dma_ch0, &mut buffer).await.unwrap();

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

    #[task(binds = USB_OTG1, local = [poller])]
    fn log_over_usb(cx: log_over_usb::Context) {
        cx.local.poller.poll();
    }
}
