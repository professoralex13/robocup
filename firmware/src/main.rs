#![no_std]
#![no_main]

use teensy4_panic as _;

mod drive_train;
mod imu;
mod lidar;
mod servo;
mod telemetry;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [KPP])]
mod app {
    use bno055::Bno055;
    use heapless::spsc::Queue;
    use lidar_lib::data::LidarPoint;
    use linalg::{quaternion::Quaternion, vector::Vector};
    use rtic_sync::{make_signal, signal::SignalReader};
    use teensy4_bsp::{
        board::{self, Lpi2cClockSpeed},
        hal::{
            dma::channel::Channel,
            lpi2c::{Lpi2c, Pins},
        },
        pins::t40::{P7, P8, P18, P19},
    };

    use imxrt_log as logging;

    use rtic_monotonics::systick::Systick;
    use uom::si::f32::Ratio;

    use crate::servo::ServoController;

    /// There are no resources shared across tasks.
    #[shared]
    struct Shared {
        lidar_points: Queue<LidarPoint, 512>,
        robot_orientation: Quaternion<Ratio>,
    }

    /// These resources are local to individual tasks.
    #[local]
    struct Local {
        // Telemetry
        telemetry_serial: board::Lpuart7,
        telemetry_read_dma: Channel,
        telemetry_write_dma: Channel,
        // Imu
        imu: Bno055<Lpi2c<Pins<P19, P18>, 1>>,
        // Drive Train
        drive_servo_controller: ServoController<1, 3, P8, P7>,
        // Lidar
        lidar_serial: board::Lpuart6,
        lidar_dma: Channel,
        // Logging
        poller: logging::Poller,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let board::Resources {
            pins,
            usb,
            lpuart6,
            lpuart7,
            mut dma,
            mut flexpwm1,
            lpi2c1,
            ..
        } = board::t40(cx.device);

        let (s, drive_train_command_reader) = make_signal!((f32, f32));

        Systick::start(
            cx.core.SYST,
            board::ARM_FREQUENCY,
            rtic_monotonics::create_systick_token!(),
        );

        lidar_task::spawn().unwrap();
        drive_train_task::spawn(drive_train_command_reader).unwrap();
        imu_task::spawn().unwrap();
        telemetry_task::spawn().unwrap();

        (
            Shared {
                lidar_points: Queue::new(),
                robot_orientation: Vector::default(),
            },
            Local {
                imu: Bno055::new(board::lpi2c(
                    lpi2c1,
                    pins.p19,
                    pins.p18,
                    Lpi2cClockSpeed::KHz400,
                ))
                .with_alternative_address(),
                drive_servo_controller: ServoController::new(
                    &mut flexpwm1.0,
                    flexpwm1.1.3,
                    pins.p8,
                    pins.p7,
                ),
                lidar_serial: board::lpuart(lpuart6, pins.p1, pins.p0, 230400),
                lidar_dma: dma[0].take().unwrap(),
                telemetry_read_dma: dma[1].take().unwrap(),
                telemetry_write_dma: dma[2].take().unwrap(),
                telemetry_serial: board::lpuart(lpuart7, pins.p29, pins.p28, 115200),
                poller: logging::log::usbd(usb, logging::Interrupts::Enabled).unwrap(),
            },
        )
    }

    #[task(local=[lidar_serial, lidar_dma], shared=[lidar_points])]
    async fn lidar_task(cx: lidar_task::Context) {
        crate::lidar::entrypoint(cx).await
    }

    #[task(local=[drive_servo_controller])]
    async fn drive_train_task(
        cx: drive_train_task::Context,
        command_signal: SignalReader<'static, (f32, f32)>,
    ) {
        crate::drive_train::entrypoint(cx, command_signal).await
    }

    #[task(local=[imu], shared=[robot_orientation])]
    async fn imu_task(cx: imu_task::Context) {
        crate::imu::entrypoint(cx).await.unwrap()
    }

    #[task(local=[telemetry_read_dma, telemetry_write_dma, telemetry_serial], shared=[robot_orientation, lidar_points])]
    async fn telemetry_task(cx: telemetry_task::Context) {
        crate::telemetry::entrpoint(cx).await.unwrap()
    }

    #[task(binds = USB_OTG1, local = [poller])]
    fn log_over_usb(cx: log_over_usb::Context) {
        cx.local.poller.poll();
    }
}
