#![no_std]
#![no_main]

use teensy4_panic as _;

mod drive_train;
mod lidar;
mod servo;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [KPP])]
mod app {
    use bsp::board;
    use heapless::spsc::Queue;
    use lidar_lib::data::LidarPoint;
    use rtic_sync::{make_signal, signal::SignalReader};
    use teensy4_bsp::{
        self as bsp,
        hal::dma::channel::Channel,
        pins::tmm::{P7, P8},
    };

    use imxrt_log as logging;

    use rtic_monotonics::systick::Systick;

    use crate::servo::ServoController;

    /// There are no resources shared across tasks.
    #[shared]
    struct Shared {
        lidar_points: Queue<LidarPoint, 512>,
    }

    /// These resources are local to individual tasks.
    #[local]
    struct Local {
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
            mut dma,
            mut flexpwm1,
            ..
        } = board::t40(cx.device);

        let (s, drive_train_command_reader) = make_signal!((f32, f32));

        Systick::start(
            cx.core.SYST,
            board::ARM_FREQUENCY,
            rtic_monotonics::create_systick_token!(),
        );

        let drive_servo_controller =
            ServoController::new(&mut flexpwm1.0, flexpwm1.1.3, pins.p8, pins.p7);

        lidar_task::spawn().unwrap();
        drive_train_task::spawn(drive_train_command_reader).unwrap();

        (
            Shared {
                lidar_points: Queue::new(),
            },
            Local {
                drive_servo_controller,
                lidar_serial: board::lpuart(lpuart6, pins.p1, pins.p0, 230400),
                lidar_dma: dma[0].take().unwrap(),
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

    #[task(binds = USB_OTG1, local = [poller])]
    fn log_over_usb(cx: log_over_usb::Context) {
        cx.local.poller.poll();
    }
}
