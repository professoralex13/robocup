use linalg::{quaternion::Quaternion, real_vector};
use rtic::Mutex;
use rtic_monotonics::systick::{ExtU32, Systick};
use teensy4_bsp::hal::lpi2c::ControllerStatus;
use uom::si::{f32::Ratio, ratio::ratio};

pub async fn entrypoint(
    mut cx: crate::app::imu_task::Context<'_>,
) -> Result<(), bno055::Error<ControllerStatus>> {
    let imu = cx.local.imu;

    imu.init(&mut Systick)?;

    imu.set_mode(bno055::BNO055OperationMode::NDOF, &mut Systick)?;

    loop {
        let Ok(quat) = imu.quaternion() else {
            // The I2C will error every now and then, not critical
            continue;
        };

        let converted = Quaternion::from_direction_component(
            Ratio::new::<ratio>(quat.s),
            real_vector!(Ratio::ratio, quat.v.x, quat.v.y, quat.v.z),
        );

        cx.shared.robot_orientation.lock(|robot_orientation| {
            *robot_orientation = converted;
        });

        Systick::delay(20.millis()).await;
    }
}
