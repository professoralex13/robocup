use rtic_sync::signal::SignalReader;

pub async fn entrypoint(
    cx: crate::app::drive_train_task::Context<'_>,
    mut command_signal: SignalReader<'_, (f32, f32)>,
) {
    let servo_controller = cx.local.drive_servo_controller;

    loop {
        let (left_command, right_command) = command_signal.wait().await;

        servo_controller.set_a_value(left_command);
        servo_controller.set_b_value(right_command);
    }
}
