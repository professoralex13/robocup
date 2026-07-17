mod bridge;
mod cli;
mod data_source;
mod protocol;
mod telemetry_state;
mod ui;

use cli::RunMode;

pub const BAUD_RATE: u32 = 921600;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    match cli::parse_args() {
        RunMode::Bridge {
            serial_port,
            listen_addr,
        } => bridge::run_bridge(&serial_port, &listen_addr),
        RunMode::Viewer { data_source } => {
            let mut command_sink = data_source::initialize_data_source(data_source)?;
            ui::run_ui(&mut command_sink)
        }
    }
}
