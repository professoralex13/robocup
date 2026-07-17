use std::env;

use crate::data_source::DataSource;

pub enum RunMode {
    Viewer {
        data_source: DataSource,
    },
    Bridge {
        serial_port: String,
        listen_addr: String,
    },
}

pub fn parse_args() -> RunMode {
    let args: Vec<String> = env::args().collect();
    let serial_port = get_arg_value(&args, "--serial").unwrap_or("/dev/ttyACM0".to_string());

    if has_flag(&args, "--bridge") {
        let listen_addr = get_arg_value(&args, "--listen").unwrap_or("0.0.0.0:9002".to_string());
        return RunMode::Bridge {
            serial_port,
            listen_addr,
        };
    }

    let data_source = match get_arg_value(&args, "--ws") {
        Some(url) => DataSource::WebSocket(url),
        None => DataSource::Serial(serial_port),
    };

    RunMode::Viewer { data_source }
}

fn has_flag(args: &[String], flag: &str) -> bool {
    args.iter().any(|arg| arg == flag)
}

fn get_arg_value(args: &[String], name: &str) -> Option<String> {
    args.windows(2)
        .find(|window| window[0] == name)
        .map(|window| window[1].clone())
}
