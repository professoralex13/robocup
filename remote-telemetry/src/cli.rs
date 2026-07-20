use std::env;

use crate::data_source::DataSource;

const DEFAULT_PORT: u16 = 9002;

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
        let listen_arg = get_arg_value(&args, "--listen").unwrap_or("0.0.0.0".to_string());

        let (listen_host, listen_arg_port) = split_host_and_optional_port(&listen_arg);

        let port = listen_arg_port.unwrap_or(DEFAULT_PORT);

        let listen_addr = format!("{listen_host}:{port}");

        return RunMode::Bridge {
            serial_port,
            listen_addr,
        };
    }

    let data_source = match get_arg_value(&args, "--ws") {
        Some(url) => {
            let (listen_host, listen_arg_port) = split_host_and_optional_port(&url);

            let port = listen_arg_port.unwrap_or(DEFAULT_PORT);

            DataSource::WebSocket(format!("ws://{listen_host}:{port}"))
        }
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

fn split_host_and_optional_port(value: &str) -> (&str, Option<u16>) {
    match value.rsplit_once(':') {
        Some((host, port_str)) if !host.is_empty() && !port_str.is_empty() => {
            match port_str.parse::<u16>() {
                Ok(port) => (host, Some(port)),
                Err(_) => (value, None),
            }
        }
        _ => (value, None),
    }
}
