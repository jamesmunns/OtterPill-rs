use chrono::prelude::*;
use chrono::{DateTime, Local, TimeZone};
use postcard::{from_bytes, to_slice_cobs};
use serde::{Deserialize, Serialize};
use serde_json;
use serialport::prelude::*;
use std::{
    collections::{HashMap, VecDeque},
    fs::{read_to_string, File, OpenOptions},
    io::{self, prelude::*},
    path::Path,
    sync::mpsc::{Receiver, Sender, TryRecvError},
    thread::sleep,
    time::{Duration, Instant},
};
use structopt::StructOpt;
use toml::{from_str, to_string};
use vintage_icd::{
    cobs_buffer::{consts::*, Buffer, FeedResult},
    DeviceToHostMessages, HostToDeviceMessages,
};

#[derive(StructOpt, Debug)]
#[structopt(rename_all = "kebab-case")]
enum SubCommands {
    Reset,
    Log,
    Debug,
}

#[derive(Debug, Serialize, Deserialize)]
struct Config {
    port: String,
    tasks: HashMap<String, Task>,
}

#[derive(Debug, Serialize, Deserialize, Default)]
struct DayLog {
    log: HashMap<String, TaskLog>,
}

#[derive(Debug, Serialize, Deserialize)]
struct TaskLog {
    time_spent: Duration,
    events: Vec<LogEvent>,
}

#[derive(Debug, Serialize, Deserialize)]
enum LogEvent {
    Start(DateTime<Local>),
    End(DateTime<Local>),
}

#[derive(Debug, Serialize, Deserialize)]
struct Task {
    pin: u8,
}

pub type Error = Box<dyn std::error::Error>;
pub type Result<T> = std::result::Result<T, Error>;

fn main() -> Result<()> {
    let opt = SubCommands::from_args();

    let config = from_str::<Config>(&read_to_string("./vintage-settings.toml")?)?;

    let mut settings: SerialPortSettings = Default::default();
    settings.timeout = Duration::from_millis(1000);
    settings.baud_rate = 115_200;

    let mut port = match serialport::open_with_settings("/dev/ttyACM0", &settings) {
        Ok(port) => port,
        Err(e) => {
            eprintln!("Failed to open \"{}\". Error: {}", "/dev/ttyACM0", e);
            ::std::process::exit(1);
        }
    };

    let ret = match opt {
        SubCommands::Reset => {
            reset(&mut port).ok();
            Ok(())
        }
        SubCommands::Log => log(config, &mut port),
        SubCommands::Debug => debug(&mut port),
    };

    if ret.is_err() {
        println!();
    }

    ret
}

fn reset(port: &mut Box<dyn SerialPort>) -> Result<()> {
    let mut raw_buf = [0u8; 256];
    let msg = HostToDeviceMessages::Reset;

    loop {
        if let Ok(slice) = postcard::to_slice_cobs(&msg, &mut raw_buf) {
            println!("Reset...");
            port.write_all(slice)?;
            port.flush()?;
        }
        sleep(Duration::from_millis(100));
    }
}

fn log(cfg: Config, port: &mut Box<dyn SerialPort>) -> Result<()> {
    let mut cobs_buf: Buffer<U256> = Buffer::new();
    let mut raw_buf = [0u8; 256];
    let mut now = Instant::now();
    let mut msgs = VecDeque::new();

    let log_path = format!("logs/{}.json", Local::today().naive_local());
    let mut current = read_to_string(&log_path)
        .map_err(drop)
        .and_then(|msg| serde_json::from_str(msg.as_str()).map_err(drop))
        .unwrap_or_else(|_| DayLog::default());

    #[derive(Debug)]
    struct LogState {
        pin: u8,
        last_flush: u32,
        last_ticks: u32,
    }

    let mut log_state: Option<LogState> = None;

    let mut project_idx: Vec<Option<String>> = vec![None; 16];

    for (key, val) in cfg.tasks.iter() {
        project_idx[val.pin as usize] = Some(key.clone());
    }

    loop {
        let buf = match port.read(&mut raw_buf) {
            Ok(ct) => &raw_buf[..ct],
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                print!(".");
                io::stdout().flush().ok().expect("Could not flush stdout");
                continue;
            }
            Err(e) => {
                eprintln!("{:?}", e);
                return Err(Error::from("BAD SERIAL ERROR"));
            }
        };

        let mut window = &buf[..];

        'cobs: while !window.is_empty() {
            use FeedResult::*;
            window = match cobs_buf.feed::<DeviceToHostMessages>(&window) {
                Consumed => break 'cobs,
                OverFull(new_wind) => new_wind,
                DeserError(new_wind) => new_wind,
                Success { data, remaining } => {
                    msgs.push_back(data);
                    remaining
                }
            };
        }

        use vintage_icd::{CurrentState, StatusMessage};
        use CurrentState::*;

        while let Some(msg) = msgs.pop_front() {
            match msg {
                DeviceToHostMessages::Status(StatusMessage {
                    current_tick,
                    state: CurrentState::Idle,
                }) => {
                    // Were we tracking something before, and we are now idle?
                    if let Some(state) = log_state.as_ref() {
                        // Is there a task by the name of what we were tracking?
                        if let Some(task_name) = &project_idx[state.pin as usize] {
                            let delta = state.last_ticks - state.last_flush;

                            // Is the task known in the list?
                            if let Some(log) = current.log.get_mut(task_name) {
                                log.events.push(LogEvent::End(Local::now()));
                                log.time_spent += Duration::from_millis(delta as u64);
                            }
                        }

                        // WRITE FILE
                        let contents = serde_json::to_string(&current)?;

                        let mut opt = OpenOptions::new();
                        opt.write(true);
                        opt.truncate(true);
                        opt.create(true);

                        let mut file = opt.open(&log_path)?;
                        file.write_all(contents.as_bytes())?;
                    }
                    log_state = None;
                }
                DeviceToHostMessages::Status(StatusMessage {
                    current_tick,
                    state: CurrentState::Timing { pin, elapsed },
                }) => {
                    // Is there a task by the name of what we were tracking?
                    if let Some(task_name) = &project_idx[pin as usize] {
                        let log = current.log.entry(task_name.to_string()).or_insert(TaskLog {
                            time_spent: Duration::from_millis(0),
                            events: vec![],
                        });

                        // Were we tracking something before?
                        if log_state.is_none() {
                            log_state = Some(LogState {
                                pin,
                                last_flush: 0,
                                last_ticks: elapsed,
                            });
                            log.events.push(LogEvent::Start(Local::now()))
                        }

                        let ref_state = log_state.as_mut().unwrap();
                        let delta = elapsed - ref_state.last_flush;
                        ref_state.last_ticks = elapsed;

                        if delta >= 5000 {
                            ref_state.last_flush = elapsed;
                            log.time_spent += Duration::from_millis(delta as u64);

                            // WRITE FILE
                            let contents = serde_json::to_string(&current)?;

                            let mut opt = OpenOptions::new();
                            opt.write(true);
                            opt.truncate(true);
                            opt.create(true);

                            let mut file = opt.open(&log_path)?;
                            file.write_all(contents.as_bytes())?;
                        }
                    }
                }
                DeviceToHostMessages::Status(StatusMessage {
                    current_tick,
                    state: CurrentState::Timeout { elapsed },
                }) => {
                    // Were we tracking something before, and we are now timed out?
                    if let Some(state) = log_state.as_ref() {
                        // Is there a task by the name of what we were tracking?
                        if let Some(task_name) = &project_idx[state.pin as usize] {
                            let delta = state.last_ticks - state.last_flush;

                            // Is the task known in the list?
                            if let Some(log) = current.log.get_mut(task_name) {
                                log.events.push(LogEvent::End(Local::now()));
                                log.time_spent += Duration::from_millis(delta as u64);
                            }
                        }

                        // WRITE FILE
                        let contents = serde_json::to_string(&current)?;

                        let mut opt = OpenOptions::new();
                        opt.write(true);
                        opt.truncate(true);
                        opt.create(true);

                        let mut file = opt.open(&log_path)?;
                        file.write_all(contents.as_bytes())?;
                    }
                    log_state = None;
                }
                _ => {}
            }
        }
    }
}

fn debug(port: &mut Box<dyn SerialPort>) -> Result<()> {
    let mut cobs_buf: Buffer<U256> = Buffer::new();
    let mut raw_buf = [0u8; 256];
    let mut now = Instant::now();

    loop {
        if now.elapsed() >= Duration::from_millis(100) {
            now = Instant::now();

            let msg = HostToDeviceMessages::Ping;

            if let Ok(slice) = postcard::to_slice_cobs(&msg, &mut raw_buf) {
                port.write_all(slice).map_err(drop).ok();
                port.flush().map_err(drop).ok();
            }

            println!("\nSENT: {:?}", msg);
        }

        let buf = match port.read(&mut raw_buf) {
            Ok(ct) => &raw_buf[..ct],
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                print!(".");
                io::stdout().flush().ok().expect("Could not flush stdout");
                continue;
            }
            Err(e) => {
                eprintln!("{:?}", e);
                return Err(Error::from("BAD SERIAL ERROR"));
            }
        };

        let mut window = &buf[..];

        'cobs: while !window.is_empty() {
            use FeedResult::*;
            window = match cobs_buf.feed::<DeviceToHostMessages>(&window) {
                Consumed => break 'cobs,
                OverFull(new_wind) => new_wind,
                DeserError(new_wind) => new_wind,
                Success { data, remaining } => {
                    println!("\nGOT: {:?}", data);
                    remaining
                }
            };
        }
    }
}
