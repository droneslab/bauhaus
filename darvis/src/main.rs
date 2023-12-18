#![feature(map_many_mut)]
#![feature(hash_extract_if)]
#![feature(extract_if)]

use std::{fs::{OpenOptions, File}, path::Path, env, time::{self, Duration}, io::{self, BufRead}, thread::{self, sleep}, sync::Arc};
use dvmap::map::Map;
use fern::colors::{ColoredLevelConfig, Color};
use glob::glob;
use log::info;
use parking_lot::{Mutex, RwLock};
use spin_sleep::LoopHelper;
#[macro_use] extern crate lazy_static;

use dvcore::{*, config::*, actor::ActorChannels, maplock::ReadWriteMap};
use crate::{actors::messages::{ShutdownMsg, ImagePathMsg, ImageMsg}, registered_actors::TRACKING_FRONTEND, modules::image};
use crate::dvmap::{bow::VOCABULARY, map::Id};

mod actors;
mod registered_actors;
mod spawn;
mod dvmap;
mod modules;
mod tests;

pub type MapLock = ReadWriteMap<Map>;
// pub type MapLock = Arc<Mutex<Map>>; // If you want to switch all locks to mutexes

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() < 3 {
        panic!("
            [ERROR] Invalid number of input parameters.
            Usage: cargo run -- [PATH_TO_DATA_DIRECTORY] [PATH_TO_CONFIG_FILE]
        ");
    }
    let img_dir = args[1].to_owned();
    let config_file = args[2].to_owned();

    // Load config, including custom settings and actor information
    let (actor_info, _module_info, log_level) = load_config(&config_file).expect("Could not load config");

    setup_logger(&log_level)?;

    // a builder for `FmtSubscriber`.
    let subscriber = tracing_subscriber::FmtSubscriber::builder()
        // all spans/events with a level higher than TRACE (e.g, debug, info, warn, etc.)
        // will be written to stdout.
        .with_max_level(tracing::Level::TRACE)
        // completes the builder.
        .finish();

    tracing::subscriber::set_global_default(subscriber)
        .expect("setting default subscriber failed");

    // Launch actor system
    let first_actor_name = TRACKING_FRONTEND.to_string(); // Actor that launches the pipeline
    let (shutdown_flag, first_actor_tx, shutdown_tx) = spawn::launch_actor_system(actor_info, first_actor_name)?;

    // Load vocabulary
    VOCABULARY.access();

    info!("System ready to receive messages!");

    let mut loop_sleep = LoopSleep::new(&img_dir);
    let timestamps = read_timestamps_file(&img_dir);

    // Check deadlocks. Turn off if you aren't using this, otherwise it will slow everything down.
    // If you turn this on and also turn all rwlocks into mutexes, this seems to more consistently
    // find deadlock errors. You can turn all rwlocks into mutexes pretty quickly by modifying 
    // pub type MapLock and turning read() and write() into lock()
    // thread::spawn(move || { 
    //     loop {
    //         let deadlocks = parking_lot::deadlock::check_deadlock();
    //         if !deadlocks.is_empty() {
    //             println!("{} deadlocks detected", deadlocks.len());
    //             for (i, threads) in deadlocks.iter().enumerate() {
    //                 println!("Deadlock #{}", i);
    //                 for t in threads {
    //                     println!("Thread Id {:#?}", t.thread_id());
    //                     println!("{:#?}", t.backtrace());
    //                 }
    //             }
    //         }
    //         thread::sleep(Duration::from_secs_f64(2.0));
    //     }
    // } );

    // Process images
    // let now = SystemTime::now();
    let mut i = 0;
    for path in &generate_image_paths(img_dir + "/image_0/") {
        if *shutdown_flag.lock().unwrap() { break; }
        loop_sleep.start();

        let image = image::read_image_file(&path.to_string());

        first_actor_tx.send(Box::new(
            ImageMsg{
                image,
                timestamp: timestamps[i],
                frame_id: i as u32
            }
        ))?;

        i += 1;
        // TODO (timestamps) ... if use_timestamps_file is true, we should sleep for the difference between now
        // and the next timestamp. Right now we are just sleeping so we keep at the target frame rate.
        loop_sleep.sleep();
    }
    shutdown_tx.send(Box::new(ShutdownMsg{}))?;

    Ok(())
}

fn generate_image_paths(img_dir: String) -> Vec<String> {
    let mut glob_str = img_dir.to_owned();
    glob_str.push_str("/*.png");
    let mut img_paths = Vec::new();

    for entry in glob(&glob_str).expect("Failed to read glob pattern") {
        match entry {
            Ok(path) => match path.to_str() {
                Some(path_str) => img_paths.push(path_str.to_owned()),
                None => panic!("Invalid path found!"),
            },
            Err(e) => panic!("{:?}", e),
        }
    }
    img_paths
}

fn read_timestamps_file(img_dir: &String) -> Vec<f64> {
    let file = File::open(img_dir.clone() + "/times.txt")
        .expect("Could not open timestamps file");
    io::BufReader::new(file).lines()
        .map(|x| x.unwrap().parse::<f64>().unwrap())
        .collect::<Vec<f64>>()
}

enum LoopType {
    Fps(LoopHelper),
    Timestamps(Vec<f64>, i32, time::Instant),
}

struct LoopSleep {
    loop_type: LoopType
}
impl LoopSleep {
    pub fn new(img_dir: &String) -> Self {
        let loop_type = match SETTINGS.get::<bool>(SYSTEM, "use_timestamps_file") {
            true => {
                // Use dataset timestamps file to run loop
                let timestamps = read_timestamps_file(img_dir);
                LoopType::Timestamps(timestamps, -1, time::Instant::now())
            },
            false => {
                // Run loop at fps rate
                LoopType::Fps(
                    LoopHelper::builder()
                        .build_with_target_rate(SETTINGS.get::<f64>(SYSTEM, "fps"))
                )
            }
        };
        LoopSleep { loop_type }
    }

    pub fn start(&mut self) {
        match &mut self.loop_type {
            LoopType::Timestamps(_timestamps, ref mut _current_index, ref mut _now) => {
                todo!("Compile error with using timestamps file");
                // now = &mut time::Instant::now();
                // current_index = &mut (*current_index + 1);
            },
            LoopType::Fps(ref mut loop_helper) => {
                let _delta = loop_helper.loop_start(); 
            },
        }
    }

    pub fn sleep(&mut self) {
        match &mut self.loop_type {
            LoopType::Timestamps(timestamps, current_index, now) => {
                if *current_index == (timestamps.len() - 1) as i32 {
                    return;
                }
                let next_timestamp = timestamps[(*current_index + 1) as usize];
                thread::sleep(Duration::from_secs_f64(next_timestamp - now.elapsed().as_secs_f64()));
            },
            LoopType::Fps(ref mut loop_helper) => {
                loop_helper.loop_sleep(); 
            }
        }
    }
}


fn setup_logger(level: &str) -> Result<(), fern::InitError> {
    let colors = ColoredLevelConfig::new()
        .info(Color::Green)
        .warn(Color::Yellow)
        .error(Color::Red)
        .trace(Color::Magenta);

    let results_folder = SETTINGS.get::<String>(SYSTEM, "results_folder");

    let log_level = match level {
        "trace" => log::LevelFilter::Trace,
        "debug" => log::LevelFilter::Debug,
        "info" => log::LevelFilter::Info,
        "warn" => log::LevelFilter::Warn,
        "error" => log::LevelFilter::Error,
        _ => log::LevelFilter::Trace,
    };
    // Two loggers in chain - one for terminal output (colored, easier to read)
    // and another for file output (not colored, info on each line deliminated by |
    // for easier parsing in python scripts)
    let start_time = chrono::Local::now();
    fern::Dispatch::new()
    .level(log_level)
    .chain(
    fern::Dispatch::new()
            .format(move |out, message, record| {
                out.finish(format_args!(
                    "{color_line}[{time}][{target}][{level}{color_line}] {message}\x1B[0m",
                    color_line = format_args!(
                        "\x1B[{}m",
                        colors.get_color(&record.level()).to_fg_str()
                    ),
                    level = colors.color(record.level()),
                    time = (chrono::Local::now() - start_time).num_milliseconds() as f64 / 1000.0,
                    target = record.target(),
                    message = message
                ))
            })
            .chain(std::io::stdout()),
    )
    .chain(
    fern::Dispatch::new()
            .format(move |out, message, record| {
                out.finish(format_args!(
                    "{time}|{target}|{level}|{message}",
                    time = (chrono::Local::now() - start_time).num_milliseconds() as f64  / 1000.0,
                    target = record.target(),
                    level = record.level(),
                    message = message
                ))
            })
            .chain(OpenOptions::new()
                .write(true)
                .create(true)
                .open(Path::new(&results_folder).join("output.log"))?
            )
    )
    .apply()?;

    Ok(())
}
