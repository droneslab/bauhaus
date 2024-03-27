#![feature(map_many_mut)]
#![feature(hash_extract_if)]
#![feature(extract_if)]

use std::{fs::{OpenOptions, File}, path::Path, env, time::{self, Duration}, io::{self, BufRead}, thread};
use map::map::Map;
use fern::colors::{ColoredLevelConfig, Color};
use glob::glob;
use log::{debug, info};
use spin_sleep::LoopHelper;
#[macro_use] extern crate lazy_static;

use core::{*, config::*, system::System, read_only_lock::ReadWriteMap};
use crate::{actors::messages::{ShutdownMsg, ImageMsg}, modules::image};
use crate::map::{map::Id};

mod actors;
mod registered_actors;
mod spawn;
mod map;
mod modules;
mod tests;
use core::config::load_config;

pub type MapLock = ReadWriteMap<Map>;
// pub type MapLock = Arc<Mutex<Map>>; // Replace above line with this if you want to switch all locks to mutexes

// To profile memory usage with tracy
// use tracy_client::*;
// #[global_allocator]
// static GLOBAL: ProfiledAllocator<std::alloc::System> =
//     ProfiledAllocator::new(std::alloc::System, 100);


fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() < 3 {
        panic!("
            [ERROR] Invalid number of input parameters.
            Usage: cargo run -- [PATH_TO_DATA_DIRECTORY] [PATH_TO_SYSTEM_CONFIG_FILE] [PATH_TO_DATASET/CAMERA_CONFIG_FILE]
        ");
    }
    let img_dir = args[1].to_owned();
    let system_config_file = args[2].to_owned();
    let dataset_config_file = args[3].to_owned();

    // Load config, including custom settings and actor information
    let (actor_info, module_info, log_level) = load_config(&system_config_file, &dataset_config_file).expect("Could not load config");

    setup_logger(&log_level)?;

    let _client = tracy_client::Client::start();
    tracy_client::set_thread_name!("main");

    // Launch actor system
    let first_actor_name = SETTINGS.get::<String>(SYSTEM, "first_actor_name"); // Actor that launches the pipeline
    let (shutdown_flag, first_actor_tx, shutdown_tx, shutdown_join) = spawn::launch_system(actor_info, module_info, first_actor_name)?;

    info!("System ready to receive messages!");

    let mut loop_sleep = LoopSleep::new(&img_dir);
    let timestamps = read_timestamps_file(&img_dir);

    if SETTINGS.get::<bool>(SYSTEM, "check_deadlocks") {
        // This slows stuff down, so only enable this if you really need to.
        // This seems to find deadlocks more consistently if you turn all rwlocks into
        // mutexes. You can do that pretty quickly by modifying pub type MapLock in the 
        // beginning of this file and turning usages of read() and write() into lock().
        thread::spawn(move || { 
            loop {
                let deadlocks = parking_lot::deadlock::check_deadlock();
                if !deadlocks.is_empty() {
                    println!("{} deadlocks detected", deadlocks.len());
                    for (i, threads) in deadlocks.iter().enumerate() {
                        println!("Deadlock #{}", i);
                        for t in threads {
                            println!("Thread Id {:#?}", t.thread_id());
                            println!("{:#?}", t.backtrace());
                        }
                    }
                }
                thread::sleep(Duration::from_secs_f64(2.0));
            }
        } );
    }

    // Process images
    // let now = SystemTime::now();
    let mut i = 0;
    for path in &generate_image_paths(img_dir + "/image_0/") {
        if *shutdown_flag.lock().unwrap() { break; }
        loop_sleep.start();
        tracy_client::frame_mark();

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
    debug!("Done with dataset! Shutting down.");
    shutdown_tx.send(Box::new(ShutdownMsg{}))?;
    shutdown_join.join().expect("Waiting for shutdown thread");

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
                todo!("timestamps: Compile error with using timestamps file");
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

    let start_time = chrono::Local::now();

    let terminal_output = fern::Dispatch::new()
        .level(log_level) // Turns off all logging for external crates, some can be very noisy
        .level_for("foxglove", log::LevelFilter::Warn)
        .format(move |out, message, record| {
            out.finish(format_args!(
                "{color_line}[{time} {target}:{line_num} {level}{color_line}] {message}\x1B[0m",
                color_line = format_args!(
                    "\x1B[{}m",
                    colors.get_color(&record.level()).to_fg_str()
                ),
                level = colors.color(record.level()),
                time = (chrono::Local::now() - start_time).num_milliseconds() as f64 / 1000.0,
                target = record.file().unwrap_or("unknown"),
                line_num = record.line().unwrap_or(0),
                message = message
            ))
        })
        .chain(std::io::stdout());

    let file_output = fern::Dispatch::new()
        .level(log_level)
        .format(move |out, message, record| {
            out.finish(format_args!(
                "[{time} {target}:{line_num} {level}] {message}",
                time = (chrono::Local::now() - start_time).num_milliseconds() as f64  / 1000.0,
                target = record.file().unwrap_or("unknown"),
                line_num = record.line().unwrap_or(0),
                level = record.level(),
                message = message
            ))
        })
        .chain(OpenOptions::new()
            .write(true)
            .truncate(true)
            .create(true)
            .open(Path::new(&results_folder).join("output.log"))?
        );
        
    // let trace_output = fern::Dispatch::new()
    //     .level(log::LevelFilter::Trace)
    //     .format(move |out, message, record| {
    //         out.finish(format_args!(
    //             "[{time} {target}:{line_num} {level}] {message}",
    //             time = (chrono::Local::now() - start_time).num_milliseconds() as f64  / 1000.0,
    //             target = record.file().unwrap_or("unknown"),
    //             line_num = record.line().unwrap_or(0),
    //             level = record.level(),
    //             message = message
    //         ))
    //     })
    //     .chain(OpenOptions::new()
    //         .write(true)
    //         .truncate(true)
    //         .create(true)
    //         .open(Path::new(&results_folder).join("trace.log"))?
    //     );

    fern::Dispatch::new()
    .chain(terminal_output)
    .chain(file_output)
    // .chain(trace_output)
    .apply()?;


    Ok(())
}
