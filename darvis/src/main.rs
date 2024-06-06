use std::{env, fs::{File, OpenOptions}, io::{self, BufRead}, path::Path, sync::atomic::AtomicBool, thread::{self, sleep}, time::{self, Duration, SystemTime}};
use map::map::Map;
use fern::colors::{ColoredLevelConfig, Color};
use glob::glob;
use log::{debug, info};
use spin_sleep::LoopHelper;
#[macro_use] extern crate lazy_static;

use core::{*, config::*, system::System, read_only_lock::ReadWriteMap};
use crate::{actors::messages::{ImageMsg, ShutdownMsg}, modules::{image, optimizer}};
use crate::map::map::Id;

mod actors;
mod registered_actors;
mod spawn;
mod map;
mod modules;
mod tests;

pub type MapLock = ReadWriteMap<Map>;
// pub type MapLock = Arc<Mutex<Map>>; // Replace above line with this if you want to switch all locks to mutexes

// To profile memory usage with tracy
// use tracy_client::*;
// #[global_allocator]
// static GLOBAL: ProfiledAllocator<std::alloc::System> =
//     ProfiledAllocator::new(std::alloc::System, 100);

pub static MAP_INITIALIZED: AtomicBool = AtomicBool::new(false); // Hack to wait processing frames until map has initialized

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() < 4 {
        panic!("
            [ERROR] Invalid number of input parameters.
            Usage: cargo run -- [PATH_TO_DATA_DIRECTORY] [PATH_TO_SYSTEM_CONFIG_FILE] [PATH_TO_DATASET/CAMERA_CONFIG_FILE] [DATASET_NAME]
        ");
    }
    let dataset_dir = args[1].to_owned();
    let system_config_file = args[2].to_owned();
    let dataset_config_file = args[3].to_owned();
    let dataset_name = args[4].to_owned();

    info!("Using dataset {:?}", dataset_name);

    // Load config, including custom settings and actor information
    let (actor_info, module_info, log_level) = load_config(&system_config_file, &dataset_config_file).expect("Could not load config");

    setup_logger(&log_level)?;

    let _client = tracy_client::Client::start();
    tracy_client::set_thread_name!("main");

    // Launch actor system
    let first_actor_name = SETTINGS.get::<String>(SYSTEM, "first_actor_name"); // Actor that launches the pipeline
    let (shutdown_flag, first_actor_tx, shutdown_tx, shutdown_join) = spawn::launch_system(actor_info, module_info, first_actor_name)?;

    info!("System ready to receive messages!");

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
    let loop_manager = LoopManager::new(dataset_name, dataset_dir);
    let mut sent_map_init = false;
    for (image_path, timestamp, frame_id) in loop_manager.into_iter() {
        if sent_map_init && !MAP_INITIALIZED.load(std::sync::atomic::Ordering::SeqCst) {
            sleep(Duration::from_millis(500));
        }

        if *shutdown_flag.lock().unwrap() { break; }
        tracy_client::frame_mark();

        let image = image::read_image_file(&image_path);

        first_actor_tx.send(Box::new(
            ImageMsg{
                image,
                timestamp,
                frame_id
            }
        ))?;

        if frame_id > 2 && !sent_map_init {
            sent_map_init = true;
        }
    }

    debug!("Done with dataset! Shutting down.");
    shutdown_tx.send(Box::new(ShutdownMsg{}))?;
    shutdown_join.join().expect("Waiting for shutdown thread");

    Ok(())
}

struct LoopManager {
    loop_helper: LoopHelper,
    image_paths: Vec<String>,
    timestamps: Vec<f64>,
    current_index: u32,
}

impl LoopManager {
    pub fn new(dataset: String, dataset_dir: String) -> Self {
        let (timestamps, img_dir);
        if dataset == "kitti" {
            img_dir = dataset_dir.clone() + "/image_0";
            timestamps = Self::read_timestamps_file_kitti(&dataset_dir);
        } else if dataset == "euroc" {
            img_dir = dataset_dir.clone() + "/data";
            timestamps = Self::read_timestamps_file_euroc(&dataset_dir);
        } else if dataset == "tum" {
            img_dir = dataset_dir.clone() + "/rgb";
            timestamps = Self::read_timestamps_file_tum(&dataset_dir);
        }  else {
            panic!("Invalid dataset name");
        };

        LoopManager { 
            loop_helper: LoopHelper::builder().build_with_target_rate(SETTINGS.get::<f64>(SYSTEM, "fps")),
            image_paths: Self::generate_image_paths(img_dir),
            timestamps,
            current_index: 0
        }
    }

    fn read_timestamps_file_kitti(time_stamp_dir: &String) -> Vec<f64> {
        let file = File::open(time_stamp_dir.clone() + "/times.txt")
            .expect("Could not open timestamps file");
        io::BufReader::new(file).lines()
            .map(|x| x.unwrap().parse::<f64>().unwrap())
            .collect::<Vec<f64>>()
    }

    fn read_timestamps_file_euroc(time_stamp_dir: &String) -> Vec<f64> {
        info!("Reading timestamps file {}", time_stamp_dir.clone());
        let file = File::open(time_stamp_dir.clone() + "/data.csv")
            .expect("Could not open timestamps file");
        io::BufReader::new(file).lines().skip(1)
            .map(|x| x.unwrap().split(',').next().unwrap().parse::<f64>().unwrap())
            .collect::<Vec<f64>>()
    }

    fn read_timestamps_file_tum(time_stamp_dir: &String) -> Vec<f64> {
        info!("Reading timestamps file {}", time_stamp_dir.clone());
        let file = File::open(time_stamp_dir.clone() + "/rgb.txt")
            .expect("Could not open timestamps file");
        io::BufReader::new(file).lines().skip(3)
            .map(|x| x.unwrap().split(' ').next().unwrap().parse::<f64>().unwrap())
            .collect::<Vec<f64>>()
    }

    fn generate_image_paths(img_dir: String) -> Vec<String> {
        let mut glob_str = img_dir.to_owned();
        glob_str.push_str("/*.png");
        let mut image_paths = Vec::new();

        for entry in glob(&glob_str).expect("Failed to read glob pattern") {
            match entry {
                Ok(path) => match path.to_str() {
                    Some(path_str) => image_paths.push(path_str.to_owned()),
                    None => panic!("Invalid path found!"),
                },
                Err(e) => panic!("{:?}", e),
            }
        }
        image_paths
    }
}

impl Iterator for LoopManager {
    type Item = (String, f64, u32);

    fn next(&mut self) -> Option<Self::Item> {
        if self.current_index as usize == self.image_paths.len() - 1{
            return None;
        }

        // First, sleep until the next timestamp
        self.loop_helper.loop_sleep(); 

        self.current_index = self.current_index + 1;
        let timestamp = self.timestamps[self.current_index as usize] * 1000000000.0;
        let image = self.image_paths[self.current_index as usize].clone();

        // Start next loop
        self.loop_helper.loop_start(); 

        Some((image, timestamp, self.current_index))
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
        .level_for("foxglove_ws", log::LevelFilter::Warn)
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
