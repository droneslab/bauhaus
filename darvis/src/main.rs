use std::{collections::VecDeque, env, fs::{File, OpenOptions}, io::{self, BufRead}, path::Path, sync::atomic::AtomicBool, thread::{self, sleep}, time::Duration};
use fern::colors::{ColoredLevelConfig, Color};
use glob::glob;
use log::{debug, info, warn};
use modules::imu::{ImuMeasurements, ImuPoint};
use opencv::core::Point3f;
use spin_sleep::LoopHelper;
#[macro_use] extern crate lazy_static;

use core::{config::*, sensor::Sensor, system::System, *};
use crate::{actors::messages::{ImageMsg, ShutdownMsg}, modules::image};
use crate::map::map::Id;

mod actors;
mod registered_actors;
mod spawn;
mod map;
mod modules;
mod tests;

// pub type ReadWriteMap = Arc<Mutex<Map>>; // Replace above line with this if you want to switch all locks to mutexes

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
        // mutexes. You can do that pretty quickly by modifying pub type ReadWriteMap in the 
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
    let read_imu = SETTINGS.get::<Sensor>(SYSTEM, "sensor").is_imu();

    let loop_manager = LoopManager::new(dataset_name, dataset_dir, read_imu);
    let mut sent_map_init = false;
    for (image_path, imu_measurements, timestamp, frame_id) in loop_manager.into_iter() {
        if sent_map_init && !MAP_INITIALIZED.load(std::sync::atomic::Ordering::SeqCst) {
            sleep(Duration::from_millis(200));
        }

        if *shutdown_flag.lock().unwrap() { break; }
        tracy_client::frame_mark();

        debug!("Read image {}", image_path);

        let image = image::read_image_file(&image_path);

        first_actor_tx.send(Box::new(
            ImageMsg{
                image,
                timestamp,
                imu_measurements,
                frame_id
            }
        ))?;

        if frame_id > 5 && !sent_map_init {
            sent_map_init = true;
        }
    }

    debug!("Done with dataset! Shutting down.");
    shutdown_tx.send(Box::new(ShutdownMsg{}))?;
    shutdown_join.join().expect("Waiting for shutdown thread");

    Ok(())
}

struct ImuData {
    acceleration: Vec<Point3f>,
    gyro: Vec<Point3f>,
    timestamps: Vec<f64>,
    first_imu_idx: usize,
}

struct LoopManager {
    loop_helper: LoopHelper,
    image_paths: Vec<String>,
    timestamps: Vec<f64>,
    current_index: u32,
    imu: Option<ImuData>,
}

impl LoopManager {
    pub fn new(dataset: String, dataset_dir: String, read_imu: bool) -> Self {
        let (mut timestamps, img_dir);
        if dataset == "kitti" {
            img_dir = dataset_dir.clone() + "/image_0";
            timestamps = Self::read_timestamps_file_kitti(&dataset_dir);
        } else if dataset == "euroc" {
            img_dir = dataset_dir.clone() + "/mav0/cam0/data";
            timestamps = Self::read_timestamps_file_euroc(&dataset_dir);
        } else if dataset == "tum" {
            img_dir = dataset_dir.clone() + "/rgb";
            timestamps = Self::read_timestamps_file_tum(&dataset_dir);
        } else if dataset == "tum-vi" {
            img_dir = dataset_dir.clone() + "/mav0/cam0/data";
            timestamps = Self::read_timestamps_file_tum(&dataset_dir);
        }  else {
            panic!("Invalid dataset name");
        };

        let mut image_paths = Self::generate_image_paths(img_dir);

        let imu = {
            if !read_imu {
                None
            } else {
                if dataset == "kitti" {
                    warn!("KITTI dataset does not have IMU data.");
                    None
                } else if dataset == "euroc" {
                    let imu_file = dataset_dir.clone() + "/mav0/imu0/data.csv";
                    Some(Self::read_imu_file(imu_file, &mut timestamps, &mut image_paths).expect("Could not read IMU file!"))
                } else if dataset == "tum-vi" {
                    let imu_file = dataset_dir.clone() + "/mav0/imu0/data.csv";
                    Some(Self::read_imu_file(imu_file, &mut timestamps, &mut image_paths).expect("Could not read IMU file!"))
                } else {
                    panic!("Invalid dataset name");
                }
            }
        };

        LoopManager { 
            loop_helper: LoopHelper::builder().build_with_target_rate(SETTINGS.get::<f64>(SYSTEM, "fps")),
            image_paths,
            timestamps,
            imu,
            current_index: 0
        }
    }

    fn read_imu_file(filename: String, camera_timestamps: &mut Vec<f64>, image_paths: &mut Vec<String>) -> Result<ImuData, Box<dyn std::error::Error>>{
        let file = File::open(filename)?;
        let mut rdr = csv::Reader::from_reader(file);
        let mut imu_data = ImuData {
            acceleration: Vec::new(),
            gyro: Vec::new(),
            timestamps: Vec::new(),
            first_imu_idx: 0,
        };

        for result in rdr.records() {
            let record = result?;

            imu_data.gyro.push(Point3f::new(
                record[1].parse::<f32>().unwrap(),
                record[2].parse::<f32>().unwrap(),
                record[3].parse::<f32>().unwrap()
            ));

            imu_data.acceleration.push(Point3f::new(
                record[4].parse::<f32>().unwrap(),
                record[5].parse::<f32>().unwrap(),
                record[6].parse::<f32>().unwrap()
            ));

            imu_data.timestamps.push(record[0].parse::<f64>().unwrap() / 1000000000000000000.0);
        }

        // Find first imu to be considered, supposing imu measurements start first
        let mut index = 0;
        if imu_data.timestamps[index] > camera_timestamps[0] {
            // Note: for some reason some euroc sequences have one camera image/timestamp
            // that is earlier than the IMU data, so just skip that image
            warn!("IMU data starts after camera data! Starting with second camera image instead");
            camera_timestamps.remove(0);
            image_paths.remove(0);
        }
        let first_timestamp_in_camera = camera_timestamps[0];

        while imu_data.timestamps[index] < first_timestamp_in_camera {
            index += 1;
        }
        imu_data.first_imu_idx = index - 1;

        Ok(imu_data)
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
        let file = File::open(time_stamp_dir.clone() + "/mav0/cam0/data.csv")
            .expect("Could not open timestamps file");
        io::BufReader::new(file).lines().skip(1)
            .map(|x| x.unwrap().split(',').next().unwrap().parse::<f64>().unwrap() / 1000000000000000000.0)
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
    type Item = (String, ImuMeasurements, f64, u32);

    fn next(&mut self) -> Option<Self::Item> {
        if self.current_index as usize == self.image_paths.len() - 1{
            return None;
        }

        // First, sleep until the next timestamp
        self.loop_helper.loop_sleep(); 

        let timestamp = self.timestamps[self.current_index as usize];
        let image = self.image_paths[self.current_index as usize].clone();

        let mut imu_measurements = VecDeque::new();
        if let Some(imu) = &mut self.imu {
            // Load imu measurements from previous frame
            while imu.timestamps[imu.first_imu_idx] <= timestamp {
                imu_measurements.push_back(ImuPoint{
                    acc: nalgebra::Vector3::new(
                        imu.acceleration[imu.first_imu_idx].x as f64,
                        imu.acceleration[imu.first_imu_idx].y as f64,
                        imu.acceleration[imu.first_imu_idx].z as f64
                    ),
                    ang_vel: nalgebra::Vector3::new(
                        imu.gyro[imu.first_imu_idx].x as f64,
                        imu.gyro[imu.first_imu_idx].y as f64,
                        imu.gyro[imu.first_imu_idx].z as f64
                    ),
                    timestamp: imu.timestamps[imu.first_imu_idx]
                });
                imu.first_imu_idx += 1;
            }
            // imu.first_imu_idx -= 1;
        };

        // Start next loop
        self.loop_helper.loop_start(); 
        self.current_index = self.current_index + 1;

        Some((image, imu_measurements, timestamp, self.current_index))
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
