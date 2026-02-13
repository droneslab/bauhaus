use fern::colors::{Color, ColoredLevelConfig};
use glob::glob;
use log::{debug, info, warn};
use modules::imu::{ImuMeasurements, ImuPoint};
use opencv::{core::Point3f, imgcodecs};
use registered_actors::CAMERA;
use spin_sleep::LoopHelper;
use std::{
    collections::{BTreeMap, VecDeque},
    env,
    fs::File,
    io::{self, BufRead},
    sync::atomic::AtomicBool,
    thread::{self, sleep},
    time::Duration,
};
#[macro_use]
extern crate lazy_static;

use crate::{map::map::Id, modules::imu::ImuBias};
use crate::{
    actors::messages::{ImageMsg, ShutdownMsg},
    map::pose::Pose,
    modules::image,
};
use core::{config::*, matrix::BHVector3, sensor::Sensor, system::System, *};

mod actors;
mod map;
mod modules;
mod registered_actors;
mod spawn;
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

    // Load config, including custom settings and actor information
    let (actor_info, module_info, log_level) =
        load_config(&system_config_file, &dataset_config_file).expect("Could not load config");

    setup_logger(&log_level)?;
    info!("Using dataset {:?}", dataset_name);

    let _client = tracy_client::Client::start();
    tracy_client::set_thread_name!("main");

    // Launch actor system
    let first_actor_name = SETTINGS.get::<String>(SYSTEM, "first_actor_name"); // Actor that launches the pipeline
    let (shutdown_flag, first_actor_tx, shutdown_tx, shutdown_join) =
        spawn::launch_system(actor_info, module_info, first_actor_name)?;

    info!("System ready to receive messages!");

    if SETTINGS.get::<bool>(SYSTEM, "check_deadlocks") {
        // This slows stuff down, so only enable this if you really need to.
        // This seems to find deadlocks more consistently if you turn all rwlocks into
        // mutexes. You can do that pretty quickly by modifying pub type ReadWriteMap in the
        // beginning of this file and turning usages of read() and write() into lock().
        thread::spawn(move || loop {
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
        });
    }

    // Process images
    let read_imu = SETTINGS.get::<Sensor>(SYSTEM, "sensor").is_imu();

    let loop_manager = LoopManager::new(dataset_name, dataset_dir, read_imu);
    let mut sent_map_init = false;
    let mut current_index = 0;
    for (image_path, imu_measurements, imu_initialization, timestamp, frame_id) in
        loop_manager.into_iter()
    {
        // If first_k set, loop until we are at the first frame.
        if current_index < SETTINGS.get::<i32>(SYSTEM, "initial_k") {
            current_index += 1;
            continue;
        }

        // If final_k set, stop processing images after this time.
        if SETTINGS.get::<i32>(SYSTEM, "final_k") != 0
            && current_index == SETTINGS.get::<i32>(SYSTEM, "final_k")
        {
            debug!("Ending early at frame {}", current_index);
            break;
        }

        if sent_map_init && !MAP_INITIALIZED.load(std::sync::atomic::Ordering::SeqCst) {
            sleep(Duration::from_millis(200));
        }

        if *shutdown_flag.lock().unwrap() {
            break;
        }
        tracy_client::frame_mark();

        let mut image_bw = image::read_image_file(&image_path, imgcodecs::IMREAD_GRAYSCALE);
        let mut image_color = image::read_image_file(&image_path, imgcodecs::IMREAD_COLOR);
        if SETTINGS.get::<bool>(CAMERA, "need_to_resize") {
            let new_height = SETTINGS.get::<i32>(CAMERA, "height");
            let new_width = SETTINGS.get::<i32>(CAMERA, "width");
            image_bw = image::resize_image(&image_bw, new_width, new_height)
                .expect("Could not resize image!");
            image_color = image::resize_image(&image_color, new_width, new_height)
                .expect("Could not resize image!");
        }

        first_actor_tx.send(Box::new(ImageMsg {
            image: image_bw,
            color_image: Some(image_color),
            timestamp: (timestamp as f64) / 1e9,
            imu_measurements,
            imu_initialization,
            frame_id,
        }))?;

        if frame_id > 5 && !sent_map_init {
            sent_map_init = true;
        }

        current_index += 1;

        // NOTE: WHEN LOOP SLEEP IS TURNED OFF, SLEEPING HERE INSTEAD, SO WE CAN SKIP PAST THE INITIAL FRAMES FASTER
        // sleep(Duration::from_millis(
        //     1000 / SETTINGS.get::<f64>(SYSTEM, "fps") as u64,
        // ));
    }

    info!("Done with dataset! Shutting down.");
    shutdown_tx.send(Box::new(ShutdownMsg {}))?;
    shutdown_join.join().expect("Waiting for shutdown thread");

    Ok(())
}

#[derive(Debug, Clone)]
struct ImuInitializationData {
    pose: Pose,
    velocity: BHVector3<f64>,
    bias: ImuBias,
}

struct ImuData {
    acceleration: Vec<Point3f>,
    gyro: Vec<Point3f>,
    timestamps: Vec<i64>,
    first_imu_idx: usize,
    initialization: BTreeMap<i64, ImuInitializationData>,
}

struct LoopManager {
    loop_helper: LoopHelper,
    image_paths: Vec<String>,
    timestamps: Vec<i64>,
    current_index: u32,
    imu: Option<ImuData>,
}

impl LoopManager {
    pub fn new(dataset: String, dataset_dir: String, read_imu: bool) -> Self {
        let (mut timestamps, img_dir, mut image_paths);
        if dataset == "kitti" {
            img_dir = dataset_dir.clone() + "/image_0";
            timestamps = Self::read_timestamps_file_kitti(&dataset_dir);
            image_paths = Self::generate_image_paths(img_dir);
        } else if dataset == "euroc" {
            img_dir = dataset_dir.clone() + "/mav0/cam0/data";
            (timestamps, image_paths) = Self::read_timestamps_file_euroc(&dataset_dir, img_dir);
        } else if dataset == "tum" {
            img_dir = dataset_dir.clone() + "/rgb";
            timestamps = Self::read_timestamps_file_tum(&dataset_dir);
            image_paths = Self::generate_image_paths(img_dir);
        } else if dataset == "tum-vi" {
            img_dir = dataset_dir.clone() + "/mav0/cam0/data";
            timestamps = Self::read_timestamps_file_tum(&dataset_dir);
            image_paths = Self::generate_image_paths(img_dir);
        } else {
            panic!("Invalid dataset name");
        };

        let imu = {
            if !read_imu {
                None
            } else {
                if dataset == "kitti" {
                    let imu_file = dataset_dir.clone() + "imu.txt";
                    let gt_file = dataset_dir.clone() + "gt.txt";
                    Some(
                        Self::read_imu(imu_file, gt_file, &mut timestamps, &mut image_paths)
                            .expect("Could not read IMU file!"),
                    )

                    // None
                } else if dataset == "euroc" {
                    let imu_file = dataset_dir.clone() + "/mav0/imu0/data.csv";
                    let gt_file =
                        dataset_dir.clone() + "/mav0/state_groundtruth_estimate0/data.csv";
                    Some(
                        Self::read_imu(imu_file, gt_file, &mut timestamps, &mut image_paths)
                            .expect("Could not read IMU file!"),
                    )
                } else if dataset == "tum-vi" {
                    let imu_file = dataset_dir.clone() + "/mav0/imu0/data.csv";
                    let gt_file =
                        dataset_dir.clone() + "/mav0/state_groundtruth_estimate0/data.csv"; // TODO This may not be the right filename
                    Some(
                        Self::read_imu(imu_file, gt_file, &mut timestamps, &mut image_paths)
                            .expect("Could not read IMU file!"),
                    )
                } else {
                    panic!("Invalid dataset name");
                }
            }
        };

        LoopManager {
            loop_helper: LoopHelper::builder()
                .build_with_target_rate(SETTINGS.get::<f64>(SYSTEM, "fps")),
            image_paths,
            timestamps,
            imu,
            current_index: 0,
        }
    }

    fn read_imu(
        imu_filename: String,
        gt_filename: String,
        camera_timestamps: &mut Vec<i64>,
        image_paths: &mut Vec<String>,
    ) -> Result<ImuData, Box<dyn std::error::Error>> {
        let imu_file = File::open(imu_filename)?;
        let mut rdr = csv::Reader::from_reader(imu_file);
        let mut imu_data = ImuData {
            acceleration: Vec::new(),
            gyro: Vec::new(),
            timestamps: Vec::new(),
            first_imu_idx: 0,
            initialization: BTreeMap::new(),
        };

        for result in rdr.records() {
            let record = result?;

            imu_data.gyro.push(Point3f::new(
                record[1].parse::<f32>().unwrap(),
                record[2].parse::<f32>().unwrap(),
                record[3].parse::<f32>().unwrap(),
            ));

            imu_data.acceleration.push(Point3f::new(
                record[4].parse::<f32>().unwrap(),
                record[5].parse::<f32>().unwrap(),
                record[6].parse::<f32>().unwrap(),
            ));

            imu_data
                .timestamps
                .push(record[0].parse::<i64>().unwrap());
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

        println!("First timestamp in camera: {}", first_timestamp_in_camera);


        while imu_data.timestamps[index] <= first_timestamp_in_camera {
            index += 1;
        }
        imu_data.first_imu_idx = index - 1;
        println!("First imu index: {}", imu_data.first_imu_idx);

        // KIMERA
        let gt_file = File::open(gt_filename).unwrap();
        rdr = csv::Reader::from_reader(gt_file);
        for result in rdr.records() {
            let record = result?;

            let timestamp = record[0].parse::<i64>().unwrap();

            // let timestamp = (current_frame.timestamp * 1e9) as i64; // Convert to int just so we can hash it
            let translation = BHVector3::new_with(
                record[1].parse::<f64>().unwrap(),
                record[2].parse::<f64>().unwrap(),
                record[3].parse::<f64>().unwrap(),
            );
            let rotation = nalgebra::Vector4::new(
                record[4].parse::<f64>().unwrap(), // w
                record[5].parse::<f64>().unwrap(), // x
                record[6].parse::<f64>().unwrap(), // y
                record[7].parse::<f64>().unwrap(), // z
            );
            let velocity = BHVector3::new_with(
                record[8].parse::<f64>().unwrap(),
                record[9].parse::<f64>().unwrap(),
                record[10].parse::<f64>().unwrap(),
            );
            let gyro_bias = BHVector3::new_with(
                record[11].parse::<f64>().unwrap(),
                record[12].parse::<f64>().unwrap(),
                record[13].parse::<f64>().unwrap(),
            );
            let acc_bias = BHVector3::new_with(
                record[14].parse::<f64>().unwrap(),
                record[15].parse::<f64>().unwrap(),
                record[16].parse::<f64>().unwrap(),
            );

            imu_data.initialization.insert(
                timestamp,
                ImuInitializationData {
                    pose: Pose::new_with_quaternion_convert(*translation, rotation),
                    velocity,
                    bias: ImuBias::new_with(gyro_bias, acc_bias),
                },
            );
        }

        Ok(imu_data)
    }

    fn read_timestamps_file_kitti(time_stamp_dir: &String) -> Vec<i64> {
        let file = File::open(time_stamp_dir.clone() + "/times.txt")
            .expect("Could not open timestamps file");
        io::BufReader::new(file)
            .lines()
            .map(|x| x.unwrap().parse::<i64>().unwrap())
            .collect::<Vec<i64>>() // *1e16
    }

    fn read_timestamps_file_euroc(
        time_stamp_dir: &String,
        image_dir: String,
    ) -> (Vec<i64>, Vec<String>) {
        info!("Reading timestamps file {}", time_stamp_dir.clone());
        let file = File::open(time_stamp_dir.clone() + "/mav0/cam0/data.csv")
            .expect("Could not open timestamps file");
        let data = io::BufReader::new(file)
            .lines()
            .skip(1)
            .map(|x| {
                let x2 = x.unwrap();
                let mut x3 = x2.split(",");
                let timestamp_before_convert = x3.next().unwrap().parse::<i64>().unwrap();
                let filename = x3.next().unwrap().to_string();
                let filepath = format!("{}/{}", image_dir, filename);
                (timestamp_before_convert, filepath)
            })
            .collect::<Vec<(i64, String)>>();
        data.into_iter().map(|(a, b)| (a, b)).unzip()
    }

    fn read_timestamps_file_tum(time_stamp_dir: &String) -> Vec<i64> {
        info!("Reading timestamps file {}", time_stamp_dir.clone());
        let file = File::open(time_stamp_dir.clone() + "/rgb.txt")
            .expect("Could not open timestamps file");
        io::BufReader::new(file)
            .lines()
            .skip(3)
            .map(|x| {
                x.unwrap()
                    .split(' ')
                    .next()
                    .unwrap()
                    .parse::<i64>()
                    .unwrap()
            })
            .collect::<Vec<i64>>()
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
    type Item = (
        String,
        ImuMeasurements,
        Option<ImuInitializationData>,
        i64,
        u32,
    );

    fn next(&mut self) -> Option<Self::Item> {
        if self.current_index as usize == self.image_paths.len() - 1 {
            return None;
        }

        // First, sleep until the next timestamp
        self.loop_helper.loop_sleep();

        let timestamp = self.timestamps[self.current_index as usize];
        let image = self.image_paths[self.current_index as usize].clone();
        // debug!("TIMESTAMP IS {}", timestamp);

        let mut imu_measurements = VecDeque::new();
        let mut imu_initialization = None;
        if let Some(imu) = &mut self.imu {
            // Load imu measurements from previous frame
            while imu.timestamps[imu.first_imu_idx] <= timestamp {
                imu_measurements.push_back(ImuPoint {
                    acc: nalgebra::Vector3::new(
                        imu.acceleration[imu.first_imu_idx].x as f64,
                        imu.acceleration[imu.first_imu_idx].y as f64,
                        imu.acceleration[imu.first_imu_idx].z as f64,
                    ),
                    ang_vel: nalgebra::Vector3::new(
                        imu.gyro[imu.first_imu_idx].x as f64,
                        imu.gyro[imu.first_imu_idx].y as f64,
                        imu.gyro[imu.first_imu_idx].z as f64,
                    ),
                    timestamp: (imu.timestamps[imu.first_imu_idx] as f64) / 1e9,
                });
                imu.first_imu_idx += 1;
            }
            imu.first_imu_idx -= 1;

            // Kimera imu initialization values
            // let timestamp_convert = (timestamp * 1e9) as i64;
            let it_low = imu.initialization.range(timestamp..).next(); // closest, non-lesser
            if let Some((_timestamp_found, data)) = it_low {
                imu_initialization = Some(data);
            } else {
                debug!("Can't find timestamp! {}", timestamp);
            }
        };

        // Start next loop
        self.loop_helper.loop_start();
        self.current_index = self.current_index + 1;

        Some((
            image,
            imu_measurements,
            imu_initialization.cloned(),
            timestamp,
            self.current_index,
        ))
    }
}

fn setup_logger(level: &str) -> Result<(), fern::InitError> {
    let colors = ColoredLevelConfig::new()
        .info(Color::Green)
        .warn(Color::Yellow)
        .error(Color::Red)
        .trace(Color::Magenta);

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
                color_line =
                    format_args!("\x1B[{}m", colors.get_color(&record.level()).to_fg_str()),
                level = colors.color(record.level()),
                time = (chrono::Local::now() - start_time).num_milliseconds() as f64 / 1000.0,
                target = record.file().unwrap_or("unknown"),
                line_num = record.line().unwrap_or(0),
                message = message
            ))
        })
        .chain(std::io::stdout());

    fern::Dispatch::new()
        .chain(terminal_output)
        .apply()?;

    Ok(())
}
