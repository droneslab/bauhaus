#![feature(map_many_mut)]
extern crate flame;
use std::{fs::{File, OpenOptions}, io::Write, path::Path, sync::{Arc, Mutex}, env};
use axiom::prelude::*;
use chrono::{DateTime, Utc};
use fern::colors::{ColoredLevelConfig, Color};
use glob::glob;
use log::{warn, info, debug};
use spin_sleep::LoopHelper;
#[macro_use]
extern crate lazy_static;
use opencv::{imgcodecs, prelude::*};

use dvcore::{*, lockwrap::ReadWriteWrapper, config::*};
use crate::actors::{messages::{ImageMsg, ShutdownMessage, TrajectoryMessage, VisPathMsg}};
use crate::dvmap::{bow::VOCABULARY, map_actor::MapActor, pose::Pose, map::Map, map::Id};
use crate::registered_modules::{TRACKING_FRONTEND, VISUALIZER, SHUTDOWN, FeatureManager};

mod actors;
mod registered_modules;
mod dvmap;
mod modules;

pub static RESULTS_FOLDER: &str = "results";

fn main() {
    setup_logger().unwrap();
    let args: Vec<String> = env::args().collect();
    if args.len() < 3 {
        println!("[ERROR] Invalid number of input parameters.");
        println!("Usage: cargo run -- [PATH_TO_DATA_DIRECTORY] [PATH_TO_CONFIG_FILE]");
        return;
    }
    let img_dir = args[1].to_owned();
    let config_file = args[2].to_owned();

    if let Some((actor_info, module_info)) = load_config(&config_file) {
        // Load config, including custom settings and actor information
        let img_paths = generate_image_paths(img_dir);
        let writeable_map = ReadWriteWrapper::new(Map::new());
        let actor_system = initialize_actor_system(actor_info, &writeable_map);
        let _map_actor_aid = MapActor::spawn(&actor_system, writeable_map);
        VOCABULARY.access(); // Loads vocabulary
        let shutdown_flag = create_shutdown_actor(&actor_system);  // Handle ctrl+c
        info!("System ready to receive messages");

        let use_visualizer = GLOBAL_PARAMS.get::<bool>(SYSTEM_SETTINGS, "show_ui");
        // Run loop at fps rate
        let mut loop_helper = LoopHelper::builder()
            .report_interval_s(0.5)
            .build_with_target_rate(GLOBAL_PARAMS.get::<f64>(SYSTEM_SETTINGS, "fps"));

        // Process images
        for path in &img_paths {
            if *shutdown_flag.lock().unwrap() { break; }
            let _delta = loop_helper.loop_start(); 
            let img = imgcodecs::imread(&path, imgcodecs::IMREAD_GRAYSCALE).unwrap();
            let tracking_frontend = actor_system.find_aid_by_name(TRACKING_FRONTEND).unwrap();

            info!("Read image {}", path);
            if use_visualizer {
                let vis_id = actor_system.find_aid_by_name(VISUALIZER).unwrap();
                vis_id.send_new(VisPathMsg::new(path.to_string())).unwrap();
            }

            warn!("TODO... Check ORBSLAM3 frame loader, image gets scaled and resized");
            // After looking into it for a while, I think not. They have the code to scale,
            // but then never set the variable mImageScale to anything but 1.0
            // https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Examples/RGB-D/rgbd_tum.cc#L89

            tracking_frontend.send_new(ImageMsg{frame: img.into(),}).unwrap();
            loop_helper.loop_sleep(); 
        }
        actor_system.find_aid_by_name(SHUTDOWN).unwrap().send_new(ShutdownMessage{}).unwrap();
    } else {
        panic!("Could not load config");
    }
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

fn initialize_actor_system(modules: Vec::<base::ActorConf>, writeable_map: &ReadWriteWrapper<Map>) -> ActorSystem {
    let system = ActorSystem::create(ActorSystemConfig::default());

    //TODO (low priority): Use Cluster manager to do remote agent calls

    // Loop through the config to initialize actor system using the default config
    for actor_conf in modules {
        info!("Spawning actor;{}", &actor_conf.name);

        // Note: read_only() is important here, otherwise all actors can
        // access the write lock of the map
        let _ = system.spawn().name(&actor_conf.name).with(
            FeatureManager::new(
                &actor_conf.actor_function,
                &actor_conf.actor_function, 
                writeable_map.read_only()
            ),
            FeatureManager::handle
        ).unwrap();

        // TODO (low priority): Identify the role of using connect_with_channels, as the system communications are working without doing the following.
        //features.push(actname.clone());
        // if i > 0 { 
        //     ActorSystem::connect_with_channels(&systems.get(&features[0]).unwrap(), &system_current);            
        // }
        //i+=1;
    }

    system
}

fn create_shutdown_actor(actor_system: &ActorSystem) -> Arc<Mutex<bool>> {
    let shutdown_flag = Arc::new(Mutex::new(false));
    let flag_clone = shutdown_flag.clone();

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    struct TrackingPoses {
        trajectory_poses: Vec<Pose>, //mlRelativeFramePoses
        trajectory_times: Vec<DateTime<Utc>>, //mlFrameTimes
        trajectory_keyframes: Vec<Id>, //mlpReferences
    }
    let state = TrackingPoses{
        trajectory_poses: Vec::new(),
        trajectory_times: Vec::new(),
        trajectory_keyframes: Vec::new()
    };

    // Creating shutdown actor allows us to use actor_system
    // after ctrlc handler moves data into its handle function.
    let shutdown_actor = actor_system
    .spawn()
    .name(SHUTDOWN)
    .with(
        state,
        |mut state: TrackingPoses, context: Context, message: Message| async move {
            if let Some(_) = message.content_as::<ShutdownMessage>() {
                warn!("Triggered shutdown, saving trajectory info");
                let mut file = File::create(
                    Path::new(RESULTS_FOLDER)
                    .join(GLOBAL_PARAMS.get::<String>(SYSTEM_SETTINGS, "trajectory_file_name"))
                ).unwrap();
                for i in 0..state.trajectory_poses.len() {
                    let string = format!("{:?} {:?}", state.trajectory_times[i], state.trajectory_poses[i]);
                    file.write_all(string.as_bytes())?;
                }

                if GLOBAL_PARAMS.get::<bool>(SYSTEM_SETTINGS, "should_profile") {
                    flame::dump_html(File::create(
                        Path::new(RESULTS_FOLDER).join("flamegraph.html")
                    ).unwrap()).unwrap();
                }

                context.system.trigger_and_await_shutdown(None);
            } else if let Some(message) = message.content_as::<TrajectoryMessage>() {
                    match message.pose {
                        Some(pose) => {
                            state.trajectory_poses.push(pose);
                            state.trajectory_times.push(message.timestamp.unwrap());
                            state.trajectory_keyframes.push(message.ref_kf_id.unwrap());
                        },
                        None => {
                            // This can happen if tracking is lost. Duplicate last element of each vector
                            if let Some(last) = state.trajectory_poses.last().cloned() { state.trajectory_poses.push(last); }
                            if let Some(last) = state.trajectory_times.last().cloned() { state.trajectory_times.push(last); }
                            if let Some(last) = state.trajectory_keyframes.last().cloned() { state.trajectory_keyframes.push(last); }
                        }
                    };
            }
            Ok(Status::done(state))
        }
    )
    .expect("Error creating shutdown actor");

    ctrlc::set_handler(move || {
        warn!("received Ctrl+C!");
        *shutdown_flag.lock().unwrap() = true;
        shutdown_actor.send_new(ShutdownMessage{}).unwrap();
    })
    .expect("Error setting Ctrl-C handler");

    flag_clone
}

fn setup_logger() -> Result<(), fern::InitError> {
    let colors = ColoredLevelConfig::new()
        .info(Color::Green)
        .warn(Color::Yellow)
        .error(Color::Red);

    // Two loggers in chain - one for terminal output (colored, easier to read)
    // and another for file output (not colored, info on each line deliminated by |
    // for easier parsing in python scripts)
    let start_time = chrono::Local::now();
    fern::Dispatch::new()
    .level(log::LevelFilter::Debug)
    .level_for("axiom", log::LevelFilter::Warn)
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
                    "{time}|{target}|{message}",
                    time = (chrono::Local::now() - start_time).num_milliseconds() as f64  / 1000.0,
                    target = record.target(),
                    message = message
                ))
            })
            .chain(OpenOptions::new()
                .write(true)
                .create(true)
                .open(Path::new(RESULTS_FOLDER).join("output.log"))?
            )
    )
    .apply()?;

    Ok(())
}
