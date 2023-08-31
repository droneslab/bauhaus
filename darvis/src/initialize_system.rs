/// *** Initialize actor system using config file.
// Returns mutex to shutdown flag and transmitters for first actor and shutdown actor.
// You probably don't want to change this code.

use std::{sync::{Arc, mpsc::{self, Receiver, Sender}, Mutex}, thread, collections::HashMap, time::Duration, any::Any, fs::File, path::Path, io::Write};

use chrono::{DateTime, Utc};
use dvcore::{
    config::{ActorConf, GLOBAL_PARAMS, SYSTEM_SETTINGS},
    base::{DVSender, DVReceiver, ActorSystem, DVMessageBox, ActorChannel, Actor},
    lockwrap::{ReadWriteWrapper, ReadOnlyWrapper}
};
use log::{info, warn};
use rerun::RecordingStreamBuilder;

use crate::{
    registered_modules::{VISUALIZER, SHUTDOWN_ACTOR, MAP_ACTOR, run_actor},
    dvmap::{map_actor::MapActor,
    map::{Map, Id}, pose::Pose},
    actors::tracking_frontend::DarvisTrackingFront,
    actors::messages::{ShutdownMessage, TrajectoryMessage}, RESULTS_FOLDER
};

pub struct ActorBuilder<A: Actor> { 
    pipe: Box<Receiver<A::MSG>>,
}

impl<A: Actor> ActorBuilder<A> {
    pub fn new() -> (Box<Sender<A::MSG>>, Self) {
        let (tx, rx) = mpsc::channel::<A::MSG>();
        (Box::new(tx), Self { pipe: Box::new(rx) })
    }
    pub fn build(handles: A::HANDLES) -> A {
        A::new(self.pipe, handles)
    }
}

pub enum Actors {
    TrackingFrontend(ActorBuilder<DarvisTrackingFront>),
}

pub fn initialize_actors(modules: Vec::<ActorConf>, first_actor_name: String) 
    -> Result<(Arc<std::sync::Mutex<bool>>, DVSender, DVSender), Box<dyn std::error::Error>> 
{
    // * SET UP CHANNELS *//
    // Create transmitters and receivers for user-defined actors
    let mut transmitters = HashMap::<String, Box::<dyn ActorChannel>>::new();
    let mut receivers = Vec::new();


    let mut builders = Vec::<Actors>::new();
    for actor_conf in &modules {
        if actor_conf.name == TRACKING_FRONTEND.to_string()  {
            let (handle, builder) = ActorBuilder::<DarvisTrackingFront>::new();
            builders.push(Actors::TrackingFrontend(builder));
            transmitters.insert(actor_conf.name.clone(), handle.clone());
        } else if false {
            // todo!("other cases");
        }
    }

    // build the actor system
    let actor_sys: ActorSystem = ActorSystem { actors: transmitters.clone() };

    for builder in builders {
        match builder {
            Actors::TrackingFrontend(builder) => {
                let actor = builder.build(actor_sys.clone());
                thread::spawn(move || { actor.run(); });

                
            }
        }
    }

    // Create transmitters/receivers for darvis system actors
    // Don't add receivers to the receivers vec because they need to be spawned separately.
    // User-defined actors still need the transmitters to these actors.
    // let (map_tx, map_rx) = mpsc::channel::<DVMessageBox>();
    // transmitters.insert(MAP_ACTOR.to_string(), map_tx);
    // let (shutdown_tx, shutdown_rx) = mpsc::channel::<DVMessageBox>();
    // transmitters.insert(SHUTDOWN_ACTOR.to_string(), shutdown_tx);
    // let vis_rx = match GLOBAL_PARAMS.get::<bool>(SYSTEM_SETTINGS, "show_visualizer") {
    //     true => {
    //         let (vis_tx, vis_rx) = mpsc::channel::<DVMessageBox>();
    //         transmitters.insert(VISUALIZER.to_string(), vis_tx);
    //         Some(vis_rx)
    //     },
    //     false => None
    // };

    // * SPAWN USER-DEFINED ACTORS *//
    // Create map
    let writeable_map = ReadWriteWrapper::new(Map::new());
    // Spawn actors
    for (actor_name, receiver) in receivers {
        info!("Spawning actor;{}", &actor_name);

        // Note: read_only() is important here, otherwise all actors can
        // access the write lock of the map
        let map_clone = writeable_map.read_only();

        let actor_system = make_actor_system(&actor_name, &transmitters, receiver);
        thread::spawn(move || { run_actor(actor_name, map_clone, actor_system) } );
    }

    // * SPAWN DARVIS SYSTEM ACTORS *//
    // Visualizer
    if GLOBAL_PARAMS.get::<bool>(SYSTEM_SETTINGS, "show_visualizer") {
        let actor_system = make_actor_system(&VISUALIZER.to_string(), &transmitters, vis_rx.unwrap());
        create_visualize_actor(actor_system, writeable_map.read_only())?;
    }

    // Map actor
    // After this point, you cannot get a read-only clone of the map!
    // So make sure to spawn all the other actors that need the map before this.
    let actor_system = make_actor_system(&MAP_ACTOR.to_string(), &transmitters, map_rx);
    thread::spawn(move || { 
        let mut map_actor = MapActor::new(actor_system, writeable_map);
        map_actor.run();
    } );

    // Ctrl+c shutdown actor
    let actor_system = make_actor_system(&SHUTDOWN_ACTOR.to_string(), &transmitters, shutdown_rx);
    let shutdown_flag = create_shutdown_actor(actor_system, transmitters.get(SHUTDOWN_ACTOR).unwrap().clone());

    //* Return transmitters for the shutdown actor and first actor in the pipeline, and the ctrl+c handler flag */
    let first_actor_tx = transmitters.get(&first_actor_name).unwrap().clone();
    let shutdown_actor_tx = transmitters.get(SHUTDOWN_ACTOR).unwrap().clone();
    Ok((shutdown_flag, first_actor_tx, shutdown_actor_tx))
}


fn make_actor_system(
    actor_name: &String,
    transmitters: &HashMap<String, DVSender>,
    receiver: DVReceiver
) -> ActorSystem {
    let mut txs = HashMap::new();
    for (other_actor_name, other_actor_transmitter) in transmitters {
        if *other_actor_name != *actor_name {
            txs.insert(other_actor_name.clone(), other_actor_transmitter.clone());
        }
    }
    ActorSystem {receiver, actors: txs}
}

fn create_visualize_actor(
    actor_system: ActorSystem, map: ReadOnlyWrapper<Map>
) -> Result<tokio::runtime::Runtime, Box<dyn std::error::Error>> {
    // This is cumbersome but this is the cleanest way to do it that I have found
    // Visualization library requires running tokio runtime in current context (creating rt)
    // We can't do this inside the actor constructor because the tokio runtime does not implement Copy, so have to do it here
    // vis_stream needs to be created in the same context as rt (or else runtime error)
    // but visualizer actor needs vis_stream to be able to visualize anything,
    // so have to create vis_stream here and then move into visualizer actor.
    let rt = tokio::runtime::Runtime::new().expect("Failed to initialize visualizer -- tokio runtime");
    let _guard = rt.enter();
    let vis_stream = RecordingStreamBuilder::new("minimal_serve_rs").serve(
        "0.0.0.0",
        Default::default(),
        Default::default(),
        true,
    )?;
    thread::spawn(move || { 
        let mut visualizer = crate::actors::visualizer::DarvisVisualizer::new(actor_system, map, vis_stream);
        visualizer.run();
     } );


    thread::sleep(Duration::from_secs(2)); // Give visualizer time to load

    Ok(rt)
}

fn create_shutdown_actor(actor_system: ActorSystem, shutdown_actor_txs: DVSender) -> Arc<Mutex<bool>> {
    let shutdown_flag = Arc::new(Mutex::new(false));
    let flag_clone = shutdown_flag.clone();

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    struct TrackingPoses {
        trajectory_poses: Vec<Pose>, //mlRelativeFramePoses
        trajectory_times: Vec<DateTime<Utc>>, //mlFrameTimes
        trajectory_keyframes: Vec<Id>, //mlpReferences
    }
    let mut state = TrackingPoses{
        trajectory_poses: Vec::new(),
        trajectory_times: Vec::new(),
        trajectory_keyframes: Vec::new()
    };

    thread::spawn(move || {
        loop {
            let message = actor_system.receive();
            if let Some(_) = <dyn Any>::downcast_ref::<ShutdownMessage>(&message) {
                warn!("Triggered shutdown, saving trajectory info");
                let mut file = File::create(
                    Path::new(RESULTS_FOLDER)
                    .join(GLOBAL_PARAMS.get::<String>(SYSTEM_SETTINGS, "trajectory_file_name"))
                ).unwrap();
                for i in 0..state.trajectory_poses.len() {
                    let string = format!("{:?} {:?}", state.trajectory_times[i], state.trajectory_poses[i]);
                    file.write_all(string.as_bytes()).unwrap();
                }

                if GLOBAL_PARAMS.get::<bool>(SYSTEM_SETTINGS, "should_profile") {
                    flame::dump_html(File::create(
                        Path::new(RESULTS_FOLDER).join("flamegraph.html")
                    ).unwrap()).unwrap();
                }

                for (_, actor_tx) in &actor_system.actors {
                    actor_tx.send(Box::new(ShutdownMessage{})).unwrap();
                }
            } else if let Some(msg) = <dyn Any>::downcast_ref::<TrajectoryMessage>(&message) {
                match msg.pose {
                    Some(pose) => {
                        state.trajectory_poses.push(pose);
                        state.trajectory_times.push(msg.timestamp.unwrap());
                        state.trajectory_keyframes.push(msg.ref_kf_id.unwrap());
                    },
                    None => {
                        // This can happen if tracking is lost. Duplicate last element of each vector
                        if let Some(last) = state.trajectory_poses.last().cloned() { state.trajectory_poses.push(last); }
                        if let Some(last) = state.trajectory_times.last().cloned() { state.trajectory_times.push(last); }
                        if let Some(last) = state.trajectory_keyframes.last().cloned() { state.trajectory_keyframes.push(last); }
                    }
                };
            }
        }
    });

    ctrlc::set_handler(move || {
        warn!("received Ctrl+C!");
        *shutdown_flag.lock().unwrap() = true;
        shutdown_actor_txs.send(Box::new(ShutdownMessage{})).unwrap();
    })
    .expect("Error setting Ctrl-C handler");

    flag_clone
}

pub struct ShutdownActor {

}
impl Actor for ShutdownActor {
    type MSG = TrackingFrontendMsg;
    type HANDLES = ActorSystem;

    fn new(receiver: Box<Receiver<Self::MSG>>, actor_system: ActorSystem) -> Self {

    }
}