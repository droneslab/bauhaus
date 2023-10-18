use std::{sync::{Arc, mpsc, Mutex}, thread, collections::HashMap, time::Duration};

use dvcore::{
    config::{ActorConf, GLOBAL_PARAMS, SYSTEM_SETTINGS},
    base::{DVSender, DVReceiver, ActorChannels, DVMessageBox, Actor},
    lockwrap::{ReadWriteWrapper, ReadOnlyWrapper}
};
use log::{info, warn};

use crate::{
    registered_actors::{VISUALIZER, SHUTDOWN_ACTOR, MAP_ACTOR, get_actor},
    dvmap::map::Map,
    actors::messages::ShutdownMessage,
};


// Initialize actor system using config file.
// Returns mutex to shutdown flag and transmitters for first actor and shutdown actor.
// You probably don't want to change this code.
pub fn initialize_actors(config: Vec::<ActorConf>, first_actor_name: String) 
    -> Result<(Arc<std::sync::Mutex<bool>>, DVSender, DVSender), Box<dyn std::error::Error>> 
{
    // * SET UP CHANNELS *//
    // Create transmitters and receivers for user-defined actors
    let mut transmitters = HashMap::new();
    let mut receivers = Vec::new();
    for actor_conf in &config {
        let (tx, rx) = mpsc::channel::<DVMessageBox>();
        transmitters.insert(actor_conf.name.clone(), tx);
        receivers.push((actor_conf.name.clone(), rx));
    }

    // Create transmitters/receivers for darvis system actors
    // Don't add receivers to the receivers vec because they need to be spawned separately.
    // User-defined actors still need the transmitters to these actors.
    let (map_tx, map_rx) = mpsc::channel::<DVMessageBox>();
    transmitters.insert(MAP_ACTOR.to_string(), map_tx);
    let (shutdown_tx, shutdown_rx) = mpsc::channel::<DVMessageBox>();
    transmitters.insert(SHUTDOWN_ACTOR.to_string(), shutdown_tx);
    let vis_rx = match GLOBAL_PARAMS.get::<bool>(SYSTEM_SETTINGS, "show_visualizer") {
        true => {
            let (vis_tx, vis_rx) = mpsc::channel::<DVMessageBox>();
            transmitters.insert(VISUALIZER.to_string(), vis_tx);
            Some(vis_rx)
        },
        false => None
    };

    // * SPAWN USER-DEFINED ACTORS *//
    // Create map
    let writeable_map = ReadWriteWrapper::new(Map::new());
    // Spawn actors
    for (actor_name, receiver) in receivers {
        spawn_actor(actor_name, &transmitters, receiver, Some(&writeable_map));
    }

    // * SPAWN DARVIS SYSTEM ACTORS *//
    // Visualizer
    if GLOBAL_PARAMS.get::<bool>(SYSTEM_SETTINGS, "show_visualizer") {
        spawn_actor(VISUALIZER.to_string(), &transmitters, vis_rx.unwrap(), Some(&writeable_map));
    }

    // Map actor
    // After this point, you cannot get a read-only clone of the map!
    // So make sure to spawn all the other actors that need the map before this.
    spawn_map_actor(&transmitters, map_rx, writeable_map);

    // Ctrl+c shutdown actor
    let shutdown_flag = spawn_shutdown_actor(&transmitters, shutdown_rx);

    //* Return transmitters for the shutdown actor and first actor in the pipeline, and the ctrl+c handler flag */
    let first_actor_tx = transmitters.get(&first_actor_name).unwrap().clone();
    let shutdown_actor_tx = transmitters.get(SHUTDOWN_ACTOR).unwrap().clone();
    Ok((shutdown_flag, first_actor_tx, shutdown_actor_tx))
}


fn spawn_actor(
    actor_name: String, transmitters: &HashMap<String, DVSender>, receiver: DVReceiver,
    writeable_map: Option<&ReadWriteWrapper<Map>>
) {
    info!("Spawning actor;{}", &actor_name);

    // Note: read_only() is important here, otherwise all actors can
    // access the write lock of the map
    let map_clone = match writeable_map {
        Some(map) => Some(map.read_only()),
        None => None
    };

    let mut txs = HashMap::new();
    for (other_actor_name, other_actor_transmitter) in transmitters {
        if *other_actor_name != *actor_name {
            txs.insert(other_actor_name.clone(), other_actor_transmitter.clone());
        }
    }
    let actor_channels = ActorChannels {receiver, actors: txs};

    thread::spawn(move || { 
        let mut actor = get_actor(actor_name, actor_channels, map_clone);
        actor.run();
    } );
}

fn spawn_map_actor(transmitters: &HashMap<String, DVSender>, receiver: DVReceiver, map: ReadWriteWrapper<Map> ) {
    info!("Spawning actor;{}", MAP_ACTOR);
    let mut txs = HashMap::new();
    for (other_actor_name, other_actor_transmitter) in transmitters {
        if *other_actor_name != *MAP_ACTOR.to_string() {
            txs.insert(other_actor_name.clone(), other_actor_transmitter.clone());
        }
    }
    let actor_channels = ActorChannels {receiver, actors: txs};

    thread::spawn(move || { 
        let mut map_actor = Box::new(crate::actors::map_actor::MapActor::new(actor_channels, map));
        map_actor.run();
    } );
}

// fn spawn_visualize_actor(
//     transmitters: &HashMap<String, DVSender>, receiver: DVReceiver, map: &ReadWriteWrapper<Map>
// ) -> Result<tokio::runtime::Runtime, Box<dyn std::error::Error>> {
//     // This is cumbersome but this is the cleanest way to do it that I have found
//     // Visualization library requires running tokio runtime in current context (creating rt)
//     // We can't do this inside the actor constructor because the tokio runtime does not implement Copy, so have to do it here
//     // vis_stream needs to be created in the same context as rt (or else runtime error)
//     // but visualizer actor needs vis_stream to be able to visualize anything,
//     // so have to create vis_stream here and then move into visualizer actor.
//     let rt = tokio::runtime::Runtime::new().expect("Failed to initialize visualizer -- tokio runtime");
//     let _guard = rt.enter();
//     let vis_stream = RecordingStreamBuilder::new("minimal_serve_rs").serve(
//         "0.0.0.0",
//         Default::default(),
//         Default::default(),
//         true,
//     )?;

//     spawn_actor(VISUALIZER.to_string(), transmitters, receiver, Some(map), Some(vis_stream.clone()));

//     thread::sleep(Duration::from_secs(2)); // Give visualizer time to load

//     Ok(rt)
// }

fn spawn_shutdown_actor(transmitters: &HashMap<String, DVSender>, receiver: DVReceiver) -> Arc<Mutex<bool>> {
    let shutdown_flag = Arc::new(Mutex::new(false));
    let flag_clone = shutdown_flag.clone();

    spawn_actor(SHUTDOWN_ACTOR.to_string(), transmitters, receiver, None);

    let shutdown_transmitter = transmitters.get(SHUTDOWN_ACTOR).unwrap().clone();
    ctrlc::set_handler(move || {
        warn!("received Ctrl+C!");
        *shutdown_flag.lock().unwrap() = true;
        shutdown_transmitter.send(Box::new(ShutdownMessage{})).unwrap();
    })
    .expect("Error setting Ctrl-C handler");

    flag_clone
}
