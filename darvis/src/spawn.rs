use std::{sync::{Arc, Mutex}, thread, collections::HashMap};
use crossbeam_channel::unbounded;

use core::{
    config::ActorConf,
    actor::{Sender, Receiver, ActorChannels},
    maplock::ReadWriteMap
};
use log::{info, warn};

use crate::{
    registered_actors::{SHUTDOWN_ACTOR, self},
    map::map::Map,
    actors::messages::ShutdownMsg, MapLock,
};

// Initialize actor system using config file.
// Returns mutex to shutdown flag and transmitters for first actor and shutdown actor.
// You probably don't want to change this code.
pub fn launch_actor_system(config: Vec::<ActorConf>, first_actor_name: String) 
    -> Result<(Arc<std::sync::Mutex<bool>>, Sender, Sender), Box<dyn std::error::Error>> 
{
    // * SET UP CHANNELS *//
    // Create transmitters and receivers for user-defined actors
    let mut transmitters = HashMap::new();
    let mut receivers = Vec::new();
    for actor_conf in &config {
        let (tx, rx) = unbounded(); // Note: bounded channels block on send if channel is full, so use unbounded and handle drops in receive()
        transmitters.insert(actor_conf.name.clone(), tx);
        receivers.push((actor_conf.name.clone(), actor_conf.tag.clone(), actor_conf.receiver_bound.clone(), rx));
    }

    // Create transmitters/receivers for darvis system actors
    // Don't add receivers to the receivers vec because they need to be spawned separately.
    // User-defined actors still need the transmitters to these actors.
    let (shutdown_tx, shutdown_rx) = unbounded();
    transmitters.insert(SHUTDOWN_ACTOR.to_string(), shutdown_tx);

    // * SPAWN USER-DEFINED ACTORS *//
    // Create map
    let writeable_map = ReadWriteMap::<Map>::new(Map::new()); // Arc::new(parking_lot::Mutex::new(Map::new()))
    // Spawn actors
    for (actor_name, actor_tag, receiver_bound, receiver) in receivers {
        spawn_actor(actor_tag, actor_name, &transmitters, receiver, receiver_bound, Some(&writeable_map));
    }

    // * SPAWN DARVIS SYSTEM ACTORS *//
    // Ctrl+c shutdown actor
    let shutdown_flag = spawn_shutdown_actor(&transmitters, shutdown_rx);

    //* Return transmitters for the shutdown actor and first actor in the pipeline, and the ctrl+c handler flag */
    let first_actor_tx = transmitters.get(&first_actor_name).unwrap().clone();
    let shutdown_actor_tx = transmitters.get(SHUTDOWN_ACTOR).unwrap().clone();
    Ok((shutdown_flag, first_actor_tx, shutdown_actor_tx))
}


fn spawn_actor(
    actor_tag: String, actor_name: String, transmitters: &HashMap<String, Sender>, receiver: Receiver,
    receiver_bound: Option<usize>, writeable_map: Option<&MapLock>
) {
    info!("Spawning actor '{}' with name {}", &actor_tag, &actor_name);

    // Note: not using this right now, but if you wanted to pass a read-only map,
    // call map.create_read_only(). Otherwise all actors can access the write lock.
    let map_clone = match writeable_map {
        Some(map) => Some(map.clone()),
        None => None
    };

    let mut txs = HashMap::new();
    for (other_actor_name, other_actor_transmitter) in transmitters {
        if *other_actor_name != *actor_name {
            txs.insert(other_actor_name.clone(), other_actor_transmitter.clone());
        }
    }
    let actor_channels = ActorChannels {receiver, receiver_bound, actors: txs, my_name: actor_name.clone()};

    thread::spawn(move || { 
        registered_actors::spawn(actor_tag, actor_channels, map_clone);
    } );

}


fn spawn_shutdown_actor(transmitters: &HashMap<String, Sender>, receiver: Receiver) -> Arc<Mutex<bool>> {
    let shutdown_flag = Arc::new(Mutex::new(false));
    let flag_clone = shutdown_flag.clone();

    let mut txs = HashMap::new();
    for (other_actor_name, other_actor_transmitter) in transmitters {
        if *other_actor_name != *SHUTDOWN_ACTOR.to_string() {
            txs.insert(other_actor_name.clone(), other_actor_transmitter.clone());
        }
    }
    let actor_channels = ActorChannels {receiver, receiver_bound: None, actors: txs, my_name: SHUTDOWN_ACTOR.to_string()};

    thread::spawn(move || { 
        registered_actors::spawn(SHUTDOWN_ACTOR.to_string(), actor_channels, None);
    } );

    let shutdown_transmitter = transmitters.get(SHUTDOWN_ACTOR).unwrap().clone();
    ctrlc::set_handler(move || {
        warn!("received Ctrl+C!");
        *shutdown_flag.lock().unwrap() = true;
        shutdown_transmitter.send(Box::new(ShutdownMsg{})).unwrap();
    })
    .expect("Error setting Ctrl-C handler");

    flag_clone
}
