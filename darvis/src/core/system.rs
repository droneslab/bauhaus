/// *** Defines actor channels and messages, module object which holds module info, and "system" object which stores references to actors/modules. *** //

use std::collections::HashMap;
use crossbeam_channel::RecvError;

use downcast_rs::{impl_downcast, Downcast};
use log::error;

pub type Timestamp = f64;

pub type MessageBox = Box<dyn ActorMessage>;
pub type Receiver = crossbeam_channel::Receiver<MessageBox>;
pub type Sender = crossbeam_channel::Sender<MessageBox>;

pub struct System {
    // Note: Run-time errors
    // each actor has actorchannels struct to communicate with other actors
    // can't make DVSender an enum depending on each actor because a collection can't hold different types
    pub actors: HashMap<String, Sender>,
    pub receiver: Receiver,
    pub my_name: String,
    pub max_queue_size: usize,
}

impl System {
    pub fn find_actor(&self, name: &str) -> &Sender {
        self.actors.get(name).expect(format!("Could not find actor {}", name).as_str())
    }

    pub fn try_send(&self, actor_name: &str, message: MessageBox) -> Option<()> {
        self.actors.get(actor_name)?.send(message).ok()
    }

    pub fn send(&self, actor_name: &str, message: MessageBox) {
        self.find_actor(actor_name).send(message).unwrap_or_else(|_| panic!("Could not send message to actor {}", actor_name));
    }

    pub fn receive(&self) -> Result<MessageBox, RecvError> {
        // Note: not doing this right now. Instead, dropping messages in the spawn thread
        // of the actors so that we can filter drops by message type.
        // match self.receiver_bound {
        //     Some(bound) => {
        //         let mut result = self.receiver.recv()?;
        //         let mut dropped = 0;
        //         while self.receiver.len() > bound {
        //             dropped += 1;
        //             result = self.receiver.recv()?;
        //         }
        //         if dropped > 0 {
        //             info!("Actor {} dropped {} messages.", self.my_name, dropped);
        //         }
        //         Ok(result)
        //     },
        //    None => 
        self.receiver.recv()
    }

    pub fn queue_len(&self) -> usize {
        self.receiver.len()
    }

    pub fn queue_full(&self) -> bool {
        self.receiver.len() > self.max_queue_size 
    }

    pub fn copy_transmitters(&self, actor_name: &String) -> HashMap<String, Sender> {
        let mut txs = HashMap::new();
        for (other_actor_name, other_actor_transmitter) in &self.actors {
            if *other_actor_name != *actor_name {
                txs.insert(other_actor_name.clone(), other_actor_transmitter.clone());
            }
        }
        txs
    }
}

pub struct NullActor { }
impl NullActor {
    pub fn new() -> Self {
        NullActor{}
    }
}
impl Actor for NullActor {
    type MapRef = ();

    fn spawn(_system: System, _map: Self::MapRef)  {
        error!("Actor Not Implemented!!");
    }
}


pub trait ActorMessage: Downcast + Send {}
impl_downcast!(ActorMessage);

pub trait Base: Downcast {}
impl_downcast!(Base);


pub trait Actor {
    type MapRef;

    fn spawn(system: System, map: Self::MapRef);
}

pub trait Module { }

// unsafe impl Send for Module {}
// unsafe impl Sync for Module {}
