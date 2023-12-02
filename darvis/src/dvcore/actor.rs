/// *** Actor channel and actor message implementation. *** //

use std::{collections::HashMap};
use crossbeam_channel::{RecvError, unbounded};

use downcast_rs::{impl_downcast, Downcast};
use log::{info, error};

pub type MessageBox = Box<dyn ActorMessage>;
pub type Receiver = crossbeam_channel::Receiver<MessageBox>;
pub type Sender = crossbeam_channel::Sender<MessageBox>;

#[derive(Debug)]
pub struct ActorChannels {
    // Note: Run-time errors
    // each actor has actorchannels struct to communicate with other actors
    // can't make DVSender an enum depending on each actor because a collection can't hold different types
    pub actors: HashMap<String, Sender>,
    pub receiver: Receiver,
    pub receiver_bound: Option<usize>,
    pub my_name: String,
}
impl ActorChannels {
    pub fn find(&self, name: &str) -> &Sender {
        self.actors.get(name).expect(format!("Could not find actor {}", name).as_str())
    }
    pub fn send(&self, actor_name: &str, message: MessageBox) {
        self.find(actor_name).send(message).unwrap_or_else(|_| panic!("Could not send message to actor {}", actor_name));
    }

    pub fn receive(&self) -> Result<MessageBox, RecvError> {
        match self.receiver_bound {
            Some(bound) => {
                let mut result = self.receiver.recv()?;
                let mut dropped = 0;
                while self.receiver.len() > bound {
                    dropped += 1;
                    result = self.receiver.recv()?;
                }
                if dropped > 0 {
                    info!("Actor {} dropped {} messages.", self.my_name, dropped);
                }
                Ok(result)
            },
            None => self.receiver.recv(),
        }
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

impl Default for ActorChannels {
    // Required because we use the default crate for all the actors but we shouldn't actually use this
    // It would be nice to figure out a clean way to require some variables to be explicitly set in
    // the constructor, while others are ok to be defaulted. But this isn't a high priority at all.
    fn default() -> Self { 
        let (_, rx) = unbounded();
        ActorChannels {
            actors: HashMap::new(),
            receiver: rx,
            receiver_bound: None,
            my_name: "Default".to_string()
        }
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

    fn new_actorstate(_actor_system: ActorChannels, _map: Self::MapRef) -> Self {
        NullActor{}
    }
    fn spawn(_actor_system: ActorChannels, _map: Self::MapRef)  {
        error!("Actor Not Implemented!!");
    }
}


pub trait ActorMessage: Downcast + Send {}
impl_downcast!(ActorMessage);

pub trait Base: Downcast {}
impl_downcast!(Base);


pub trait Actor {
    type MapRef;

    fn new_actorstate(actor_channels: ActorChannels, map: Self::MapRef) -> Self;
    fn spawn(actor_channels: ActorChannels, map: Self::MapRef);
}
