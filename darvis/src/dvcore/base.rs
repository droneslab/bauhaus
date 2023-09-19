use std::{collections::HashMap, sync::mpsc::{Sender, Receiver, RecvError}};
use downcast_rs::{DowncastSync, impl_downcast, Downcast};

pub type DVMessageBox = Box<dyn ActorMessage>;
pub type DVReceiver = Receiver<DVMessageBox>;
pub type DVSender = Sender<DVMessageBox>;

#[derive(Debug)]
pub struct ActorChannels {
    pub actors: HashMap<String, DVSender>,
    pub receiver: DVReceiver,
}
impl ActorChannels {
    pub fn find(&self, name: &str) -> Option<&DVSender> {
        self.actors.get(name)
    }
    pub fn receive(&self) -> Result<DVMessageBox, RecvError> {
        self.receiver.recv()
    }
    pub fn copy_transmitters(&self, actor_name: &String) -> HashMap<String, DVSender> {
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
        let (_, rx) = std::sync::mpsc::channel::<DVMessageBox>();
        ActorChannels {
            actors: HashMap::new(),
            receiver: rx,
        }
    }
}

pub struct DarvisNone {
    name: String,
}
impl DarvisNone {
    pub fn new(name: String) -> Self {
        DarvisNone{name}
    }
}
impl Actor for DarvisNone{
    // type INPUTS = String;

    fn run(&mut self)  {
        panic!("Actor {} Not Implemented!!", self.name);
    }
}

// pub trait ActorMessage {}
pub trait ActorMessage: Downcast + Send {}
impl_downcast!(ActorMessage);
// downcast_rs::impl_downcast!(Base<T> assoc H where T: Clone, H: Copy);

pub trait Base: Downcast {}
impl_downcast!(Base);


pub trait Actor {
    // type INPUTS;

    // fn new(inputs: Self::INPUTS) -> Self;
    fn run(&mut self);
}
