use std::{collections::HashMap, sync::mpsc::{Sender, Receiver, RecvError}};

#[derive(Debug)]
pub struct ActorSystem {
    pub actors: HashMap<String, Sender<Box<dyn ActorMessage + Send>>>,
    pub receiver: Receiver<Box<dyn ActorMessage + Send>>,
}
impl ActorSystem {
    pub fn find(&self, name: &str) -> Option<&Sender<Box<dyn ActorMessage + Send>>> {
        self.actors.get(name)
    }
    pub fn receive(&self) -> Result<Box<dyn ActorMessage + Send>, RecvError> {
        self.receiver.recv()
    }
}

impl Default for ActorSystem {
    // Required because we use the default crate for all the actors but we shouldn't actually use this
    // It would be nice to figure out a clean way to require some variables to be explicitly set in
    // the constructor, while others are ok to be defaulted. But this isn't a high priority at all.
    fn default() -> Self { 
        let (_, rx) = std::sync::mpsc::channel::<Box<dyn ActorMessage + Send>>();
        ActorSystem {
            actors: HashMap::new(),
            receiver: rx,
        }
    }
}

pub struct DarvisNone {}
impl DarvisNone {
    pub fn run(name: String)  {
        panic!("Actor {} Not Implemented!!", name);
    }
}

pub trait ActorMessage {}

pub trait Actor {
    fn run(&mut self);
}

pub type DVReceiver = Receiver<Box<dyn ActorMessage + Send>>;
pub type DVSender = Sender<Box<dyn ActorMessage + Send>>;
pub type DVMessageBox = Box<dyn ActorMessage + Send>;