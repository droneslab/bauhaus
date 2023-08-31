use std::{collections::HashMap, sync::mpsc::{Sender, Receiver, RecvError, self}, any::Any};

// Struct that every actor should have, references all the transmitters to other actors
// and the receiver for the current actor that it should read messages with.
#[derive(Clone)]
pub struct ActorSystem {
    pub actors: HashMap<String, Box::<dyn ActorChannel>>,
    // pub receiver: Receiver<Box<dyn ActorMessage + Send>>,
}
impl ActorSystem {
    pub fn find(&self, name: &str) -> Option<&Box<dyn ActorChannel>> {
        self.actors.get(name)
    }
    // pub fn receive(&self) -> Result<Box<dyn ActorMessage + Send>, RecvError> {
    //     self.receiver.recv()
    // }
}

impl Default for ActorSystem {
    // Required because we use the default crate for all the actors but we shouldn't actually use this
    // It would be nice to figure out a clean way to require some variables to be explicitly set in
    // the constructor, while others are ok to be defaulted. But this isn't a high priority at all.
    fn default() -> Self { 
        let (_, rx) = std::sync::mpsc::channel::<Box<dyn ActorMessage + Send>>();
        ActorSystem {
            actors: HashMap::new(),
            // receiver: rx,
        }
    }
}

pub trait ActorMessage {
    // fn as_any(&self) -> &dyn Any;
}

pub trait ActorChannel {
    fn clone_me(&self) -> Box<dyn ActorChannel>;
}

impl Clone for Box<dyn ActorChannel> {
    fn clone(&self) -> Box<dyn ActorChannel> {
        self.clone_me()
    }
}

impl<M: ActorMessage> ActorChannel for Sender<M> {
    fn clone_me(&self) -> Box<dyn ActorChannel> {
        let c = self.clone();
        Box::new(c)
    }
}

pub type DVReceiver = Receiver<Box<dyn ActorMessage + Send>>;
pub type DVSender = Sender<Box<dyn ActorMessage + Send>>;
// pub type DVMessageBox = Box<dyn ActorMessage + Send>;

pub trait Actor {
    type MSG: ActorMessage;
    type HANDLES;

    fn new(pipe: Box<Receiver<Self::MSG>>, handles: Self::HANDLES) -> Self;
}

