use axiom::prelude::*;
use log::error;

/// Prototype function to be implemented for each new actor added.
/// Each actor implementation should have this trait implemented in order to make it callable through the framework
pub trait Function: FunctionClone {
    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()>;  
}

pub trait FunctionClone {
    fn clone_box(&self) -> Box<dyn Function>;
}

impl<T> FunctionClone for T
where
    T: 'static + Function + Clone,
{
    fn clone_box(&self) -> Box<dyn Function> {
        Box::new(self.clone())
    }
}

impl Clone for Box<dyn Function> {
    fn clone(&self) -> Box<dyn Function> {
        self.clone_box()
    }
}

#[derive(Clone)]
/// All the function traits will be wrapped under  FunctionProxy struct in order to attain polymorphism. 
/// To select a function at runtime using framework, the actual function is boxed into this struct.
pub struct FunctionProxy {
    pub function: Box<dyn Function>,
}

// to avoid error in async_trait handle implementation
unsafe impl Send for FunctionProxy {}
unsafe impl Sync for FunctionProxy {}

impl Function for FunctionProxy {
    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()>  {
        self.function.handle(_context, message).unwrap();
        Ok(Status::done(()))
    }
}

#[derive(Debug, Clone)]
/// A Dummy Implementation that will be called for unimplemented actor function 
/// i.e. if the framework didn't find any registered actor, 
/// will by default call to this trait implementation.
pub struct DarvisNone;

impl Function for DarvisNone {
    fn handle(&mut self, _context: axiom::prelude::Context, _message: Message) -> ActorResult<()>
    {
        error!("Darvis None : Actor Not Implemented!!");
        Ok(Status::done(()))
    }
}