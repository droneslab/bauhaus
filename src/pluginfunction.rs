use axiom::prelude::*;


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
pub struct FunctionProxy {
    pub function: Box<dyn Function>,
}


// to avoid error in async_trait handle implementation
unsafe impl Send for FunctionProxy {}
unsafe impl Sync for FunctionProxy {}


impl Function for FunctionProxy {

    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()> 
    {
        self.function.handle(_context, message).unwrap();
        Ok(Status::done(()))
    }

}