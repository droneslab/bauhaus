pub mod align;
pub mod base;
pub mod utils;
pub mod vis;
pub mod config;
pub mod orb;

pub mod darvismsg;

use axiom::prelude::*;

use darvismsg::DarvisMessage;
use std::collections::HashMap;


pub static CORE_VERSION: &str = env!("CARGO_PKG_VERSION");
pub static RUSTC_VERSION: &str = env!("RUSTC_VERSION");

//#[async_trait]
pub trait Function: FunctionClone {
    
    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()>;  

    fn send_new(&mut self, message: DarvisMessage, aids:  &HashMap<String, axiom::actors::Aid>) -> ActorResult<()>;
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



#[derive(Debug, Clone, PartialEq)]
pub enum InvocationError {
    InvalidArgumentCount { expected: usize, found: usize },
    Other { msg: String },
}

impl<S: ToString> From<S> for InvocationError {
    fn from(other: S) -> InvocationError {
        InvocationError::Other {
            msg: other.to_string(),
        }
    }
}

#[derive(Copy, Clone)]
pub struct PluginDeclaration {
    pub rustc_version: &'static str,
    pub core_version: &'static str,
    pub register: unsafe extern "C" fn(&mut dyn PluginRegistrar),
}

pub trait PluginRegistrar {
    fn register_function(&mut self, name: &str, function: Box<dyn Function>);
    
}

#[macro_export]
macro_rules! export_plugin {
    ($register:expr) => {
        #[doc(hidden)]
        #[no_mangle]
        pub static plugin_declaration: $crate::PluginDeclaration =
            $crate::PluginDeclaration {
                rustc_version: $crate::RUSTC_VERSION,
                core_version: $crate::CORE_VERSION,
                register: $register,
            };
    };
}
