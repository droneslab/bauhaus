use libloading::Library;
use plugins_core::{Function, InvocationError, PluginDeclaration};
use std::{
    alloc::System, collections::HashMap, env, ffi::OsStr, io, path::PathBuf,
    rc::Rc,
};
use axiom::prelude::*;

//#[global_allocator]
//static ALLOCATOR: System = System;

pub struct Manager{
    pub curr_handle : String,
    //pub functions :  ExternalFunctions,
    pub object: FunctionProxy,
}

pub fn load(lib_path:String) -> ExternalFunctions
{
    let mut functions = ExternalFunctions::new();
    unsafe {
        println!("test random load libplugins.so");
        functions
            .load(lib_path)
            .expect("Function loading failed");
    }
    functions
}

impl Manager
{
    pub async fn handle(mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<Self> {
        
        //let fnstr :&str = &self.curr_handle.clone();
        self.object.handle(_context, message).unwrap();
        //self.functions.handle(fnstr).clone().handle(_context, message);
        //self.functions.clone().call(fnstr, _context, message);
        Ok(Status::done(self))    
    }
}


/// A map of all externally provided functions.
#[derive(Default, Clone)]
pub struct ExternalFunctions {
    functions: HashMap<String, FunctionProxy>,
    libraries: Vec<Rc<Library>>,
}

unsafe impl Sync for ExternalFunctions{}
unsafe impl Send for ExternalFunctions{}


impl ExternalFunctions {
    pub fn new() -> ExternalFunctions { ExternalFunctions::default() }


    pub fn handle(&self,
        function: &str,
    ) -> FunctionProxy
    {
        self.functions
        .get(function)
        .ok_or_else(|| format!("\"{}\" not found", function)).unwrap().clone()
       // .handle
    }
    // pub fn call(mut self,
    //     function: &str, _context: axiom::prelude::Context, message: Message
    // ) ->  ActorResult<Self> 
    // {
    //     self.functions
    //     .get(function)
    //     .ok_or_else(|| format!("\"{}\" not found", function)).unwrap()
    //     .handle(_context, message);
    //     //println!("Function Proxy handle call");
    //    Ok(Status::done(self))
    // }
    /// Load a plugin library and add all contained functions to the internal
    /// function table.
    ///
    /// # Safety
    ///
    /// A plugin library **must** be implemented using the
    /// [`plugins_core::plugin_declaration!()`] macro. Trying manually implement
    /// a plugin without going through that macro will result in undefined
    /// behaviour.
    pub unsafe fn load<P: AsRef<OsStr>>(
        &mut self,
        library_path: P,
    ) -> io::Result<()> {
        // load the library into memory
        let library = Rc::new(Library::new(library_path)?);

        // get a pointer to the plugin_declaration symbol.
        let decl = library
            .get::<*mut PluginDeclaration>(b"plugin_declaration\0")?
            .read();

        // version checks to prevent accidental ABI incompatibilities
        if decl.rustc_version != plugins_core::RUSTC_VERSION
            || decl.core_version != plugins_core::CORE_VERSION
        {
            return Err(io::Error::new(
                io::ErrorKind::Other,
                "Version mismatch",
            ));
        }

        let mut registrar = PluginRegistrar::new(Rc::clone(&library));

        (decl.register)(&mut registrar);

        // add all loaded plugins to the functions map
        self.functions.extend(registrar.functions);
        // and make sure ExternalFunctions keeps a reference to the library
        self.libraries.push(library);

        Ok(())
    }
}

struct PluginRegistrar {
    functions: HashMap<String, FunctionProxy>,
    lib: Rc<Library>,
}

impl PluginRegistrar {
    fn new(lib: Rc<Library>) -> PluginRegistrar {
        PluginRegistrar {
            lib,
            functions: HashMap::default(),
        }
    }
}

impl plugins_core::PluginRegistrar for PluginRegistrar {
    fn register_function(&mut self, name: &str, function: Box<dyn Function>) {
        let proxy = FunctionProxy {
            function,
            _lib: Rc::clone(&self.lib),
        };
        self.functions.insert(name.to_string(), proxy);
    }
}

/// A proxy object which wraps a [`Function`] and makes sure it can't outlive
/// the library it came from.
#[derive(Clone)]
pub struct FunctionProxy {
    pub function: Box<dyn Function>,
    _lib: Rc<Library>,
}


// to avoid error in async_trait handle implementation
unsafe impl Send for FunctionProxy {}
unsafe impl Sync for FunctionProxy {}

//#[async_trait]

impl Function for FunctionProxy {

    //async 
    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()> 
    {
        //println!("Function Proxy handle");
        self.function.handle(_context, message);
        Ok(Status::done(()))
    }

}
