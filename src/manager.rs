
#[macro_use]
use std::collections::HashMap;
extern crate const_cstr;
extern crate libc;

extern crate dlopen;

use axiom::prelude::*;

use dlopen::raw::Library;
use libc::{c_char, c_int};
use std::ffi::CStr;
use const_cstr::*;


type DarvisMessage = fn(_a: (),_context: axiom::prelude::Context, message: Message) -> ActorResult<()>;


use lazy_static::lazy_static;


lazy_static! {
    static ref ALL_HANDLES: HashMap<&'static str, &'static str> = {
        let mut map = HashMap::new();
        map.insert("orb::orb_extract", "orb_extract");
        map.insert("align::align", "align");
        map.insert("vis::Vis::visualize", "visualize_static");
        map
    };
}

use std::ffi::c_void;

pub struct Myobject{
    pub object :  *mut c_void
}
unsafe impl Send for Myobject {}
unsafe impl Sync for Myobject {}


pub struct Manager{
    pub handles :  HashMap<String, DarvisMessage>,
    pub curr_handle : String,
    pub object :  Myobject
}


impl Manager
{
    pub async fn handle(mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<Self> {

        self.handles[&self.curr_handle]((), _context, message).unwrap();
        Ok(Status::done(self))    
    }

}

use std::ffi::CString;

pub fn get_handler(function_name: &str) -> HashMap<String, DarvisMessage>
{
    let lib_path = "libdarvis.so";
    let lib = Library::open(lib_path).expect("Could not open library");
    
    let strfun : &str = ALL_HANDLES[function_name];
    let c_str = CString::new(strfun).unwrap();
    let c_world = c_str.as_c_str();    

    //get several symbols and play around 
    let message_handler: DarvisMessage =
        unsafe { lib.symbol_cstr(c_world) }.unwrap();

    let mut handles : HashMap<String, DarvisMessage> = HashMap::new();
    handles.insert(function_name.to_string(), message_handler);
    handles

}