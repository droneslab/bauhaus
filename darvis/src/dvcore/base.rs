use std::collections::HashMap;

#[derive(Debug, Default, Clone)]
// Struct holding configuration parameters for a given actor
pub struct ActorConf{
    pub name: String,
    pub file: String,
    pub actor_message: String,
    pub actor_function: String,
    pub ip_address: String,
    pub port: String,
    pub multithreaded: bool,
    pub threads: i64,
    pub possible_paths: HashMap<String, String>
}

#[derive(Debug, Default, Clone)]
// Struct holding configuration parameters for a given module
pub struct ModuleConf{
    pub name: String
}