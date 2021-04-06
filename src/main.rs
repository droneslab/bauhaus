use std::env;
use glob::glob;
use axiom::prelude::*;

mod base;
mod orb;

fn main() {
    let args: Vec<String> = env::args().collect();
    let img_dir = args[1].to_owned();
    let mut glob_str = img_dir.to_owned();
    glob_str.push_str("/*.png");

    let mut img_paths = Vec::new();

    for entry in glob(&glob_str).expect("Failed to read glob pattern") {
        match entry {
            Ok(path) => match path.to_str() {
                Some(path_str) => img_paths.push(path_str.to_owned()),
                None => println!("Invalid path found!"),
            },
            Err(e) => println!("{:?}", e),
        }
    }

    // First we initialize the actor system using the default config.
    let config = ActorSystemConfig::default();
    let system = ActorSystem::create(config);

    // Spawn the actor and send the message.
    let aid = system.spawn().name("orb_extract").with((), orb::orb_extract).unwrap();
    aid.send_new(orb::OrbMsg::new(aid.clone(), img_paths)).unwrap();

    // The actor will trigger shutdown, we just wait for it.
    system.await_shutdown(None);
}
