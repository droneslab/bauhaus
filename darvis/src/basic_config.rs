// use dvcore::{base::{Actor, ActorChannels, DarvisNone}, lockwrap::ReadOnlyWrapper};

// use crate::{actors::{tracking_frontend::DarvisTrackingFront, tracking_backend::DarvisTrackingBack, local_mapping::DarvisLocalMapping, loop_closing::DarvisLoopClosing}, registered_modules::{TRACKING_FRONTEND, TRACKING_BACKEND, LOCAL_MAPPING, LOOP_CLOSING}, dvmap::map::Map};


// pub struct DarvisConfig {
//     actors: Vec::<Box<dyn Actor>>,
//     actor_configs: Vec::<ActorConfig>,
// }
// impl DarvisConfig {
//     pub fn spawn() {

//     }
// }

// impl DarvisConfig {
//     pub fn create_config() -> Vec::<ActorConfig> {
//         let tracking_frontend = ActorConfig {
//             name: TRACKING_FRONTEND.to_string(),
//             from: vec![],
//             to: vec![TRACKING_BACKEND.to_string()],
//         };
//         let tracking_backend = ActorConfig {
//             name: TRACKING_BACKEND.to_string(),
//             from: vec![TRACKING_FRONTEND.to_string()],
//             to: vec![LOCAL_MAPPING.to_string(), LOOP_CLOSING.to_string()],
//         };

//         return vec![tracking_frontend, tracking_backend];


//         // ActorSystem{actors}
//     }
// }

// pub struct ActorConfig {
//     pub name: String,
//     // actor: Box<dyn Actor>,
//     from: Vec<String>,
//     to: Vec<String>,
// }

// impl ActorConfig {
//     // pub fn new_simple(name: String, actor_channels: ActorChannels, map: Option<ReadOnlyWrapper<Map>>) -> Self {
//     //     let new_tx = actor_channels.copy_transmitters(&name);

//     //     if name == TRACKING_FRONTEND.to_string() {
//     //         return Self { name, actor: Box::new(DarvisTrackingFront::new(actor_channels)), from: vec![], to: vec![] };

//     //     } else {
//     //         return Self { name, actor: Box::new(DarvisNone::new(name)), from: vec![], to: vec![] };
//     //     }
//     // }

//     // pub fn update_connections(&mut self, from: Vec<Box<dyn Actor>>, to: Vec<Box<dyn Actor>>) {
//     //     self.from = from;
//     //     self.to = to;
//     // }

// }
