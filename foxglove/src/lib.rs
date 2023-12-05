// Include the `items` module, which is generated from items.proto.
// It is important to maintain the same structure as in the proto.
pub mod foxglove {
    pub mod items {
        include!(concat!(env!("OUT_DIR"), "/foxglove.rs"));
    }
}

use foxglove::items::{Vector3, Pose, Quaternion};


pub fn get_file_descriptor_set_bytes() -> &'static [u8] {
    include_bytes!(concat!(env!("OUT_DIR"), "/file_descriptor_set.bin"))
}

pub fn make_pose(x: f64, y: f64, z: f64) -> Option<Pose> {
    let position = Vector3 { x, y, z };

    let orientation = Quaternion {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0
    };

    Some(Pose {
        position: Some(position),
        orientation: Some(orientation),
    })
}
