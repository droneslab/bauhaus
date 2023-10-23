use std::{collections::BTreeMap, borrow::Cow, sync::Arc, fs, io::BufWriter};
use mcap::{Channel, records::MessageHeader, Writer, Schema};
use prost::Message;
use prost_types::Timestamp;
use foxglove::{foxglove::items::{SceneEntity, SceneUpdate, SpherePrimitive, Vector3, Quaternion, Color, FrameTransform, LinePrimitive, line_primitive, Point3}, get_file_descriptor_set_bytes, make_pose};

fn main() {
    let mut writer = Writer::new(
        BufWriter::new(fs::File::create("results/out.mcap").unwrap())
    ).unwrap();

    // Spheres
    let schema = Schema {
        name: String::from("foxglove.SceneUpdate"),
        encoding: String::from("protobuf"),
        data: Cow::Borrowed(get_file_descriptor_set_bytes())
    };
    let channel = Channel {
        topic: String::from("/scene"),
        schema: Some(Arc::new(schema)),
        message_encoding: String::from("protobuf"),
        metadata: BTreeMap::default()
    };
    let sphere_channel = writer.add_channel(&channel).unwrap();


    // Frame transforms
    let schema2 = Schema {
        name: String::from("foxglove.FrameTransform"),
        encoding: String::from("protobuf"),
        data: Cow::Borrowed(get_file_descriptor_set_bytes())
    };
    let channel2 = Channel {
        topic: String::from("/frame_transforms"),
        schema: Some(Arc::new(schema2)),
        message_encoding: String::from("protobuf"),
        metadata: BTreeMap::default()
    };
    let transform_channel = writer.add_channel(&channel2).unwrap();


    
    // Write basic frame transform
    let transform = FrameTransform {
        timestamp: Some(Timestamp { seconds: 0,  nanos: 0 }),
        parent_frame_id: 0.to_string(),
        child_frame_id: 1.to_string(),
        translation: Some(Vector3{ x: 0.0, y: 0.0, z: 0.0 }),
        rotation: Some(Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }),
    };
    writer.write_to_known_channel(
        &MessageHeader {
            channel_id: transform_channel,
            sequence: 0,
            log_time: 0,
            publish_time: 0
        },
        &transform.encode_to_vec()
    ).unwrap();



    // Default visual stuff
    let sphere_color = Color { r: 1.0, g: 0.0, b: 0.0, a: 1.0 };
    let line_color = Color { r: 0.0, g: 1.0, b: 0.0, a: 1.0 };
    let sphere_size = Vector3 { x: 0.3, y: 0.3, z: 0.3 };



    // Write trajectory
    for i in 1..100 {
        println!("publish time {}", i);
        let timestamp = Timestamp { 
            seconds: i, 
            nanos: 0
        };

        let sphere = SpherePrimitive {
            pose: make_pose(i as f64, 0.0, 0.0),
            size: Some(sphere_size.clone()),
            color: Some(sphere_color.clone()),
        };

        let line = LinePrimitive {
            r#type: line_primitive::Type::LineStrip as i32,
            pose: make_pose(0.0, 0.0, 0.0),
            thickness: 1.0,
            scale_invariant: true,
            points: vec![Point3{ x: (i - 1) as f64, y: 0.0, z: 0.0 }, Point3{ x: i as f64, y: 0.0, z: 0.0 }],
            color: Some(line_color.clone()),
            colors: vec![],
            indices: vec![],
        };

        let entity1 = SceneEntity {
            timestamp: Some(timestamp),
            frame_id: 1.to_string(),
            id: i.to_string(),
            lifetime: None,
            frame_locked: false,
            metadata: Vec::new(),
            arrows: Vec::new(),
            cubes: Vec::new(),
            spheres: vec![sphere],
            cylinders: Vec::new(),
            lines: vec![line],
            triangles: Vec::new(),
            texts: Vec::new(),
            models: Vec::new(),
        };

        let sceneupdate = SceneUpdate {
            deletions: vec![],
            entities: vec![entity1],
        };

        let time_in_nsec = i as u64 * 1000000000;
        writer.write_to_known_channel(
            &MessageHeader {
                channel_id: sphere_channel,
                sequence: i as u32,
                log_time: time_in_nsec,
                publish_time: time_in_nsec
            },
            &sceneupdate.encode_to_vec()
        ).unwrap();

    }

    writer.finish().unwrap();
}



