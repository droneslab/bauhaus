use std::{borrow::Cow, fs::File, sync::Arc, collections::BTreeMap, fs, io::BufWriter};
use log::warn;
use mcap::{Schema, Channel, records::MessageHeader, Writer};
use opencv::prelude::{Mat, MatTraitConst, MatTraitConstManual};
use foxglove::{foxglove::items::{SceneEntity, SceneUpdate, SpherePrimitive, Vector3, Quaternion, Color, FrameTransform, LinePrimitive, line_primitive, Point3}, get_file_descriptor_set_bytes, make_pose};

use dvcore::{
    vis_schemas::{POSE_SCHEMA, IMAGE_SCHEMA, VisSchema},
    base::{ActorChannels, Actor},
};
use prost_types::Timestamp;
use crate::{
    dvmap::{map::{Map, Id}, pose::Pose},
    lockwrap::ReadOnlyWrapper,
    modules::image,
    actors::messages::{ShutdownMsg, TrajectoryMsg, VisFeaturesMsg}
};

pub struct DarvisVisualizer {
    actor_system: ActorChannels,
    map: ReadOnlyWrapper<Map>,
    writer: McapWriter,

    current_frame_id: u64,
    current_timestamp: u64,
    prev_pose: Point3,

    image_channel: u16,
    scene_update_channel: u16,
    transform_channel: u16,
}

impl Actor for DarvisVisualizer {
    fn run(&mut self) {
        loop {
            let message = self.actor_system.receive().unwrap();

            if let Some(msg) = message.downcast_ref::<VisFeaturesMsg>() {
                // self.draw_image(msg).expect("Visualizer could not draw image!");
                // self.draw_features(msg).expect("Visualizer could not draw features!");
                // self.write_image_to_file(msg).expect("Could not write image to file");
                self.current_frame_id += 1;
            } else if let Some(msg) = message.downcast_ref::<TrajectoryMsg>() {
                self.current_timestamp = msg.timestamp;
                self.update_scene(msg).expect("Visualizer could not draw pose!");
            } else if let Some(_) = message.downcast_ref::<ShutdownMsg>() {
                warn!("Closing mcap file");
                self.writer.finish().expect("Could not close file");
                break;
            } else {
                warn!("Visualizer received unknown message type!");
            }

        }
    }
}

impl DarvisVisualizer {
    pub fn new(actor_system: ActorChannels, map: ReadOnlyWrapper<Map>) -> DarvisVisualizer {
        let mut writer = McapWriter::new("results/out.mcap").unwrap();

        // Image 
        let image_channel = writer.create_and_add_channel(
            "foxglove.RawImage", "/camera",
        ).expect("Could not make image channel");

        // Scene contains...
        // Keyframes, current frame, and trajectory line connecting them all
        // Mappoints as 3d dots
        let scene_update_channel = writer.create_and_add_channel(
            "foxglove.SceneUpdate", "/scene",
        ).expect("Could not make current scene channel");

        // Frame transforms
        let transform_channel = writer.create_and_add_channel(
            "foxglove.FrameTransform", "/frame_transforms",
        ).expect("Could not make frame transform channel");

        // Write first frame transform
        let transform = FrameTransform {
            timestamp: Some(Timestamp { seconds: 0,  nanos: 0 }),
            parent_frame_id: "0".to_string(),
            child_frame_id: "1".to_string(),
            translation: Some(Vector3{ x: 0.0, y: 0.0, z: 0.0 }),
            rotation: Some(Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }),
        };
        writer.write(transform_channel, transform, 0, 0).unwrap();

        // Trajectory starts at 0,0
        let prev_pose = Point3 { x: 0.0, y: 0.0, z: 0.0 };

        return DarvisVisualizer{
            actor_system,
            map,
            writer,
            current_frame_id: 0,
            current_timestamp: 0,
            prev_pose,
            image_channel,
            scene_update_channel,
            transform_channel,
        };
    }

    fn write_image_to_file(&mut self, msg: &VisFeaturesMsg) -> Result<(), Box<dyn std::error::Error>> {
        let path = format!("results/images/{}-kp.png", msg.image_filename);
        image::write_features(&msg.image, &msg.keypoints, &path);
        Ok(())
    }

    // fn write_image_to_mcap(&mut self, image: &Mat, timestamp: u64) -> Result<(), Box<dyn std::error::Error>> {
    //     // Doesn't work...shows up as all black in foxglove for some reason
    //     let data = json!({
    //         "timestamp": {
    //             "sec": self.current_timestamp,
    //             "nsec": 1
    //         },
    //         "frame_id": self.current_frame_id,
    //         "width": image.cols(),
    //         "height": image.rows(),
    //         "encoding": "mono8",
    //         // Step is image cols * size of each element in bytes
    //         // For mono8 (CV_8UC1) each element is u8 -> size is 1
    //         // You can check this with ```size_of::<u8>()```
    //         "step": image.cols(),
    //         "data": Mat::data_bytes(&image)?
    //     });

    //     println!("Mat!!...{:?}",
    //         image
    //     );

    //     println!("Mat!!...{:?}",
    //         &image.data_typed::<u8>()?
    //     );

    //     println!("total bytes...{}, cols...{}, rows...{}",
    //         Mat::data_bytes(&image)?.len(),
    //         image.cols(),
    //         image.rows()
    //     );

    //     self.writer.write(self.image_channel, &data, timestamp, 0)?;

    //     Ok(())
    // }

    fn update_scene(&mut self, msg: &TrajectoryMsg) -> Result<(), Box<dyn std::error::Error>> {
        // Default visual stuff
        let sphere_color = Color { r: 1.0, g: 0.0, b: 0.0, a: 1.0 };
        let line_color = Color { r: 0.0, g: 1.0, b: 0.0, a: 1.0 };
        let sphere_size = Vector3 { x: 0.3, y: 0.3, z: 0.3 };

        let timestamp = Timestamp { seconds: msg.timestamp as i64, nanos: 0 };

        // Draw current pose as sphere
        let current_pose_sphere = SpherePrimitive {
            pose: Some(msg.pose.into()),
            size: Some(sphere_size.clone()),
            color: Some(sphere_color.clone()),
        };

        // Draw line connecting prev pose to current pose
        let points = vec![self.prev_pose.clone(), msg.pose.into()];
        let line = LinePrimitive {
            r#type: line_primitive::Type::LineStrip as i32,
            pose: make_pose(0.0, 0.0, 0.0),
            thickness: 1.0,
            scale_invariant: true,
            points,
            color: Some(line_color.clone()),
            colors: vec![],
            indices: vec![],
        };

        let entity1 = SceneEntity {
            timestamp: Some(timestamp),
            frame_id: 1.to_string(),
            id: "0".to_string(),
            lifetime: None,
            frame_locked: false,
            metadata: Vec::new(),
            arrows: Vec::new(),
            cubes: Vec::new(),
            spheres: vec![current_pose_sphere],
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

        self.writer.write(self.scene_update_channel, sceneupdate, msg.timestamp as u64, 0)?;

        Ok(())
    }

        // !! keyframes

        // const float &w = mKeyFrameSize;
        // const float h = w*0.75;
        // const float z = w*0.6;

        // Map* pActiveMap = mpAtlas->GetCurrentMap();
        // // DEBUG LBA
        // std::set<long unsigned int> sOptKFs = pActiveMap->msOptKFs;
        // std::set<long unsigned int> sFixedKFs = pActiveMap->msFixedKFs;

        // if(!pActiveMap)
        //     return;

        // const vector<KeyFrame*> vpKFs = pActiveMap->GetAllKeyFrames();

        // if(bDrawKF)
        // {
        //     for(size_t i=0; i<vpKFs.size(); i++)
        //     {
        //         KeyFrame* pKF = vpKFs[i];
        //         Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
        //         unsigned int index_color = pKF->mnOriginMapId;

        //         glPushMatrix();

        //         glMultMatrixf((GLfloat*)Twc.data());

        //         if(!pKF->GetParent()) // It is the first KF in the map
        //         {
        //             glLineWidth(mKeyFrameLineWidth*5);
        //             glColor3f(1.0f,0.0f,0.0f);
        //             glBegin(GL_LINES);
        //         }
        //         else
        //         {
        //             //cout << "Child KF: " << vpKFs[i]->mnId << endl;
        //             glLineWidth(mKeyFrameLineWidth);
        //             if (bDrawOptLba) {
        //                 if(sOptKFs.find(pKF->mnId) != sOptKFs.end())
        //                 {
        //                     glColor3f(0.0f,1.0f,0.0f); // Green -> Opt KFs
        //                 }
        //                 else if(sFixedKFs.find(pKF->mnId) != sFixedKFs.end())
        //                 {
        //                     glColor3f(1.0f,0.0f,0.0f); // Red -> Fixed KFs
        //                 }
        //                 else
        //                 {
        //                     glColor3f(0.0f,0.0f,1.0f); // Basic color
        //                 }
        //             }
        //             else
        //             {
        //                 glColor3f(0.0f,0.0f,1.0f); // Basic color
        //             }
        //             glBegin(GL_LINES);
        //         }

        //         glVertex3f(0,0,0);
        //         glVertex3f(w,h,z);
        //         glVertex3f(0,0,0);
        //         glVertex3f(w,-h,z);
        //         glVertex3f(0,0,0);
        //         glVertex3f(-w,-h,z);
        //         glVertex3f(0,0,0);
        //         glVertex3f(-w,h,z);

        //         glVertex3f(w,h,z);
        //         glVertex3f(w,-h,z);

        //         glVertex3f(-w,h,z);
        //         glVertex3f(-w,-h,z);

        //         glVertex3f(-w,h,z);
        //         glVertex3f(w,h,z);

        //         glVertex3f(-w,-h,z);
        //         glVertex3f(w,-h,z);
        //         glEnd();

        //         glPopMatrix();

        //         glEnd();
        //     }
        // }

        // if(bDrawGraph)
        // {
        //     glLineWidth(mGraphLineWidth);
        //     glColor4f(0.0f,1.0f,0.0f,0.6f);
        //     glBegin(GL_LINES);

        //     // cout << "-----------------Draw graph-----------------" << endl;
        //     for(size_t i=0; i<vpKFs.size(); i++)
        //     {
        //         // Covisibility Graph
        //         const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
        //         Eigen::Vector3f Ow = vpKFs[i]->GetCameraCenter();
        //         if(!vCovKFs.empty())
        //         {
        //             for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
        //             {
        //                 if((*vit)->mnId<vpKFs[i]->mnId)
        //                     continue;
        //                 Eigen::Vector3f Ow2 = (*vit)->GetCameraCenter();
        //                 glVertex3f(Ow(0),Ow(1),Ow(2));
        //                 glVertex3f(Ow2(0),Ow2(1),Ow2(2));
        //             }
        //         }

        //         // Spanning tree
        //         KeyFrame* pParent = vpKFs[i]->GetParent();
        //         if(pParent)
        //         {
        //             Eigen::Vector3f Owp = pParent->GetCameraCenter();
        //             glVertex3f(Ow(0),Ow(1),Ow(2));
        //             glVertex3f(Owp(0),Owp(1),Owp(2));
        //         }

        //         // Loops
        //         set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
        //         for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
        //         {
        //             if((*sit)->mnId<vpKFs[i]->mnId)
        //                 continue;
        //             Eigen::Vector3f Owl = (*sit)->GetCameraCenter();
        //             glVertex3f(Ow(0),Ow(1),Ow(2));
        //             glVertex3f(Owl(0),Owl(1),Owl(2));
        //         }
        //     }

        //     glEnd();
        // }

        // if(bDrawInertialGraph && pActiveMap->isImuInitialized())
        // {
        //     glLineWidth(mGraphLineWidth);
        //     glColor4f(1.0f,0.0f,0.0f,0.6f);
        //     glBegin(GL_LINES);

        //     //Draw inertial links
        //     for(size_t i=0; i<vpKFs.size(); i++)
        //     {
        //         KeyFrame* pKFi = vpKFs[i];
        //         Eigen::Vector3f Ow = pKFi->GetCameraCenter();
        //         KeyFrame* pNext = pKFi->mNextKF;
        //         if(pNext)
        //         {
        //             Eigen::Vector3f Owp = pNext->GetCameraCenter();
        //             glVertex3f(Ow(0),Ow(1),Ow(2));
        //             glVertex3f(Owp(0),Owp(1),Owp(2));
        //         }
        //     }

        //     glEnd();
        // }

        // vector<Map*> vpMaps = mpAtlas->GetAllMaps();

        // if(bDrawKF)
        // {
        //     for(Map* pMap : vpMaps)
        //     {
        //         if(pMap == pActiveMap)
        //             continue;

        //         vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

        //         for(size_t i=0; i<vpKFs.size(); i++)
        //         {
        //             KeyFrame* pKF = vpKFs[i];
        //             Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
        //             unsigned int index_color = pKF->mnOriginMapId;

        //             glPushMatrix();

        //             glMultMatrixf((GLfloat*)Twc.data());

        //             if(!vpKFs[i]->GetParent()) // It is the first KF in the map
        //             {
        //                 glLineWidth(mKeyFrameLineWidth*5);
        //                 glColor3f(1.0f,0.0f,0.0f);
        //                 glBegin(GL_LINES);
        //             }
        //             else
        //             {
        //                 glLineWidth(mKeyFrameLineWidth);
        //                 glColor3f(mfFrameColors[index_color][0],mfFrameColors[index_color][1],mfFrameColors[index_color][2]);
        //                 glBegin(GL_LINES);
        //             }

        //             glVertex3f(0,0,0);
        //             glVertex3f(w,h,z);
        //             glVertex3f(0,0,0);
        //             glVertex3f(w,-h,z);
        //             glVertex3f(0,0,0);
        //             glVertex3f(-w,-h,z);
        //             glVertex3f(0,0,0);
        //             glVertex3f(-w,h,z);

        //             glVertex3f(w,h,z);
        //             glVertex3f(w,-h,z);

        //             glVertex3f(-w,h,z);
        //             glVertex3f(-w,-h,z);

        //             glVertex3f(-w,h,z);
        //             glVertex3f(w,h,z);

        //             glVertex3f(-w,-h,z);
        //             glVertex3f(w,-h,z);
        //             glEnd();

        //             glPopMatrix();
        //         }
        //     }
        // }

        // !! mappoints
        // Map* pActiveMap = mpAtlas->GetCurrentMap();
        // if(!pActiveMap)
        //     return;

        // const vector<MapPoint*> &vpMPs = pActiveMap->GetAllMapPoints();
        // const vector<MapPoint*> &vpRefMPs = pActiveMap->GetReferenceMapPoints();

        // set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

        // if(vpMPs.empty())
        //     return;

        // glPointSize(mPointSize);
        // glBegin(GL_POINTS);
        // glColor3f(0.0,0.0,0.0);

        // for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
        // {
        //     if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
        //         continue;
        //     Eigen::Matrix<float,3,1> pos = vpMPs[i]->GetWorldPos();
        //     glVertex3f(pos(0),pos(1),pos(2));
        // }
        // glEnd();

        // glPointSize(mPointSize);
        // glBegin(GL_POINTS);
        // glColor3f(1.0,0.0,0.0);

        // for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
        // {
        //     if((*sit)->isBad())
        //         continue;
        //     Eigen::Matrix<float,3,1> pos = (*sit)->GetWorldPos();
        //     glVertex3f(pos(0),pos(1),pos(2));

        // }

}

pub struct McapWriter {
    writer: Writer<'static, BufWriter<File>>,
}

impl McapWriter {
    pub fn new(path: &str) -> Result<McapWriter, Box<dyn std::error::Error>> {
        Ok(McapWriter{
            writer: Writer::new(BufWriter::new(fs::File::create(path)?))?
        })
    }

    pub fn create_and_add_channel(
        &mut self,
        schema: &str,
        topic: &str,
    ) -> Result<u16, Box<dyn std::error::Error>> {
        let schema = Schema {
            name: String::from(schema),
            encoding: String::from("protobuf"),
            data: Cow::Borrowed(get_file_descriptor_set_bytes()),
        };
        let channel = Channel {
            topic: String::from(topic),
            schema: Some(Arc::new(schema)),
            message_encoding: String::from("protobuf"),
            metadata: BTreeMap::default()
        };
        let channel_id = self.writer.add_channel(&channel)?;

        Ok(channel_id)
    }

    pub fn write<T: prost::Message>(
        &mut self,
        channel_id: u16,
        data: T,
        timestamp: u64,
        sequence: u32,
    ) -> Result<(), Box<dyn std::error::Error>> {
        Ok(
            self.writer.write_to_known_channel(
                &MessageHeader {
                    channel_id,
                    sequence,
                    log_time: timestamp,
                    publish_time: timestamp,
                },
                &data.encode_to_vec()
            )?
        )
    }

    pub fn finish(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        Ok(self.writer.finish()?)
    }
}


impl From<Pose> for foxglove::foxglove::items::Pose {
    fn from(pose: Pose) -> foxglove::foxglove::items::Pose {
        let trans = pose.get_translation();
        let position = Vector3 { 
            x: trans[0], 
            y: trans[1], 
            z: trans[2]
        };

        let orientation = Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0
        };

        foxglove::foxglove::items::Pose {
            position: Some(position),
            orientation: Some(orientation),
        }
    }
}

impl From<Pose> for foxglove::foxglove::items::Point3 {
    fn from(pose: Pose) -> foxglove::foxglove::items::Point3 {
        let trans = pose.get_translation();
        Point3 { 
            x: trans[0], 
            y: trans[1], 
            z: trans[2]
        }
    }
}

