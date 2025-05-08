/// *** Structs to help get/set global configuration parameters that are read from the config file. *** //
/// 
/// This section is a little complicated and the details don't matter too much if you're just using it.
/// Basically, this implementation of Settings allows inserting a setting of 4 different types
/// (string, bool, f64, and i32) without needing to call a specific function for each type.
/// 
/// To insert a new parameter into SETTINGS:
///     SETTINGS.insert(SYSTEM, "show_visualizer", show_visualizer);
/// To get a parameter from SETTINGS:
///     let max_features= SETTINGS.get::<i32>(SYSTEM, "max_features".to_string());
/// 
use std::{collections::HashMap, sync::RwLock};
use lazy_static::*;
use linked_hash_map::LinkedHashMap;

use std::fs::File;
use std::io::Read;
extern crate yaml_rust;
use yaml_rust::yaml;
use yaml_rust::yaml::Yaml;

use crate::{matrix::DVMatrix4, sensor::{FrameSensor, ImuSensor, Sensor}};

pub static SYSTEM: &str = "SYSTEM"; 

//* GLOBAL SETTING PARAMETERS FOR LOOKUP */
pub struct Settings {
    // Lock is necessary because Settings is a static variable
    // https://stackoverflow.com/questions/34832583/global-mutable-hashmap-in-a-library
    settings: RwLock<HashMap<String, SettingBox>>,
}

lazy_static! {
    #[derive(Debug)]
    pub static ref SETTINGS: Settings = Settings {
        settings: RwLock::new(HashMap::new())
    };
}

impl Settings {
    pub fn get<T>(&self, module : &str, param: &str) -> T
    where Self: OverloadedSetting<T> {
        let key = format!("{}_{}", module, param);
        let unlocked_params = SETTINGS.settings.read().unwrap();
        let boxed_value = unlocked_params.get(&key).unwrap();
        return self.get_value_from_box(boxed_value);
    }
    pub fn insert<T: std::fmt::Debug>(&self, namespace: &str, key_param: &str, value: T)
    where Self: OverloadedSetting<T> {
        let key = format!("{}_{}", namespace, key_param);
        let value = self.make_box_from_value(value);
        let mut unlocked_params = SETTINGS.settings.write().unwrap();
        unlocked_params.insert(key, value);
    }
}

impl OverloadedSetting<String> for Settings {
    fn get_value_from_box(&self, boxed_value : &SettingBox) -> String {
        return boxed_value.string_field.as_ref().unwrap().to_string();
    }
    fn make_box_from_value(&self, value: String) -> SettingBox {
        return SettingBox {
            string_field: Some(value.clone()),
            bool_field: None,
            float_field: None,
            int_field: None,
            sensor_field: None,
            matrix_field: None
        };
    }
}

impl OverloadedSetting<bool> for Settings {
    fn get_value_from_box(&self, boxed_value : &SettingBox) -> bool {
        return *boxed_value.bool_field.as_ref().unwrap();
    }
    fn make_box_from_value(&self, value: bool) -> SettingBox  {
        return SettingBox {
            string_field: None,
            bool_field: Some(value),
            float_field: None,
            int_field: None,
            sensor_field: None,
            matrix_field: None
        };
    }
}

impl OverloadedSetting<f64> for Settings {
    fn get_value_from_box(&self, boxed_value : &SettingBox) -> f64 {
        return *boxed_value.float_field.as_ref().unwrap();
    }
    fn make_box_from_value(&self, value: f64) -> SettingBox  {
        return SettingBox {
            string_field: None,
            bool_field: None,
            float_field: Some(value),
            int_field: None,
            sensor_field: None,
            matrix_field: None
        };
    }
}

impl OverloadedSetting<i32> for Settings {
    fn get_value_from_box(&self, boxed_value : &SettingBox) -> i32 {
        return *boxed_value.int_field.as_ref().unwrap();
    }
    fn make_box_from_value(&self, value: i32) -> SettingBox  {
        return SettingBox {
            string_field: None,
            bool_field: None,
            float_field: None,
            int_field: Some(value),
            sensor_field: None,
            matrix_field: None
        };
    }
}

impl OverloadedSetting<DVMatrix4<f64>> for Settings {
    fn get_value_from_box(&self, boxed_value : &SettingBox) -> DVMatrix4<f64> {
        return boxed_value.matrix_field.as_ref().unwrap().clone(); // TODO Can we get rid of this clone?
    }
    fn make_box_from_value(&self, value: DVMatrix4<f64>) -> SettingBox  {
        return SettingBox {
            string_field: None,
            bool_field: None,
            float_field: None,
            int_field: None,
            sensor_field: None,
            matrix_field: Some(value),
        };
    }
}


impl OverloadedSetting<Sensor> for Settings {
    fn get_value_from_box(&self, boxed_value : &SettingBox) -> Sensor {
        return *boxed_value.sensor_field.as_ref().unwrap();
    }
    fn make_box_from_value(&self, value: Sensor) -> SettingBox  {
        return SettingBox {
            string_field: None,
            bool_field: None,
            float_field: None,
            int_field: None,
            sensor_field: Some(value),
            matrix_field: None
        };
    }
}

pub trait OverloadedSetting<T> {
    fn get_value_from_box(&self, boxed_value : &SettingBox) -> T;
    fn make_box_from_value(&self, value: T) -> SettingBox;
}

pub struct SettingBox {
    string_field: Option<String>,
    bool_field: Option<bool>,
    float_field: Option<f64>,
    int_field: Option<i32>,
    sensor_field: Option<Sensor>,
    matrix_field: Option<DVMatrix4<f64>>
}




// * LOADING CONFIGURATION FROM FILE *//

#[derive(Debug, Default, Clone)]
// Struct holding configuration parameters for a given actor
pub struct ActorConf{
    // Splitting up names and tags allows us to have other actors refer to an actor by name
    // without it being tied to a specific implementation. For example, LOCAL_MAPPING can refer
    // to TRACKING_BACKEND without needing to know about which file implements TRACKING_BACKEND.
    pub name: String,  // How this actor is referred to by other actors
    pub tag: String, // Tag to match up with executable in registered_actors.rs
    pub receiver_bound: Option<usize>,
}

#[derive(Debug, Default, Clone)]
// Struct holding configuration parameters for a given module
pub struct ModuleConf{
    pub name: String, // How this module is referred to by other actors/modules
    pub tag: String, // Tag to match up with executable in registered_actors.rs
}

pub fn load_config(system_fn: &String, camera_fn: &String) -> Result<(Vec<ActorConf>, Vec<ModuleConf>, String), Box<dyn std::error::Error>> {
    println!("Configs... System: {}, Camera: {}", system_fn, camera_fn);

    let (actor_println, mut module_println, log_level) = load_system_settings(system_fn);
    load_camera_settings(camera_fn, &mut module_println);

    Ok((actor_println, module_println, log_level))
}

fn load_system_settings(system_fn: &String) -> (Vec<ActorConf>, Vec<ModuleConf>, String) {
    let mut config_string = String::new();

    let mut f = File::open(system_fn).unwrap();
    f.read_to_string(&mut config_string).unwrap();
    let yaml_document = &yaml::YamlLoader::load_from_str(&config_string).unwrap()[0];

    println!("SYSTEM SETTINGS");

    // Load additional custom settings from config file
    let system_settings = &yaml::YamlLoader::load_from_str(&config_string).unwrap()[0]["system"];
    add_setting_bool(SYSTEM, "localization_only_mode", &system_settings["localization_only_mode"]);
    add_setting_string(SYSTEM, "vocabulary_file", &system_settings["vocabulary_file"]);
    add_setting_string(SYSTEM, "keyframe_trajectory_file_name", &system_settings["keyframe_trajectory_file_name"]);
    add_setting_string(SYSTEM, "camera_trajectory_file_name", &system_settings["camera_trajectory_file_name"]);
    add_setting_string(SYSTEM, "results_folder", &system_settings["results_folder"]);
    add_setting_string(SYSTEM, "first_actor_name", &system_settings["first_actor_name"]);
    add_setting_f64(SYSTEM, "fps", &system_settings["fps"]);
    add_setting_bool(SYSTEM, "check_deadlocks", &system_settings["check_deadlocks"]);
    let log_level = system_settings["log_level"].as_str().unwrap().to_owned();

    // Values from Kimera-style pipeline
    add_setting_i32(SYSTEM, "initial_k", &system_settings["initial_k"]);
    add_setting_i32(SYSTEM, "final_k", &system_settings["final_k"]);

    // Load sensor settings
    let framesensor = match system_settings["framesensor"].as_str().unwrap() {
        "Mono" => FrameSensor::Mono,
        "Stereo" => FrameSensor::Stereo,
        "RGBD" => FrameSensor::Rgbd,
        _ => panic!("Incompatible frame sensor type"),
    };
    let imusensor = match system_settings["imusensor"].as_str().unwrap() {
        "Some" => ImuSensor::Some,
        "None" => ImuSensor::None,
        _ => panic!("Incompatible IMU sensor type")
    };
    let sensor = Sensor(framesensor, imusensor);
    SETTINGS.insert(SYSTEM, "sensor", sensor);
    println!("\t {} = {}", "SENSOR", sensor);

    // Load actors
    let mut actor_println = Vec::<ActorConf>::new();
    for actor in yaml_document["actors"].as_vec().unwrap() {
        let h = &actor.as_hash().unwrap();

        let a_conf = ActorConf {
            tag: get_val(h, "tag").as_str().unwrap().to_string(),
            name: get_val(h, "name").as_str().unwrap().to_string(),
            receiver_bound: match get_val(h, "receiver_bound").as_i64().unwrap() == -1 {
                true => None,
                false => Some(get_val(h, "receiver_bound").as_i64().unwrap() as usize)
            }
        };

        actor_println.push(a_conf.clone());

        add_settings(get_val(h, "settings").as_vec().unwrap(), &a_conf.name);
    }

    // Load modules
    let mut module_println = Vec::<ModuleConf>::new();
    for module in yaml_document["modules"].as_vec().unwrap() {
        let h = module.as_hash().unwrap();
        let m_conf = ModuleConf {
            name: get_val(h, "name").as_str().unwrap().to_string(),
            tag: get_val(h, "tag").as_str().unwrap().to_string(),
        };

        SETTINGS.insert(&m_conf.name, "module_tag", m_conf.tag.clone());

        add_settings(get_val(h, "settings").as_vec().unwrap(), &m_conf.name);
        module_println.push(m_conf);
    }

    (actor_println, module_println, log_level)
}

fn load_camera_settings(camera_fn: &String, module_println: &mut Vec<ModuleConf>) {
    let mut config_string = String::new();
    let mut f = File::open(camera_fn).unwrap();
    f.read_to_string(&mut config_string).unwrap();

    println!("CAMERA SETTINGS");

    // Load additional custom settings from config file
    let yaml_document = &yaml::YamlLoader::load_from_str(&config_string).unwrap()[0];
    add_setting_f64("CAMERA", "fx", &yaml_document["fx"]);
    add_setting_f64("CAMERA", "fy", &yaml_document["fy"]);
    add_setting_f64("CAMERA", "cx", &yaml_document["cx"]);
    add_setting_f64("CAMERA", "cy", &yaml_document["cy"]);
    add_setting_i32("CAMERA", "width", &yaml_document["width"]);
    add_setting_i32("CAMERA", "height", &yaml_document["height"]);
    add_setting_i32("CAMERA", "stereo_overlapping_begin", &yaml_document["stereo_overlapping_begin"]);
    add_setting_i32("CAMERA", "stereo_overlapping_end", &yaml_document["stereo_overlapping_end"]);

    add_setting_f64("CAMERA", "k1", &yaml_document["k1"]);
    add_setting_f64("CAMERA", "k2", &yaml_document["k2"]);
    add_setting_f64("CAMERA", "p1", &yaml_document["p1"]);
    add_setting_f64("CAMERA", "p2", &yaml_document["p2"]);
    add_setting_f64("CAMERA", "k3", &yaml_document["k3"]);
    if yaml_document["k1"].as_f64().unwrap() != 0.0 {
        SETTINGS.insert(&"CAMERA", &"need_to_undistort", true);
    }


    match SETTINGS.get::<Sensor>(SYSTEM, "sensor") {
        Sensor(FrameSensor::Stereo, _) | Sensor(FrameSensor::Rgbd, _) => {
            add_setting_f64("CAMERA", "stereo_baseline_times_fx", &yaml_document["stereo_baseline_times_fx"]);
            add_setting_i32("CAMERA", "thdepth", &yaml_document["thdepth"]);
        },
        Sensor(FrameSensor::Mono, _) => {
            SETTINGS.insert(&"CAMERA", &"stereo_baseline_times_fx", 0.0);
            println!("\t {} {} = {}", "CAMERA", "stereo_baseline_times_fx", 0.0);
            SETTINGS.insert(&"CAMERA", &"thdepth", 0);
            println!("\t {} {} = {}", "CAMERA", "thdepth", 0);

        }
    }

    if !yaml_document["new_height"].is_badvalue() {
        let new_height = yaml_document["new_height"].as_i64().unwrap() as i32;
        let old_height = yaml_document["height"].as_i64().unwrap();
        let scale_row_factor = (new_height as f64) / (old_height as f64);

        SETTINGS.insert(&"CAMERA", &"fy", yaml_document["fy"].as_f64().unwrap() * scale_row_factor);
        SETTINGS.insert(&"CAMERA", &"cy", yaml_document["cy"].as_f64().unwrap() * scale_row_factor);
        add_setting_i32("CAMERA", "height", &yaml_document["new_height"]);
    }

    if !yaml_document["new_width"].is_badvalue() {
        let new_width = yaml_document["new_width"].as_i64().unwrap() as i32;
        let old_width = yaml_document["width"].as_i64().unwrap();
        let scale_col_factor = (new_width as f32) / (old_width as f32);

        SETTINGS.insert(&"CAMERA", &"fx", yaml_document["fx"].as_f64().unwrap() * scale_col_factor as f64);
        SETTINGS.insert(&"CAMERA", &"cx", yaml_document["cx"].as_f64().unwrap() * scale_col_factor as f64);
        add_setting_i32("CAMERA", "width", &yaml_document["new_width"]);
        SETTINGS.insert(&"CAMERA", &"need_to_resize", true);
    } else {
        SETTINGS.insert(&"CAMERA", &"need_to_resize", false);
    }

    // Not all datasets have imu printlnrmation, but that only matters if imu is used
    // match SETTINGS.get::<Sensor>(SYSTEM, "sensor").imu() {
    //     ImuSensor::Some => {
            add_setting_f64("IMU", "noise_gyro", &yaml_document["imu__noise_gyro"]);
            add_setting_f64("IMU", "noise_acc", &yaml_document["imu__noise_acc"]);
            add_setting_f64("IMU", "gyro_walk", &yaml_document["imu__gyro_walk"]);
            add_setting_f64("IMU", "acc_walk", &yaml_document["imu__acc_walk"]);
            add_setting_f64("IMU", "frequency", &yaml_document["imu__frequency"]);
            add_setting_imu_matrix("IMU", "T_b_c1", vec![&yaml_document["imu__T_b_c1_row1"],&yaml_document["imu__T_b_c1_row2"],&yaml_document["imu__T_b_c1_row3"],&yaml_document["imu__T_b_c1_row4"]]);
    //     },
    //     _ => {}
    // };

    // KIMERA
    add_setting_f64("CAMERA", "imu_rate", &yaml_document["imu__frequency"]);

    // Add dataset name
    if SETTINGS.get::<Sensor>(SYSTEM, "sensor").is_imu() {
        add_setting_f64("IMU", "initialPositionSigma", &yaml_document["initialPositionSigma"]);
        add_setting_f64("IMU", "initialRollPitchSigma", &yaml_document["initialRollPitchSigma"]);
        add_setting_f64("IMU", "initialYawSigma", &yaml_document["initialYawSigma"]);
        add_setting_f64("IMU", "initialVelocitySigma", &yaml_document["initialVelocitySigma"]);
        add_setting_f64("IMU", "initialAccBiasSigma", &yaml_document["initialAccBiasSigma"]);
        add_setting_f64("IMU", "initialGyroBiasSigma", &yaml_document["initialGyroBiasSigma"]);
        add_setting_f64("IMU", "imu_integration_sigma", &yaml_document["imu_integration_sigma"]);
        add_setting_f64("IMU", "imu_bias_init_sigma", &yaml_document["imu_bias_init_sigma"]);
    }

    // Add camera module to module_println so it gets spawned
    let camera_module = ModuleConf {
        name: "CAMERA".to_string(),
        tag: "CAMERA".to_string(),
    };
    module_println.push(camera_module);


}

fn add_settings(settings: &Vec<Yaml>, namespace: &String) -> Option<()> {
    for s in 0..settings.len() {
        let setting = settings[s].as_hash()?;
        let s_name = setting[&Yaml::String("name".to_string())].as_str()?.to_string();
        let s_value = &setting[&Yaml::String("value".to_string())];
        let s_type = setting[&Yaml::String("type".to_string())].as_str()?.to_string();

        match s_type.as_ref() {
            "bool" => add_setting_bool(namespace, &s_name, s_value),
            "i32" => add_setting_i32(namespace, &s_name, s_value),
            "f64" => add_setting_f64(namespace, &s_name, s_value),
            "string" => add_setting_string(namespace, &s_name, s_value),
            _ => panic!("Incompatible type {} for setting {} in {}", s_type, s_value.as_str()?.to_string(), s_name)
        };
    }
    Some(())
}

fn add_setting_bool(namespace: &str, key: &str, value: &Yaml) {
    let val = value.as_bool().unwrap();
    SETTINGS.insert(&namespace, &key, val);
    println!("\t {} {} = {}", namespace, key, val);
}
fn add_setting_i32(namespace: &str, key: &str, value: &Yaml) {
    let val = value.as_i64().unwrap() as i32;
    SETTINGS.insert(&namespace, &key, val);
    println!("\t {} {} = {}", namespace, key, val);
}
fn add_setting_f64(namespace: &str, key: &str, value: &Yaml) {
    let val = value.as_f64().unwrap();
    SETTINGS.insert(&namespace, &key, val);
    println!("\t {} {} = {}", namespace, key, val);
}
fn add_setting_string(namespace: &str, key: &str, value: &Yaml) {
    let val = value.as_str().unwrap().to_string();
    SETTINGS.insert(&namespace, &key, val.clone());
    println!("\t {} {} = {}", namespace, key, val);
}
fn add_setting_imu_matrix(namespace: &str, key: &str, matrix_rows: Vec<&Yaml>) {
    // This is so hacky ... doesn't accept different-sized matrixes than 4x4

    let mut array = [[0f64; 4]; 4];
    for row in 0..4 {
        let curr_matrix_row = matrix_rows[row].as_vec().unwrap();
        for col in 0..4 {
            array[row][col] = curr_matrix_row[col].as_f64().unwrap();
        }
    }

    // let mat = opencv::core::Mat::from_slice_2d(& array).expect("Failed to create matrix");
    let mat = nalgebra::Matrix4::from(array);
    let matrix: DVMatrix4<f64> = DVMatrix4::<f64>::new(mat);

    SETTINGS.insert(&namespace, &key, matrix);
}
fn get_val<'a>(hashmap: &'a LinkedHashMap<Yaml, Yaml>, string: &str) -> &'a Yaml {
    &hashmap[&Yaml::String(string.to_string())]
}

