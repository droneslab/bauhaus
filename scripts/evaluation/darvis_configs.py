KITTI_COMMON = '''

k1: 0.0
k2: 0.0
p1: 0.0
p2: 0.0
k3: 0.0

# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
RGB: 1

# Camera resolution
width: 1241
height: 376

stereo_overlapping_begin: 0
stereo_overlapping_end: 1000

'''


KITTI00_02 = '''

# Camera calibration and distortion parameters (OpenCV) 
fx: 718.856
fy: 718.856
cx: 607.1928
cy: 185.2157

stereo_baseline_times_fx: 386.1448
thdepth: 35

'''

KITTI03 = '''

# Camera calibration and distortion parameters (OpenCV) 
fx: 721.5377
fy: 721.5377
cx: 609.5593
cy: 172.854

stereo_baseline_times_fx: 387.5744
thdepth: 40

'''

KITTI04_12 = '''

# Camera calibration and distortion parameters (OpenCV) 
fx: 707.0912
fy: 707.0912
cx: 601.8873
cy: 183.1104

k1: 0.0
k2: 0.0
p1: 0.0
p2: 0.0
k3: 0.0

stereo_baseline_times_fx: 379.8145
thdepth: 40


'''


ORBSLAM_KITTI = '''

# Validate yaml with: https://yamlchecker.com/

master:
       name: manager
       address: localhost
       file: main.rs

system:
      localization_only_mode: false
      vocabulary_file: "../Vocabulary/ORBvoc.txt"
      framesensor: Mono
      imusensor: None
      first_actor_name: TRACKING_FRONTEND # Name of actor to kick off pipeline

      # FPS
      fps: 10.0
      use_timestamps_file: false # Instead of running at a fixed fps, use timestamps from the dataset

      # RESULTS
      results_folder: "results"
      trajectory_file_name: "trajectory.txt"

      # DEBUGGING
      log_level: debug # options are [trace, debug, info, warn, error]
      check_deadlocks: true # Spawns a new thread to check for deadlocks every 2 secs, but slows stuff down.
      # To enable profiling, set features = ["enable"] for tracy-client in Cargo.toml
      # To turn on visualizer, uncomment visualizer actor below

modules:
      - 
        name: FEATURE_DETECTION
        tag: "feature detection"
        settings:
          - 
            name: max_features
            value: 2000
            type: i32
          - 
            name: scale_factor
            value: 1.2
            type: f64
          - 
            name: n_levels
            value: 8
            type: i32
          - 
            name: ini_th_fast
            value: 20
            type: i32
          - 
            name: min_th_fast
            value: 7
            type: i32

      -
        name: MATCHER
        tag: "matcher"
        settings:
          - 
            name: far_points_threshold
            value: 20.0
            type: f64
      -
        name: FEATURES
        tag: "features"
        settings:
          - 
            name: frame_grid_rows
            value: 48
            type: i32
          - 
            name: frame_grid_cols
            value: 64
            type: i32

            
actors:


'''


VISUALIZER_ACTOR = '''

      -
       name: VISUALIZER
       tag: visualizer
       receiver_bound: -1
       settings:
          -
            name: stream # if true, stream to websocket. if false, save to mcap file
            value: false
            type: bool
          - name: port # port used if streaming
            value: 8765
            type: i32
          - 
            name: image_draw_type
            value: plain # options are [none, plain, features, featuresandmatches]
            type: string
          -
            name: mcap_file_path
            value: "results/out.mcap"
            type: string
          - name: draw_all_mappoints # Not sure this works right now... If true, draws all mappoints in the map. If false, only draws matches and recently created mappoints. Set to false to save space.
            value: false
            type: bool
          - name: draw_local_mappoints # Whether to draw local mappoints (not tracked, but in local map based on local keyframes)
            value: true
            type: bool
          - name: draw_connected_kfs # You can set this to be true but it doesn't look good. Need to figure out some nicer way to distribute the points so it's actually readable
            value: true
            type: bool
          - name: draw_only_trajectory
            value: false
            type: bool


'''

ORBSLAM_TRACKING_FRONTEND_ACTOR = '''
      -
       name: TRACKING_FRONTEND
       tag: "orbslam tracking frontend"
       receiver_bound: 10
       settings:
          -
'''

ORBSLAM_TRACKING_BACKEND_ACTOR = '''
      -
       name: TRACKING_BACKEND
       tag: "orbslam tracking backend"
       receiver_bound: 10
       possible_paths:
                     -
                      from: tracking_frontend
                      to: localmapping
                      input: keypoints, descriptors
                      output: newkeyframe
       settings:
          - 
            name: recently_lost_cutoff
            value: 5
            type: i32
          - 
            name: frames_to_reset_IMU
            value: 30
            type: i32
          - 
            name: insert_KFs_when_lost
            value: true
            type: bool
          - 
            name: min_num_features
            value: 2000
            type: i32
          - 
            name: min_frames_to_insert_kf
            value: 0
            type: i32
          -
            name: max_frames_to_insert_kf
            value: 10
            type: i32
'''

ORBSLAM_LOCAL_MAPPING_ACTOR = '''
      -
       name: LOCAL_MAPPING
       tag: "orbslam local mapping"
       receiver_bound: 3
       settings:
          - 
            name: recently_lost_cutoff
            value: 5
            type: i32
          - 
            name: insert_KFs_when_lost
            value: true
            type: bool
          - 
            name: min_num_features
            value: 2000
            type: i32
'''

ORBSLAM_LOOP_CLOSING_ACTOR = '''
      -
       name: LOOP_CLOSING
       tag: "orbslam2 loop closing"
       receiver_bound: -1
       settings:
          - 
            name: covisibility_consistency_threshold
            value: 3
            type: i32
'''



PIPELINE_ORBSLAM = ORBSLAM_KITTI + ORBSLAM_TRACKING_FRONTEND_ACTOR + ORBSLAM_TRACKING_BACKEND_ACTOR + ORBSLAM_LOCAL_MAPPING_ACTOR + ORBSLAM_LOOP_CLOSING_ACTOR 

PIPELINE_ORBSLAM_VIZ = PIPELINE_ORBSLAM = ORBSLAM_KITTI + VISUALIZER_ACTOR+ ORBSLAM_TRACKING_FRONTEND_ACTOR + ORBSLAM_TRACKING_BACKEND_ACTOR + ORBSLAM_LOCAL_MAPPING_ACTOR + ORBSLAM_LOOP_CLOSING_ACTOR 



all_names_map = {
    'KITTI00_02':KITTI_COMMON+ KITTI00_02,
    'KITTI03': KITTI_COMMON+KITTI03,
    'KITTI04_12': KITTI_COMMON + KITTI04_12,
    'PIPELINE_ORBSLAM': PIPELINE_ORBSLAM,
    'PIPELINE_ORBSLAM_VIZ': PIPELINE_ORBSLAM_VIZ
}

pipeline_names = ['PIPELINE_ORBSLAM', 'PIPELINE_ORBSLAM_VIZ']
dataset_names = ['KITTI00_02', 'KITTI03', 'KITTI04_12']

supported_datasets = ['KITTI']


import os

class DarvisPipeline:
    def __init__(self, pipeline, dataset, sequence, dataset_path, build='release', project_root='../../darvis/'):
        # Initialize the Darvis pipeline
        if pipeline not in pipeline_names:
            print("Possible pipeline names are:")
            print(pipeline_names)
            raise ValueError('Invalid pipeline name')
        
        if dataset == 'KITTI':
            seq_str = ''
            if sequence >=0 and sequence <= 2:
                seq_str = '00_02'
            elif sequence == 3:
                seq_str = '03'
            elif sequence >= 4 and sequence <= 12:
                seq_str = '04_12'
            else:
                raise ValueError('Invalid sequence number')
            dataset_config = 'KITTI' + seq_str

        if dataset_config not in dataset_names:
            raise ValueError('Invalid dataset name')
        


        self.system_config = all_names_map[pipeline]
        self.dataset_config = all_names_map[dataset_config]
        seq_str = str(sequence).zfill(2)
        self.dataset_path = os.path.join(dataset_path, seq_str)
        self.build = build
        self.project_root = project_root
        self.dataset = dataset
        self.pipeline = pipeline
        self.sequence = sequence


    def save_configs(self, system_config_file, dataset_config_file):
        # Save the Darvis pipeline configuration files
        with open(system_config_file, 'w') as f:
            f.write(self.system_config)
        with open(dataset_config_file, 'w') as f:
            f.write(self.dataset_config)



    def print_configs(self):
        # Run the Darvis pipeline
        print('Running the Darvis pipeline...')
        print('System config:')
        print(self.system_config)
        print('Dataset config:')
        print(self.dataset_config)

    def run_pipeline(self, system_config_file, dataset_config_file):
        # Run the Darvis pipeline
        # Save the Darvis pipeline configuration files
        full_system_config_file = os.path.join(self.project_root, system_config_file)
        full_dataset_config_file = os.path.join(self.project_root, dataset_config_file)
        self.save_configs(full_system_config_file, full_dataset_config_file)

        print('Running the Darvis pipeline...')
        full_command = f'cd {self.project_root}; cargo run --{self.build} {self.dataset_path} {system_config_file} {dataset_config_file}'

        print(full_command)
        os.system(full_command)

        # save trajectory and mcap to run name folder
        results_dir = os.path.join(self.project_root, 'autogen_results')
        full_dir = os.path.join(results_dir, self.dataset, self.pipeline)
        seq_str = str(self.sequence).zfill(2)
        os.makedirs(full_dir, exist_ok=True)
        print(f'Copying results to {full_dir}')
        #move trajecotry.txt from results/trajectory.txt to full_dir/trajectory.txt
        os.system(f'mv {self.project_root}results/trajectory.txt {full_dir}/{seq_str}.txt')
        #move out.mcap from results/out.mcap to full_dir/out.mcap
        os.system(f'mv {self.project_root}results/out.mcap {full_dir}/out.mcap')





