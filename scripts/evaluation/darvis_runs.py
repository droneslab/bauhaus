from darvis_configs import *

import argparse
import os
def main():

    # # Parse the command line arguments
    parser = argparse.ArgumentParser(description='Run the Darvis pipeline')
    parser.add_argument('pipeline', type=str,  help='Name of the pipeline')
    parser.add_argument('dataset', type=str,  help='Name of the dataset')
    parser.add_argument('sequence', type=int,  help='Sequence number')
    parser.add_argument('dataset_path', type=str,  help='Path to the dataset upto sequences folder if KITTI dataset is used')
    parser.add_argument('--build', type=str, default='release', help='Build type')
    parser.add_argument('--project_root', type=str, default='../../darvis/', help='Path to the Darvis project root')
    args = parser.parse_args()

    # Run the Darvis pipeline
    pipeline = args.pipeline # 'PIPELINE_ORBSLAM'
    dataset = args.dataset #'KITTI'
    sequence = args.sequence # 3
    dataset_path = args.dataset_path # '/Volumes/dl-primary-pool/lidar-research/Geometric_Seg_Pranay/datasets/data_odometry_labels/dataset/sequences'
    darvis_pipeline = DarvisPipeline(pipeline, dataset, sequence, dataset_path, args.build, args.project_root)
    # darvis_pipeline.print_configs()

    config_dir_name = 'autogen_darvis_configs'
    system_config_file = os.path.join(config_dir_name,args.pipeline+'_system_config.yaml')
    dataset_config_file = os.path.join(config_dir_name,f'{dataset}_{sequence}.yaml')

    configs_dir = os.path.join(args.project_root, config_dir_name)

    if not os.path.exists(configs_dir):
        os.makedirs(configs_dir)

    darvis_pipeline.run_pipeline(system_config_file, dataset_config_file)

    



if __name__ == '__main__':
    main()


