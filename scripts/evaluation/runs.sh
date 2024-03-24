#!/bin/bash


dataset_path="/datasets/sequences"

pipeline="PIPELINE_ORBSLAM"
dataset="KITTI"

st=0
d=0
for ((i=st; i<=d; i++)); do
   echo "Run $i"
    echo python3 darvis_runs.py ${pipeline} ${dataset} $i  ${dataset_path}
done


