#!/usr/bin/env bash
if [ "$#" -lt 1 ]; then
    echo "Usage: ./docker_run.sh [path to dataset directory]"
    exit
fi

datasetdir=$1

# stuff to get x11 forwarding working
export IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
echo "IP IS: " $IP
# xhost +$IP

docker build -t darvis:latest . && docker run --mount type=bind,source="$(pwd)",target=/darvis --volume $datasetdir:/datasets/ -e DISPLAY=$IP:0 -it darvis:latest
docker system prune -f
