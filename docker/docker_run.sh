#!/usr/bin/env bash
if [ "$#" -lt 1 ]; then
    echo "Usage: ./docker_run.sh [path to dataset directory]"
    exit
fi

datasetdir=$1

docker build -t darvis:latest . && docker run --mount type=bind,source="$(pwd)",target=/darvis --mount type=bind,source="../g2o-bindings/",target=/g2o-bindings  --volume datasetdir:/datasets/ -it darvis:latest
docker system prune -f
