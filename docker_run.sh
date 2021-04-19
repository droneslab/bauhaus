#!/usr/bin/env bash
docker build -t darvis:latest . && docker run --mount type=bind,source="$(pwd)",target=/darvis -it darvis:latest
docker system prune -f
