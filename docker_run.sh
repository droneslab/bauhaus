#!/usr/bin/env bash
docker build -t darvis:latest . && docker run -it darvis:latest
# docker system prune -f

