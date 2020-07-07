#!/usr/bin/env bash
#
# Typical usage: ./join.bash subt
#

xhost +
containerid=$(docker ps -aqf "ancestor=${IMG}") && echo $containerid
docker exec -it \
    --privileged \
    -e DISPLAY=${DISPLAY} \
    -e LINES="$(tput lines)" \
    sis_competition \
    bash
xhost -
