#!/usr/bin/env bash

COLOR_RED='\033[0;31m'
COLOR_YELLOW='\033[0;33m'
COLOR_NC='\033[0m'

# Make sure processes in the container can connect to the x server
# Necessary so gazebo can create a context for OpenGL rendering (even headless)
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<<"$xauth_list")
    if [ ! -z "$xauth_list" ]; then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

#Prevent executing "docker run" when xauth failed.
if [ ! -f $XAUTH ]; then
    echo "[$XAUTH] was not properly created. Exiting..."
    exit 1
fi

#
# Specify tag name
#
if [ $# -gt 0 ]; then
    if [ "$1" == "same" ]; then
        echo -e "RUN: \"docker exec\""
    else
        echo -e "RUN: \"docker\""
        DOCKER_TAG="$1"
    fi
else
    echo -e "${COLOR_RED}Usage: source docker_run.sh [tag_name | same]${COLOR_NC}"
fi

#
# Execute command
#
if [ $# -gt 0 ]; then
    if [ "$1" == "same" ]; then
        docker exec -it sis_competition bash
    else
        docker run -it \
            --rm \
            --name sis_competition \
            --network host \
            --privileged \
            --user root \
            --runtime nvidia \
            -v "/etc/localtime:/etc/localtime:ro" \
            -v "/dev:/dev" \
            -v "/home/$USER/sis_competition_template/competition_modules:/home/sis/sis_competition_2020/catkin_ws/src" \
            -v "/home/$USER/.bashrc:/home/sis/.bashrc" \
            -e DISPLAY=$DISPLAY \
            -e XAUTHORITY=$XAUTH \
            -v "$XAUTH:$XAUTH" \
            -v "/tmp/.X11-unix/:/tmp/.X11-unix" \
            cihuang123/sis_competition_2020:$DOCKER_TAG \
            bash
    fi
else
    echo "please provide docker tag name."
fi
