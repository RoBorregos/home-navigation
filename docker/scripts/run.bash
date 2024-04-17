#!/bin/bash

# Variables required for logging as a user with the same id as the user running this script
export LOCAL_USER_ID=`id -u $USER`
export LOCAL_GROUP_ID=`id -g $USER`
export LOCAL_GROUP_NAME=`id -gn $USER`
DOCKER_USER_ARGS="--env LOCAL_USER_ID --env LOCAL_GROUP_ID --env LOCAL_GROUP_NAME"

# Variables for forwarding ssh agent into docker container
SSH_AUTH_ARGS=""
if [ ! -z $SSH_AUTH_SOCK ]; then
    DOCKER_SSH_AUTH_ARGS="-v $(dirname $SSH_AUTH_SOCK):$(dirname $SSH_AUTH_SOCK) -e SSH_AUTH_SOCK=$SSH_AUTH_SOCK"
fi

DOCKER_NETWORK_ARGS="--net host"
if [[ "$@" == *"--net "* ]]; then
    DOCKER_NETWORK_ARGS=""
fi

CONTAINER_NAME="nav"
VOLUME_COMMANDS=""
for i in "$@"
do
case $i in
    # Receive volumes to mount from command line, through the --volumes argument, each volume should be separated by a comma
    --volumes=*)
    for volume in $(echo ${i#*=} | tr "," "\n")
    do
        # If the path starts with ~, expand it to the user's home directory
        if [[ "$volume" == ~* ]]; then
            resolved_path="${volume/#\~/$HOME}"
        else
            resolved_path=$(realpath "$volume")
        fi
        folder_name=$(basename "$resolved_path")
        VOLUME_COMMANDS="$VOLUME_COMMANDS -v $resolved_path:/workspace/$folder_name"
    done
    shift # past argument=value
    ;;
    --name=*)
    # if the name is not empty, set the container name
    if [ -n "${i#*=}" ]; then
        CONTAINER_NAME="${i#*=}"
    fi
    shift # past argument=value
    ;;
    --xavier)
    IMAGE_NAME="nav-xavier"
    shift # past argument with no value
    ;;
    --mediapipe)
    IMAGE_NAME="nav-mediapipe"
    shift # past argument with no value
    ;;
    *)
          # unknown option
    ;;
esac
done

IMAGE_NAME="nav"


echo "Container name: $CONTAINER_NAME"
echo "Volumes to mount: $VOLUME_COMMANDS"

DOCKER_COMMAND="docker run"

xhost +
$DOCKER_COMMAND -it -d\
    $DOCKER_USER_ARGS \
    $DOCKER_SSH_AUTH_ARGS \
    $DOCKER_NETWORK_ARGS \
    $ADDITIONAL_COMMANDS \
    --add-host nano:192.168.31.123 \
    --add-host rbrgs:192.168.31.23 \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /var/run/docker.sock:/var/run/docker.sock \
    -v /dev:/dev \
    --device /dev/video0:/dev/video0 \
    $VOLUME_COMMANDS \
    -w /workspace/ws \
    --name=$CONTAINER_NAME \
    $IMAGE_NAME \
    bash