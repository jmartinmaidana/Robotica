#!/bin/bash
set -eo pipefail
DEFAULT_HOST="$WO_HOST"
DEFAULT_DIR="$WO_DIR"

# Parse args for overrides
POSITIONAL=()
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    --hostname)
    export WO_HOST="$2"
    shift # past argument
    shift # past value
    ;;
        --dir)

    shift # past argument
    shift # past value
    ;;
    *)    # unknown option
    POSITIONAL+=("$1") # save it in an array for later
    shift # past argument
    ;;
esac
done
set -- "${POSITIONAL[@]}" # restore positional parameter

usage(){
  echo "Usage: $0 <command>"
  echo
  echo "This program helps to manage the setup/teardown of the docker containers for running ros2_robotica. We recommend that you read the full documentation of docker at https://docs.docker.com if you want to customize your setup."
  echo 
  echo "Command list:"
  echo "        start [options]         Start ros2_robotica container"
  echo "        build                   Builds ros2_robotica image"
  echo "        stop                    Stop ros2_robotica"
  echo "        down                    Stop and remove ros2_robotica's docker containers"
  echo "        rebuild                 Rebuild docker image and perform cleanups"
  echo "        open                    Open terminal to run scripts and whatever"
  echo ""
  #echo "Options:"
  #echo "       --hostname      <hostname>      Set the hostname that PointLearning will be accessible from (default: $DEFAULT_HOST)"
  #echo "       ---dir  <path>  Path where data will be persisted (default: $DEFAULT_DIR (docker named volume))"
  exit
}


run(){
  echo "$1"
  eval "$1"
}

start(){
        xhost +local:docker
        run "podman run --rm --name ros2_robotica --shm-size=256m -v ./volume2:/root/ros2_ws/src/robotica --env DISPLAY=$DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --env QT_X11_NO_MITSHM=1 -it -d ros2_robotica "
  # -it is for having an interactive terminal, and -d is enable reenter after closing the terminal
}

down(){
  run "podman stop ros2_robotica" 
  run "podman rm ros2_robotica"
}

rebuild(){
  down
  run "podman build -t ros2_robotica . "
}

build(){
  run "podman build -t ros2_robotica ."
}

open(){
  run "podman exec -it ros2_robotica bash"
}


if [[ $1 = "start" ]]; then
        start
elif [[ $1 = "stop" ]]; then
        echo "Stopping ros2_robotica..."
        run "podman stop ros2_robotica"
elif [[ $1 = "down" ]]; then
        echo "Tearing down ros2_robotica..."
        down
elif [[ $1 = "rebuild" ]]; then
        echo  "Rebuilding ros2_robotica..."
        rebuild
elif [[ $1 = "build" ]]; then
        echo  "Building ros2_robotica..."
        build
elif [[ $1 = "open" ]]; then
        echo "Opening terminal..."
        open
else
        usage
fi

