#!/bin/bash

sudo docker run --name remoteracer -it --rm \
   --privileged \
   --mount type=volume,source=remote_racer,dst=/root/ros_ws \
   --volume="/dev:/dev:rw" \
   --network=host remote_racer_img
#   --env="DISPLAY=$DISPLAY" \
#   --env="QT_X11_NO_MITSHM=1" \
#   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
#   --env="XAUTHORITY=$XAUTH" \
#   --volume="$XAUTH:$XAUTH" \
