sudo docker run --name remoteracer -it --rm \
 |   --privileged \
 |   --network=host \
 |   --mount type=volume,source=remote)racer,dst=/root/ros_ws \
 |   --env="DISPLAY=$DISPLAY" \
 |   --env="QT_X11_NO_MITSHM=1" \
 |   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
 |   --env="XAUTHORITY=$XAUTH" \
 |   --volume="$XAUTH:$XAUTH" \
 |   sisaha9/remote_racer_docker:latest