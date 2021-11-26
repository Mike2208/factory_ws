#!/bin/bash
xhost +

docker run --gpus all --rm \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --device=/dev/dri \
        --group-add video \
        --device=/dev/snd:/dev/snd \
        --group-add audio \
        --net=host \
        --privileged \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --env ROS_MASTER_URI="http://localhost:11311" \
        --env GAZEBO_MASTER_URI="http://localhost:11345" \
        --env NVIDIA_VISIBLE_DEVICES=0 \
        --name "promen_aid_sim_instance" \
	-v /var/run/docker.sock:/var/run/docker.sock \
        "$@"
