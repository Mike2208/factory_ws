FROM osrf/ros:noetic-desktop-full
MAINTAINER Alessandro Di Fava <adifava@robontik.es>

# Non Root user
ARG user_name=ros
ARG user_uid=1000
ARG user_home=/home/$user_name
ARG user_shell=/bin/bash
ARG ck_dir=$user_home/factory_ws
ARG ck_src_dir=$ck_dir/src

RUN useradd -m -d $user_home -s $user_shell -u $user_uid $user_name \
	&& echo "PS1='\[\e]0;\u@\h: \w\a\]${debian_chroot:+($debian_chroot)}\[\033[01;33m\]\u\[\033[00m\]@\[\033[01;31m\]\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> ~/.bashrc

ENV DEBIAN_FRONTEND noninteractive
#Instalacion paquetes deb (paquetes basicos, instalación del sistema y finalmente todos los paquetes de ros)
RUN apt-get update \
	&& apt-get install -q -y \
		wget \
		apt-utils \
		# dialog \
		sudo \
	&& apt-get clean -q -y \
	&& apt-get autoremove -q -y \
	&& rm -rf /var/lib/apt/lists/* \
	&& echo '%ros ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

RUN apt-get update \
	&& apt upgrade -y \
	&& apt-get clean -q -y \
	&& apt-get autoremove -q -y \
	&& rm -rf /var/lib/apt/lists/*

RUN apt-get update \
	&& apt-get install -q -y \
		python3-vcstool \
		gnupg2 \
		git \
		git-lfs \
		vim \
		terminator \
	&& apt-get clean -q -y \
	&& apt-get autoremove -q -y \
	&& rm -rf /var/lib/apt/lists/*

RUN apt-get update \
	&& apt-get install -q -y \
		ros-noetic-object-recognition-msgs \
		ros-noetic-moveit-core \
		ros-noetic-moveit-ros-perception \
		ros-noetic-moveit-ros-planning-interface \
		ros-noetic-velocity-controllers \
		ros-noetic-twist-mux \
		python3-rostopic \
		ros-noetic-effort-controllers \
		ros-noetic-position-controllers \
		ros-noetic-hector-gazebo \
		ros-noetic-ros-controllers \
	&& apt-get clean -q -y \
	&& apt-get autoremove -q -y \
	&& rm -rf /var/lib/apt/lists/*


COPY ros_entrypoint.sh /

USER $user_name

#RUN mkdir -p $ck_src_dir
RUN true \
	&& echo "PS1='\[\e]0;\u@\h: \w\a\]${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u\[\033[00m\]@\[\033[01;31m\]\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> ~/.bashrc \
	&& echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
	&& echo "source $ck_dir/devel/setup.bash" >> ~/.bashrc

WORKDIR $user_home

ARG factory_ws_url=https://github.com/Mike2208/factory_ws
RUN true \
	&& git clone --branch joint_states_enable $factory_ws_url $ck_dir \
	&& true

WORKDIR $ck_dir

RUN true \
	&& . /opt/ros/noetic/setup.sh \
	&& export ROS_PARALLEL_JOBS=-j$(nproc --all) \
	&& catkin_make

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENV USE_GPU_FOR_SIMULATION true

ENV CATKIN_WS $ck_dir
ENV RBK_CATKIN_PATH $ck_dir
