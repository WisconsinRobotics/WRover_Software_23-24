FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt install -y python3-pip
RUN pip install virtualenv

WORKDIR /WRover_Software
COPY . .

ENTRYPOINT [ "/ros_entrypoint.sh"]
