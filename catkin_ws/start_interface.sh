#!/bin/bash

#########################################
#Ident:     start_interface.sh
#Author:    Diener, Clemens
#Subject:   Start the OPC UA interface
#Language:  bash
#Purpose:   
#Version:   1.0
#Comment:   If the interface is started for the first time on a system or if anything changed on the interface,
#           the script init_docker.sh needs to be run first
#########################################

# Prepare for later inputs:
source /opt/ros/noetic/setup.bash

# If there is already a docker container with the interface running:
if [ "$(docker ps -q -f name=ros_opcua)" ]; then
    docker stop ros_opcua   # Stop the container
fi

# Start the docker container interactive with connection to the host system
docker run -it --net=host ros_opcua


