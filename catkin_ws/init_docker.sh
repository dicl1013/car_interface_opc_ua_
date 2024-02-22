#!/bin/bash

#########################################
#Ident:     init_docker.sh
#Author:    Diener, Clemens
#Subject:   Initialise the docker container.
#Language:  bash
#Purpose:   
#Version:   1.0
#Comment:   The system needs to be connected to the internet
#########################################

# Prepare for later inputs:
source /opt/ros/noetic/setup.bash

# Build the docker container:
docker build -f Dockerfile -t ros_opcua .
