#########################################
#Ident:     Dockerfile
#Author:    Diener, Clemens
#Subject:   Construct the docker-container
#Language:  Docker
#Version:   1.0
#Comment:   The system should be connected to the internet for the first use
#########################################

# Load the ROS-Noetic image:
FROM ros:noetic 
LABEL Name=source Version=0.0.1

# Prepare for more commands:
SHELL ["/bin/bash", "-c" ]

# Install all necesarry packages:
RUN apt-get update
RUN apt-get -y install python3-pip
# Specific version are used for the python packages, so no version problems appeare with newer versions:
RUN pip3 install pytz==2019.3
RUN pip3 install aiofiles==23.2.1
RUN pip3 install pyOpenSSL==23.2.0
#RUN pip install asyncua==1.0.4 # Own version of asyncua is used 

# Copy the whole source code to the docker container:
COPY ./src /app/src

# Set the ROS enviroment:
RUN source /opt/ros/noetic/setup.bash &&\cd app &&\ 
   catkin_make
# Write the source command to the bshrc file:
RUN echo "source /app/devel/setup.bash" >> ~/.bashrc

# Start the ROS-Node (ROS-Core need to be started first):
CMD source /app/devel/setup.bash; cd /app;  rosrun car_interface_opc_ua opc_ua_server.py

