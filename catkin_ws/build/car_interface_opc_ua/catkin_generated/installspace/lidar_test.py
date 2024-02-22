#!/usr/bin/env python3

'''
Author: Philipp Merz
E-Mail: meph1018@hs-karlsruhe.de; philipp@merz.no

Edited by: Felix Mueller; E-Mail: felix.mueller@h-ka.de

Edited by: Clemens Diener; E-Mail: cl.diener@t-online.de; 07.12.2023
            - Added constant measurement and publisher with the center position of the targets
            - Some cleanup of the source code
            - Fixed len(filter) bug for Python3

This ROS Node is part of a Bachelor's thesis at the University of Applied Sciences Karlsruhe. Its purpose is to analyse LiDAR Data and to check if the LiDAR sensor is mounted correctly.
This version is adapted to present a coloured text based userinterface for demo use.
'''

import rospy, math, string
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import time
#from progress.spinner import MoonSpinner
from datetime import datetime

#import bcolors #for colors in the terminal
import os
os.system('color')
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    CEND      = '\33[0m'
    CBOLD     = '\33[1m'
    CITALIC   = '\33[3m'
    CURL      = '\33[4m'
    CBLINK    = '\33[5m'
    CBLINK2   = '\33[6m'
    CSELECTED = '\33[7m'
    CBLACK  = '\33[30m'
    CRED    = '\33[31m'
    CGREEN  = '\33[32m'
    CYELLOW = '\33[33m'
    CBLUE   = '\33[34m'
    CVIOLET = '\33[35m'
    CBEIGE  = '\33[36m'
    CWHITE  = '\33[37m'
    CBLACKBG  = '\33[40m'
    CREDBG    = '\33[41m'
    CGREENBG  = '\33[42m'
    CYELLOWBG = '\33[43m'
    CBLUEBG   = '\33[44m'
    CVIOLETBG = '\33[45m'
    CBEIGEBG  = '\33[46m'
    CWHITEBG  = '\33[47m'
    CGREY    = '\33[90m'
    CRED2    = '\33[91m'
    CGREEN2  = '\33[92m'
    CYELLOW2 = '\33[93m'
    CBLUE2   = '\33[94m'
    CVIOLET2 = '\33[95m'
    CBEIGE2  = '\33[96m'
    CWHITE2  = '\33[97m'
    CGREYBG    = '\33[100m'
    CREDBG2    = '\33[101m'
    CGREENBG2  = '\33[102m'
    CYELLOWBG2 = '\33[103m'
    CBLUEBG2   = '\33[104m'
    CVIOLETBG2 = '\33[105m'
    CBEIGEBG2  = '\33[106m'
    CWHITEBG2  = '\33[107m'


# -------------------------------------------------------- CONFIG AREA -------------------------------------------------------- #     
#
# Setup distances are defined here, all distances in meters unless otherwise specified
# X Axis: points in direction of forward motion for the vehicle
# Y Axis: points to the left of the vehicle
# Z Axis: positive direction up
#-- Position of target relative to mid point between the front wheels - Reference Point

xRaw1 = 2.5 #xRaw1 = 2.5
yRaw1 = 0.0 #yRaw1 = 0.0
zRaw1 = 1.55 #zRaw1 = 1.5

xRaw2 = 3.0 #xRaw2 = 3.0
yRaw2 = 0.5 #yRaw2 = 0.5
zRaw2 = 1.3 #zRaw2 = 1.3

#
#-- Sensor offsets -> used to convert to sensor reference frame
xRel = 1.3 #xRel = 1.3 
yRel = 0.0 #yRel = 0.0 
zRel = -2.0 #zRel = -2.0


#zRel = -2.0
#          
#-- Tolerances -> Acceptable divergence from position due to inaccuracies and placement errors in each direction
xTolerance = 0.1
yTolerance = 0.05
zTolerance = 0.05
#
#-- Filter area - Distance along the x axis in which points shall be considered (window size)
filterWindow = 0.5
#              
#-- Target size
target_height = 0.2
target_width = 0.2
height_tolerance = 0.01
width_tolerance = 0.01
#                   
#-- Target intensity
min_intensity = 2400
#
#-- Blockage range - How close points can be before blockage is indicated [UNIT = mm]
block_range = 250
#
#-- Acceptable number of points within block_range
block_accept = 2
#
# 
# ----------------------------------------------------- END OF CONFIG AREA ---------------------------------------------------- #

#-- Calculating coordinates and acceptable bounds
xDist1 = xRaw1 + xRel
yDist1 = yRaw1 + yRel
zDist1 = zRaw1 + zRel

xDist2 = xRaw2 + xRel
yDist2 = yRaw2 + yRel
zDist2 = zRaw2 + zRel

upper_x1 = xDist1 + xTolerance
lower_x1 = xDist1 - xTolerance
upper_y1 = yDist1 + yTolerance
lower_y1 = yDist1 - yTolerance
upper_z1 = zDist1 + zTolerance
lower_z1 = zDist1 - zTolerance

upper_x2 = xDist2 + xTolerance
lower_x2 = xDist2 - xTolerance
upper_y2 = yDist2 + yTolerance
lower_y2 = yDist2 - yTolerance
upper_z2 = zDist2 + zTolerance
lower_z2 = zDist2 - zTolerance

#-- Creating search areas
minDist1 = xDist1 - (0.5 * filterWindow)
maxDist1 = xDist1 + (0.5 * filterWindow)

minDist2 = xDist2 - (0.5 * filterWindow)
maxDist2 = xDist2 + (0.5 * filterWindow)

#-- Create log file
now = datetime.now()
log = open("lidar_test.log", "w+")
log.write("This is the log file for the lidar_test module.\n" + "Log created: " + str(now) + "\n\n\n")

#-- Callback - Most of the Data processing happens here
def callback(self, Publisher):
    pub1, pub2 = Publisher
    i = 0
    ii = 0
    count = 0
    passed = False
    passed_block = False
    passed_rel_pos = False
    passed_abs_pos = False
    gen = pc2.read_points(self, field_names=("x","y","z","intensity","range"))   # get relevant data from PC2 Object   
    listData = list(gen)                                                    # convert generator object to list  -   Not too RAM intensive   
    useful = filter(nonZero, listData)
    useful = list(filter(distNonZero, useful))                              # filters out points with range attribute of zero. otherwise corrupted data will indicate blockage.

    i = len(listData)

    #print(useful)
    #print(list(useful))

    ii = len(useful)

    dim = 5                                                                 # How many fields are read from PC2 message; 5 - x, y, z, intensity, range
    shp = (ii, dim)                                                         # shape of numpy array, needs to contain all the useful points, but not more
    xyzr = np.zeros(shp)                                                    # create numpy array for use later on, filled with zeros

    for el in useful:                                                       # converting the useful list elements to numpy array for better computation
        xyzr[count,0] = el[0]                                               # x value
        xyzr[count,1] = el[1]                                               # y value
        xyzr[count,2] = el[2]                                               # z value
        xyzr[count,3] = float(el[3])                                        # intensity value
        xyzr[count,4] = el[4]                                               # range value

        count += 1                                                          # increment counter
    
    refArray = xyzr[xyzr[:,3] > min_intensity]                              # filters all the datapoints with a intensity higher than the threshold
    Target1 = refArray[refArray[:,0] > minDist1]                            # filters points based on distance
    Target1 = Target1[Target1[:,0] < maxDist1]

    Target2 = refArray[refArray[:,0] > minDist2]
    Target2 = Target2[Target2[:,0] < maxDist2]

    #-- check first target:
    print("[INFO]:  " + "Calculate target 1:")
    log.write("\n\nCalculate target 1:\n\n")
    msgTarget1 = CalculateCenter(Target1)

    #-- check second target
    print("[INFO]:  " + "Calculate target 2:")
    log.write("\n\nCalculate target 2:\n\n")
    msgTarget2 = CalculateCenter(Target2)

    # TODO: Check if blocked points helpfull
    passed_block = checkBlock(xyzr)
    
    # Publish the center of both targets:
    pub1.publish(msgTarget1)
    pub2.publish(msgTarget2)

    print("[INFO]:  " + "--------DONE--------")
    #log.close()                                                             # close log file

    rospy.sleep(1)      # Sleep, change duration later 

#####################################################################################
#------------------------------------ FUNCTIONS ------------------------------------#
#####################################################################################

def listener():
    rospy.init_node('Lidar_Test', anonymous=True)                             # initializes node
    #rospy.loginfo("Node initialized")
    ##rospy.loginfo("Tester initialized")
    print("[INFO]:  " + "Tester initialized")

    pub_1 = rospy.Publisher('Target_1', String, queue_size=10)
    pub_2 = rospy.Publisher('Target_2', String, queue_size=10)

    rospy.Subscriber("/os_cloud_node/points", PointCloud2, callback, [pub_1, pub_2])        # subscribes to PC2 message, executes calback function when PC2 received
    rospy.spin()

def nonZero(el):
    return el != (0.0, 0.0, 0.0, 0, 0)

def distNonZero(el):                                                        # Filter function to remove all points with range == 0
    if el[4]!=0:
        return el 

def getHeight(target_array, numRows):
    high_z_Value = target_array[:,2].argsort()                              # sorts potential target points by their y value to get highest and lowest datapoint
    low_index = high_z_Value[0]
    high_index = high_z_Value[(numRows-1)]
    high = target_array[high_index,2]
    low = target_array[low_index,2]
    height = high - low                                                     # gets height of target by subtracting 

    height_ok = False
    counter = 0

    while height_ok == False:                                               # check that height is plausible, if not, check the next height down
        if height > (target_height + height_tolerance):
            counter += 1
            low_index = high_z_Value[counter]
            high_index = high_z_Value[(numRows-1-counter)]
            high = target_array[high_index,2]
            low = target_array[low_index,2]
            height = high - low

            if(counter >= ((numRows/2) - 4)):
                ##rospy.loginfo(bcolors.FAIL + "ERROR: could not determine height. Target not found." + bcolors.ENDC)
                print("[INFO]:  " + bcolors.FAIL + "ERROR: could not determine height. Target not found." + bcolors.ENDC)
                log.write("\n\nERROR: could not determine target height. No matching points found.\n\n")

                return 0, 0, 0, 0, 0

        else:
            ##rospy.loginfo(bcolors.OKGREEN + "Target height ok" + bcolors.ENDC)
            print("[INFO]:  " + bcolors.OKGREEN + "Target height ok" + bcolors.ENDC)
            height_ok = True
    
    return height, high, low, high_index, low_index


def getWidth(target_array, numRows):
    high_y_Value = target_array[:,1].argsort()                              # sorts potential target points by their y value to get highest and lowest datapoint -> only provides indices
    right_index = high_y_Value[0]
    left_index = high_y_Value[(numRows-1)]
    right = target_array[right_index,1]
    left = target_array[left_index,1]
    width = left - right                                                    # gets width of target by subtracting 
    
    width_ok = False
    counter = 0
    while width_ok == False:
        if (width > (target_width + width_tolerance)):
            counter += 1
            right_index = high_y_Value[counter]
            left_index = high_y_Value[(numRows-1-counter)]
            right = target_array[right_index,1]
            left = target_array[left_index,1]
            width = left - right

            if(counter >= ((numRows/2) - 4)):                               # fail safe.
                ##rospy.loginfo(bcolors.FAIL + "ERROR: could not determine width. Target not found." + bcolors.ENDC)
                print("[INFO]:  " + bcolors.FAIL + "ERROR: could not determine width. Target not found." + bcolors.ENDC)
                log.write("\n\nERROR: could not determine target width. No matching points found.\n\n")
                
                return 0, 0, 0, 0, 0
        
        else:
            ##rospy.loginfo(bcolors.OKGREEN + "Target width ok" + bcolors.ENDC)
            print("[INFO]:  " + bcolors.OKGREEN + "Target width ok" + bcolors.ENDC)
            width_ok = True
    
    return width, right, left, right_index, left_index


def getDistance(target_array, high_index, low_index, left_index, right_index):  # creates average x value for known target points
    x1 = target_array[high_index,0]
    x2 = target_array[low_index,0]
    x3 = target_array[right_index,0]
    x4 = target_array[left_index,0]
    dist = (x1+x2+x3+x4)/4

    return dist


def getCenter(height, low, width, left, center_x):                              # Finds center of Target based on mid point between edges
    center_y = left - 0.5*width
    center_z = low + 0.5*height

    return center_x, center_y, center_z                                         # returns x/y/z coordinates (straight pass through for x)


def checkBlock(full_array):                                                     # checks if there is a blockage in the sensor'S FOV
    bad_dist = full_array[full_array[:,4] <= block_range]                       # gets all points with "range" <= block_range (default 250mm)
    shp = bad_dist.shape

    log.write("\n\n" + "!!---------- RESULTS OF BLOCKAGE TEST ----------!!\n\n")
    log.write("A total of " + str(shp[0]) + " points were found to be blocked (x/y/z/intensity/range):\n\n")
    log.write(np.array2string(bad_dist))                                        # writes abridged list of points to log file
    log.write("\n\nFor full list, consult lidar_blocked.txt.\n\n")
    np.savetxt("lidar_blocked.txt", bad_dist)                                   # saves all points found to separate text file

    if shp[0] < block_accept:
        ##rospy.loginfo(bcolors.OKGREEN + "PASS: No blockage detected." + bcolors.ENDC)
        print("[INFO]:  " + bcolors.OKGREEN + "PASS: No blockage detected." + bcolors.ENDC)
        log.write("\nPASS: No blockage detected.\n\n")
        return True
    else:
        ##rospy.loginfo(bcolors.FAIL + "FAIL: Blockage detected!" + bcolors.ENDC)
        print("[INFO]:  " + bcolors.FAIL + "FAIL: Blockage detected!" + bcolors.ENDC)
        log.write("\nFAIL: Blockage detected!\n\n")
        return False

def CalculateCenter(Target):
    """
    """
    #-- check target
    TargetRows = Target.shape[0]
    if TargetRows < 2:                                                            # if no potential targets were found, so no high reflectivity points within search zone
        ##rospy.loginfo(bcolors.FAIL + "ERROR: No targets found!" + bcolors.ENDC)
        print("[INFO]:  " + bcolors.FAIL + "ERROR: No targets found!" + bcolors.ENDC)
        log.write("\n\n!!-- NO TARGETS FOUND --!!\n\nCould not find any potential targets for target.\n\n")
        
        # Create message for ROS:
        msgTarget = "NOT FOUND"
    else:
        # get target [height, high, low, high_index, low_index]
        TargetHeight, TargetHigh, TargetLow, TargetHighIndex, TargetLowIndex = getHeight(Target, TargetRows)              
        # get target [width, right, left, right_index, left_index]
        TargetWidth, TargetRight, TargetLeft, TargetRightIndex, TargetLeftIndex = getWidth(Target, TargetRows)    
        TargetXCenter = getDistance(Target,TargetHighIndex,TargetLowIndex,TargetLeftIndex,TargetRightIndex)   # get x coordinate for center
        TargetCenter = getCenter(TargetHeight, TargetLow, TargetWidth, TargetLeft, TargetXCenter)             # get y and z coordinates for center
        log.write("\n\nCenter of target: "+ str(TargetCenter[0]) + "/" + str(TargetCenter[1]) + "/" + str(TargetCenter[2]))
        # TODO: Check the relative offset
        print("[INFO]:  " + "Relative Center Target: "+ str(round(TargetCenter[0],3)-xRel) + "m/" + str(round(TargetCenter[1],3)-yRel) + "m/" + str(round(TargetCenter[2],3)-zRel)+"m")
        print("[INFO]:  " + "Center Target: "+ str(round(TargetCenter[0],3)) + "m/" + str(round(TargetCenter[1],3)) + "m/" + str(round(TargetCenter[2],3))+"m")

        msgTarget = str(TargetCenter[0]) + "/" + str(TargetCenter[1]) + "/" + str(TargetCenter[2])

    return msgTarget

#------------------------------ Trigger listener -----------------------------# 

if __name__=='__main__':
    listener()
