#!/usr/bin/env python3

'''
Author: Philipp Merz
E-Mail: meph1018@hs-karlsruhe.de; philipp@merz.no

This ROS Node is part of a Bachelor's thesis at the University of Applied Sciences Karlsruhe. Its purpose is to analyse LiDAR Data and to check if the LiDAR sensor is mounted correctly.

Edited by Felix Mueller
publish info via ROS Topics

WORK IN PROGRESS

'''

import rospy, math, string
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from datetime import datetime



# -------------------------------------------------------- CONFIG AREA -------------------------------------------------------- #     
#
# Setup distances are defined here, all distances in meters unless otherwise specified
# X Axis: points in direction of forward motion for the vehicle
# Y Axis: points to the left of the vehicle
# Z Axis: positive direction up
#-- Position of target relative to mid point between the front wheels - Reference Point
xRaw1 = 2.5
yRaw1 = 0.0
zRaw1 = 1.2

xRaw2 = 3.0
yRaw2 = 0.5
zRaw2 = 1.5
#
#-- Sensor offsets -> used to convert to sensor reference frame
xRel = 1.3 
yRel = 0.0 
zRel = -2.0
#          
#-- Tolerances -> Acceptable divergence from position due to inaccuracies and placement errors in each direction
xTolerance = 0.1
yTolerance = 0.05
zTolerance = 0.05
#
#-- Filter area - Distance along the x axis in which points shall be considered (window size)
filterWindow = 2.0
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
try:
    log = open("lidar_test.log", "w+")
    log.write("This is the log file for the lidar_test module.\n" + "Log created: " + str(now) + "\n\n\n")
except:
    print("Could not create log file!")


#-- Callback - Most of the Data processing happens here

def callback(self):
    i = 0
    ii = 0
    count = 0
    rospy.loginfo("launched successfully")


    passed = False
    passed_block = False
    passed_rel_pos = False
    passed_abs_pos = False
    gen = pc2.read_points(self, field_names=("x","y","z","intensity","range"))   # get relevant data from PC2 Object   
    listData = list(gen)                                                    # convert generator object to list  -   Not too RAM intensive   
    useful = filter(nonZero, listData)
    useful = filter(distNonZero, useful)                                    # filters out points with range attribute of zero. otherwise corrupted data will indicate blockage.

    i = len(listData)
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
    potential_Target = refArray[refArray[:,0] > minDist1]                   # filters points based on distance
    potential_Target = potential_Target[potential_Target[:,0] < maxDist1]

    pot_other_Target = refArray[refArray[:,0] > minDist2]
    pot_other_Target = pot_other_Target[pot_other_Target[:,0] < maxDist2]

    rows_columns = potential_Target.shape
    rows = rows_columns[0]
    if rows < 2:                                                            # if no potential targets were found, so no high reflectivity points within search zone
        rospy.loginfo("ERROR: No targets found!")
        log.write("\n\n!!-- NO TARGETS FOUND --!!\n\nCould not find any potential targets for target 1.\n\n")
        rospy.spin()
        #while not rospy.is_shutdown():
            #rospy.sleep(1)  # sleep for one second

    if pot_other_Target.shape[0] < 2:                                       # if no potential targets were found, so no high reflectivity points within search zone
        rospy.loginfo("ERROR: No targets found!")
        log.write("\n\n!!-- NO TARGETS FOUND --!!\n\nCould not find any potential targets for target 2.\n\n")
        rospy.spin() 
        #while not rospy.is_shutdown():
            #rospy.sleep(1)  # sleep for one second

    #-- check first target
    height_high_low = getHeight(potential_Target, rows)                     # get target [height, high, low, high_index, low_index]
    height = height_high_low[0]                                             # tuple to single variables for increased readablity
    high = height_high_low[1]
    low = height_high_low[2]
    high_index = height_high_low[3]
    low_index = height_high_low[4]

    width_right_left = getWidth(potential_Target, rows)                     # get target [width, right, left, right_index, left_index]
    width = width_right_left[0]
    right = width_right_left[1]
    left = width_right_left[2]
    right_index = width_right_left[3]
    left_index = width_right_left[4]
    
    x_center1 = getDistance(potential_Target,high_index,low_index,left_index,right_index)   # get x coordinate for center
    center1 = getCenter(height, low, width, left, x_center1)               # get y and z coordinates for center

    log.write("\n\nCenter of target 1 found: "+ str(center1[0]) + "/" + str(center1[1]) + "/" + str(center1[2]))
    np.savetxt('sample_ref_array.txt', potential_Target)

    #-- check second target
    checked_height = getHeight(pot_other_Target, pot_other_Target.shape[0])
    checked_width = getWidth(pot_other_Target, pot_other_Target.shape[0])
    x_center2 = getDistance(pot_other_Target, checked_height[3], checked_height[4], checked_width[4], checked_width[3])
    center2 = getCenter(checked_height[0], checked_height[2], checked_width[0], checked_width[2], x_center2)
    log.write("\n\nCenter of target 2 found: "+ str(center2[0]) + "/" + str(center2[1]) + "/" + str(center2[2]) + "\n\n")

    passed_block = checkBlock(xyzr)
    passed_abs_pos = comparePosition(center1, center2)
    passed_rel_pos = checkRelPos(center1, center2)

    if passed_block and passed_abs_pos and passed_rel_pos:
        rospy.loginfo("")
        rospy.loginfo("VERDICT: Sensor PASSED the test!")
        rospy.loginfo("")
        log.write("\n\n\n!!-- VERDICT: SENSOR PASSED ALL TESTS! --!!")

    else:
        rospy.loginfo("")
        rospy.loginfo("VERDICT: Sensor FAILED the test!")
        rospy.loginfo("For more info consult log file.")
        rospy.loginfo("")
        log.write("\n\n\n!!-- VERDICT: SENSOR FAILED THE TEST --!!")

    rospy.loginfo("--------DONE--------")
    log.close()                                                             # close log file
    rospy.spin()    # stops node from exiting and executing for the next PC2 message
    #while not rospy.is_shutdown():
            #rospy.sleep(1)  # sleep for one second






#####################################################################################
#------------------------------------ FUNCTIONS ------------------------------------#
#####################################################################################

def listener():
    rospy.init_node('finder', anonymous=True)                             # initializes node
    rospy.loginfo("Node initialized")

    rospy.Subscriber("/os_cloud_node/points", PointCloud2, callback)        # subscribes to PC2 message, executes calback function when PC2 received
    pub_1 = rospy.Publisher('Target_1', String, queue_size=10)
    pub_2 = rospy.Publisher('Target_2', String, queue_size=10)
    rospy.spin()

#def talker():
    #pub_1 = rospy.Publisher('Target_1', String, queue_size=10)
    #pub_2 = rospy.Publisher('Target_2', String, queue_size=10)
    ##rospy.init_node('OBD_lidar', anonymous=False)
    #rate = rospy.Rate(10) #10hz
    #rospy.sleep(10) #sleep for one second

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
                rospy.loginfo("ERROR: could not determine height. Target not found.")
                log.write("\n\nERROR: could not determine target height. No matching points found.\n\n")
                rospy.spin()
                #while not rospy.is_shutdown():
                    #rospy.sleep(1)  # sleep for one second

        else:
            rospy.loginfo("Height ok")
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
                rospy.loginfo("ERROR: could not determine width. Target not found.")
                log.write("\n\nERROR: could not determine target width. No matching points found.\n\n")
                rospy.spin()
                #while not rospy.is_shutdown():
                    #rospy.sleep(1)  # sleep for one second
        
        else:
            rospy.loginfo("Width ok")
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
        rospy.loginfo("PASS: No blockage detected.")
        log.write("\nPASS: No blockage detected.\n\n")
        return True
    else:
        rospy.loginfo("FAIL: Blockage detected!")
        log.write("\nFAIL: Blockage detected!\n\n")
        return False


def comparePosition(center1, center2):                                          # checks measured position of targets against known position
    log.write("\n\n!!---------- CHECK POSITION OF TARGET ----------!!\n\n")
    x_ok = (lower_x1 <= center1[0] <= upper_x1) and (lower_x2 <= center2[0] <= upper_x2)
    y_ok = (lower_y1 <= center1[1] <= upper_y1) and (lower_y2 <= center2[1] <= upper_y2)
    z_ok = (lower_z1 <= center1[2] <= upper_z1) and (lower_z2 <= center2[2] <= upper_z2)

    if x_ok and y_ok and z_ok:
        rospy.loginfo("PASS: Target positions match.")
        log.write("PASS: Center values within bounds\n\n")
        log.write("\t calculated center of Target 1: " + str(center1) +  "\n")
        log.write("\t calculated center of Target 2: " + str(center2) +  "\n")
        while (Time.sleep(1)):
            message = str(center1)
            pub_1.publish(message)
        log.write("\t center of target 1 should be: " + str(xDist1) + " / " + str(yDist1) + " / " + str(zDist1) + "\n")
        log.write("\t center of target 2 should be: " + str(xDist2) + " / " + str(yDist2) + " / " + str(zDist2) + "\n\n")
        return True
    else:
        rospy.loginfo("FAIL: Target positions do not match! -> Check log file")
        log.write("FAIL: center values out of bounds!\n\n")
        log.write("\t calculated center of Target 1: " + str(center1) +  "\n")
        log.write("\t calculated center of Target 2: " + str(center2) +  "\n")
        log.write("\t center of target 1 should be: " + str(xDist1) + " / " + str(yDist1) + " / " + str(zDist1) + "\n")
        log.write("\t center of target 2 should be: " + str(xDist2) + " / " + str(yDist2) + " / " + str(zDist2) + "\n\n")
        
        if (center1[0] < lower_x1) and (center2[0] < lower_x2):             # diagnose why sensor might have failed.
            log.write("Targets are perceived too close. The sensor might be pushed forward.\n")
        if (center1[0] > upper_x1) and (center2[0] > upper_x2):
            log.write("Targets are perceived too far away. The sensor might be pushed back.\n")

        if (center1[1] < lower_y1) and (center2[1] < lower_y2):
            log.write("Targets are perceived too far to the right. Sensor might be translated along y Axis.\n")
        if (center1[1] > upper_y1) and (center2[1] > upper_y2):
            log.write("Targets are perceived too far to the left. Sensor might be translated along y Axis.\n")

        if (center1[2] < lower_z1) and (center2[2] < lower_z2):
            log.write("Targets are perceived too far down. Sensor might be translated along z Axis.\n")
        if (center1[2] > upper_z1) and (center2[2] > upper_z2):
            log.write("Targets are perceived too far up. Sensor might be translated along z Axis.\n")

        return False



def checkRelPos(center1, center2):              # checks relative position of targets
    rel = np.array([0.0, 0.0, 0.0])
    rel[0] = xDist1 - xDist2
    rel[1] = yDist1 - yDist2
    rel[2] = zDist1 - zDist2

    rel_real = np.array([0.0, 0.0, 0.0])        # relative distances between targets as measured by the sensor
    rel_real[0] = center1[0] - center2[0]
    rel_real[1] = center1[1] - center2[1]
    rel_real[2] = center1[2] - center2[2]

    diff = np.subtract(rel, rel_real)           # creates array with difference between measurement and reality

    log.write("\n\n!!---------- CHECK RELATIVE POSITIONS OF TARGETS ----------!!\n\n")
    log.write("The relative position differs as follows (x/y/z):\n")
    log.write("\t" + str(diff) + "\n\n")

    if (abs(diff[0]) <= xTolerance) and (abs(diff[1]) <= yTolerance) and (abs(diff[2]) <= zTolerance):
        log.write("PASS: Relative position of the targets matches!\n\n")
        rospy.loginfo("PASS: Relative Position")
        return True
    
    else:
        log.write("FAIL: Relative position of targets does not match.\n")
        if (abs(diff[0]) <= xTolerance) and (abs(diff[1]) > yTolerance) and (abs(diff[2]) > zTolerance):
            log.write("\n  Sensor may be twisted around X - Axis")
        
        elif (abs(diff[0]) > xTolerance) and (abs(diff[1]) <= yTolerance) and (abs(diff[2]) > zTolerance):
            log.write("\n  Sensor may be twisted around Y - Axis")

        elif (abs(diff[0]) > xTolerance) and (abs(diff[1]) > yTolerance) and (abs(diff[2]) <= zTolerance):
            log.write("\n  Sensor may be twisted around Z - Axis")

        else:
            log.write("\n  Sensor may be twisted around several Axes")

        
        log.write("\n\n  If the sensor is found to be mounted correctly, it may need to  be calibrated.\n  Please also check if targets are placed correctly.\n")
        rospy.loginfo("FAIL: Relative position - check log file!")
        return False


    

#------------------------------ Trigger listener -----------------------------# 

if __name__=='__main__':
    listener()
    #talker()
