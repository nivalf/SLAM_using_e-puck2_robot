#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

DEBUG_MODE = False

DEFAULT_LINEAR_VELOCITY = 5
DEFAULT_ANGULAR_VELOCITY = 0.8
THR = 0.02 # Threshold for obstacle detection in meters

prox_values = {}
cmd_vel_pub = rospy.Publisher('mobile_base/cmd_vel', Twist, queue_size=1)


def navigate():
    rospy.init_node('navigate', anonymous=True)
    rate = rospy.Rate(30) # 30hz
    state = 0

    # Wait for the device to be ready
    rospy.sleep(1)

    # moveInCircle()
    while not rospy.is_shutdown():

        getProxReadings()

        if DEBUG_MODE:
            rospy.loginfo("Prox values: %s", prox_values)
            continue

        if state == 0:
            if pathClear():
                rospy.loginfo("Moving forward...")
                moveForward()
                state = 1
        elif state == 1:
            if obstacleDetected():
                rospy.loginfo("Obstacle detected. Truning...")
                turn()
                state = 0
        else:
            rospy.loginfo("Invalid state: %s", state)


        rate.sleep()


# ************************ HELPERS ************************ #

def obstacleDetected():
    obstacleDetected_ = False
    for value in prox_values.values():
        if value < THR:
            obstacleDetected_ = True
            break
    return obstacleDetected_

def pathClear():
    pathClear_ = True
    for value in prox_values.values():
        if value <  2.4* THR:
            pathClear_ = False
            break
    return pathClear_

# Get the proximity readings and save it to the prox_values dictionary
def getProxReadings():
    rospy.Subscriber("proximity0", Range, proxCallback)
    rospy.Subscriber("proximity1", Range, proxCallback)
    rospy.Subscriber("proximity6", Range, proxCallback)
    rospy.Subscriber("proximity7", Range, proxCallback)


def proxCallback(data):
    saveSensorData(data)

# Save the sensor data to the dictionary
def saveSensorData(data):
    sensor_no = int(data.header.frame_id[-1])
    prox_values[sensor_no] = data.range

# Move forward
def moveForward(linear_vel = DEFAULT_LINEAR_VELOCITY):
    publishTwistMessage(linear_vel)

# Move in a circle
def moveInCircle(linear_vel = 10, angular_vel = DEFAULT_ANGULAR_VELOCITY):
    # rospy.loginfo("Moving in a circle")
    publishTwistMessage(linear_vel, angular_vel)

# Turn
def turn(angular_vel = DEFAULT_ANGULAR_VELOCITY):
    direction = 1 if prox_values[0] + prox_values[1] < prox_values[6] + prox_values[7] else -1
    publishTwistMessage(0, direction * angular_vel)

# Stop the movement
def stop():
    rospy.loginfo("Stopping")
    publishTwistMessage(0, 0)

# Publish the twist message
def publishTwistMessage(linear_vel=0, angular_vel=0):
    twist = Twist()
    twist.linear.x = linear_vel
    twist.angular.z = angular_vel
    cmd_vel_pub.publish(twist)

# ************************ MAIN ************************ #

if __name__ == '__main__':
    try:
        navigate()
    except rospy.ROSInterruptException:
        pass