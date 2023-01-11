#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import UInt8MultiArray
import math

DEFAULT_LINEAR_VELOCITY = 4
DEFAULT_ANGULAR_VELOCITY = 0.8
THR = 0.02 # Threshold for obstacle detection in meters

prox = {}
od_sensors = [0, 1, 6, 7]

# Publishers
cmd_vel_pub = rospy.Publisher('mobile_base/cmd_vel', Twist, queue_size=1)
rgb_leds_pub = rospy.Publisher('mobile_base/rgb_leds', UInt8MultiArray, queue_size=1)
cmd_leds_pub = rospy.Publisher('mobile_base/cmd_leds', UInt8MultiArray, queue_size=1)

def navigate():
    rospy.init_node('navigate', anonymous=True)
    rate = rospy.Rate(30) # 30hz
    state = 0

    # Wait for the device to be ready
    rospy.sleep(2)


    while not rospy.is_shutdown():

        getProxReadings()

        # Wait for the proximity readings to be available
        if not bool(prox):
            continue

        setLEDIndication("pathClear")

        if state == 0:
            traceBoundary()

            if obstacleDetected():
                setLEDIndication("obstacleDetected")
                state = 1
                rospy.loginfo("Obstacle detected. Turning...")
        elif state == 1:
            turn()
            if pathClear():
                setLEDIndication("pathClear")
                rospy.loginfo("Path clear: Moving forward...")
                state = 0
        else:
            rospy.loginfo("Invalid state: %s", state)


        rate.sleep()


# ************************ HELPERS ************************ #

# Check if obstacle detected
def obstacleDetected():
    obstacleDetected_ = False
    for sensor, value in prox.items():
        if value < THR and sensor in od_sensors:
            obstacleDetected_ = True
            break
    return obstacleDetected_

# Check if path is clear
def pathClear():
    pathClear_ = True
    for sensor, value in prox.items():
        if value < 1.5 * THR and sensor in od_sensors:
            pathClear_ = False
            break
    
    return pathClear_

# Get the proximity readings and save it to the prox dictionary
def getProxReadings():
    rospy.Subscriber("proximity0", Range, proxCallback)
    rospy.Subscriber("proximity1", Range, proxCallback)
    rospy.Subscriber("proximity2", Range, proxCallback)
    rospy.Subscriber("proximity3", Range, proxCallback)
    rospy.Subscriber("proximity4", Range, proxCallback)
    rospy.Subscriber("proximity5", Range, proxCallback)
    rospy.Subscriber("proximity6", Range, proxCallback)
    rospy.Subscriber("proximity7", Range, proxCallback)


def proxCallback(data):
    saveSensorData(data)

# Save the sensor data to the dictionary
def saveSensorData(data):
    sensor_no = int(data.header.frame_id[-1])
    prox[sensor_no] = data.range

# Trace the boundary of the environment
def traceBoundary():
    dist = prox[2]  # Sensor 2 is right side sensor
    # angular_vel = -math.tan(dist * math.pi/0.05)
    # angular_vel = 10 if angular_vel > 10 else -10 if angular_vel < -10 else angular_vel
    angular_vel = 0.7 if dist < 0.01 else .5 if dist < 0.02 else .2 if dist < 0.03 else -0.2 if dist < 0.04 else -0.5 if dist < 0.05 else -0.7
    rospy.loginfo('DEV: angular vel: %s, dist: %s', angular_vel, dist)
    publishTwistMessage(DEFAULT_LINEAR_VELOCITY,angular_vel)

# Move forward
def moveForward(linear_vel = DEFAULT_LINEAR_VELOCITY):
    publishTwistMessage(linear_vel)

# Move in a circle
def moveInCircle(linear_vel = 10, angular_vel = DEFAULT_ANGULAR_VELOCITY):
    # rospy.loginfo("Moving in a circle")
    publishTwistMessage(linear_vel, angular_vel)

# Turn
def turn(angular_vel = DEFAULT_ANGULAR_VELOCITY):
    direction = 1 if prox[0] + prox[1] < prox[6] + prox[7] else -1
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

# Set the LED indication
# Few LEDs weren't turning on in the hardware and the mapping seemed disoriented.
# This function maps the LEDs correctly to an extent. (The Red of RGB LED 1 doesn't seem to work - Insted green turns on)
def setLEDIndication(type):
    if type == "pathClear":
        rgb_leds = UInt8MultiArray()
        rgb_leds.data = [100,0,0, 0,0,0, 0,0,0, 100,0,0]
        rgb_leds_pub.publish(rgb_leds)

        cmd_leds = UInt8MultiArray()
        cmd_leds.data = [0,0,0,0,0,0]
        cmd_leds_pub.publish(cmd_leds)

    elif type == "obstacleDetected":
        rgb_leds = UInt8MultiArray()
        rgb_leds.data = [0,0,0, 0,0,0, 0,0,100, 0,0,0]
        rgb_leds_pub.publish(rgb_leds)

        cmd_leds = UInt8MultiArray()
        cmd_leds.data = [1,0,0,0,0,0]
        cmd_leds_pub.publish(cmd_leds)

# ************************ MAIN ************************ #

if __name__ == '__main__':
    try:
        navigate()
    except rospy.ROSInterruptException:
        pass