#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic


import rospy
from geometry_msgs.msg import Twist

DEFAULT_LINEAR_VELOCITY = 5
DEFAULT_ANGULAR_VELOCITY = 0.8

cmd_vel_pub = rospy.Publisher('mobile_base/cmd_vel', Twist, queue_size=1)


def navigate():
    rospy.init_node('navigate', anonymous=True)
    rate = rospy.Rate(30) # 30hz

    while not rospy.is_shutdown():
        moveInCircle()
        rate.sleep()


# ************************ HELPERS ************************ #


# Move in a circle
def moveInCircle(linear_vel = 10, angular_vel = DEFAULT_ANGULAR_VELOCITY):
    publishTwistMessage(linear_vel, angular_vel)

# Stop the movement
def stop():
    publishTwistMessage(0, 0)

# Publish the twist message
def publishTwistMessage(linear_vel, angular_vel):
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