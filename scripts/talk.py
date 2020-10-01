#!/usr/bin/env python

import rospy
from std_msgs.msg import String 

rospy.init_node('topic_publisher')
pub = rospy.Publisher('phrases', String, queue_size=10)

rate = rospy.Rate(2)
msg_str = String()
i = 0

while not rospy.is_shutdown():
    a = "Ola, numdo num:" + str(i)
    msg_str = a
    i = i+1
    pub.publish(msg_str)
    rate.sleep()