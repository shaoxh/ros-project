#!/usr/bin/env python
# coding:utf-8
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    nodeName = 'py_publisher'
    rospy.init_node(nodeName)
    topicName = 'py_topic'
    print ("ros topic: " + topicName)
    publisher = rospy.Publisher(topicName, String, queue_size=5)
    rate = rospy.Rate(1)
    i = 0
    while not rospy.is_shutdown():
        date = String()
        date.data = 'hello{}'.format(i)
        i += 1
        print ("ros msg: " + date.data)
        publisher.publish(date)
        rate.sleep()
