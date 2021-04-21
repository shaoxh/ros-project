#!/usr/bin/env python
# coding:utf-8
import rospy
from std_msgs.msg import String
import threading
import MySQLdb as mysql


def write_db(name):
    con = mysql.connect(host='localhost', user='root', passwd='passwd', db='test')
    cursor = con.cursor()
    sql = 'insert into test (name) value (\"{}\")'.format(name.data)
    print (sql)
    cursor.execute(sql)
    con.commit()
    cursor.execute('select * from test;')
    res = cursor.fetchall()
    print ('______________________')
    print (res)
    con.close()

def callBack(data):
    write_db(data)

if __name__ == '__main__':
    nodeName = 'py_subscriper'
    rospy.init_node(nodeName)
    topicName = 'py_topic'
    subscriber = rospy.Subscriber(topicName, String, callBack)
    print (subscriber)
    rospy.spin()
