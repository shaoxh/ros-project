import rospy
# import PySpin
# rospy.init_node('pointgrey')
# print(rospy.Time.now())
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import serial
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation
import time
import numpy as np
import binascii


portx = "/dev/ttyUSB0"
bps = 115200
timex = 50

ser = serial.Serial(portx, bps, timeout=timex)
NMEA = [0xAA, 0x55, 0x00, 0x00, 0x07, 0x00, 0x34, 0x3B, 0x00]
STOP = [0xAA, 0x55, 0x00, 0x00, 0x07, 0x00, 0xFE, 0x05, 0x01]
QuatData = [0xAA, 0x55, 0x00 ,0x00 ,0x07, 0x00, 0x36, 0x3D, 0x00]


# ser.write(STOP)
# time.sleep(4)
# ser.write(NMEA)

pub = rospy.Publisher('imu_data', Imu, queue_size=200)
rospy.init_node('imu_node', anonymous=True)
rate = rospy.Rate(200)


def talker():
    while not rospy.is_shutdown():

        if ser.in_waiting:
            str1 = ser.readline()
            str_split = str1.split(',')
            if (str_split[0] == '$PAPR'):
                # print(str1)
                roll = float(str_split[3])
                pitch = float(str_split[4])
                heading = float(str_split[5])

                header = Header(stamp=rospy.Time.now())
                header.frame_id = 'pandar'
                imu = Imu()
                imu.header = header
                quat = Rotation.from_euler('ZXY', [heading, pitch, roll], degrees=True).as_quat()
                imu.orientation.w = quat[3]
                imu.orientation.x = quat[0]
                imu.orientation.y = quat[1]
                imu.orientation.z = quat[2]

                # q=quat[3]
                quat[0]=imu.orientation.w
                quat[1]=imu.orientation.x
                quat[2]=imu.orientation.y
                quat[3]=imu.orientation.z

                # heading1=np.rad2deg(np.arctan2(2*(quat[1]*quat[2]-quat[0]*quat[3]),(quat[0]**2+quat[2]**2-quat[1]**2-quat[3]**2)))
                # pitch1=np.rad2deg(np.arcsin(2*(quat[2]*quat[3]+quat[0]*quat[1])))

                # print heading,heading1
                # print pitch,pitch1
                # imu.angular_velocity.x
                # print 0
                # str1=str(rospy.Time.now())+','+str1
                pub.publish(imu)
            # if(str1=="exit"):
            #     break
            # else:
            #     print(str1)

        # hello_str=str(rospy.Time.now())
        # print(hello_str)
        # pub.publish(hello_str)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInternalException:
        pass
