# coding:utf-8

import roslib
import rosbag
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import os
import sys
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import glob

# path = '/media/tongji-survey/data/data0929/image250//'  # 存放图片的位置
# bag_path='/media/tongji-survey/data/data0929/2020-09-29-11-07-33.bag'

bag_path=glob.glob('*.bag')[0]

folder=os.path.split(bag_path)[0]
image0=os.path.join(folder,'image_250')
image1=os.path.join(folder,'image_251')
image2=os.path.join(folder,'image_252')
image3=os.path.join(folder,'image_253')
pcd=os.path.join(folder,'pcd_pandar')

for p in [image0,image1,image2,image3,pcd]:
    if(not os.path.exists(p)):
        os.mkdir(p)

# sys.exit()

class ImageCreator():

    def __init__(self):
        self.bridge = CvBridge()
        with rosbag.Bag(bag_path, 'r') as bag:  # 要读取的bag文件；
            for topic, msg, t in bag.read_messages():
                try:
                    # if topic == "/image/19279250":  # 图像的topic；
                    #     try:
                    #         cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    #     except CvBridgeError as e:
                    #         print e
                    #     timestr = "%.6f" % msg.header.stamp.to_sec()
                    #     # %.6f表示小数点后带有6位，可根据精确度需要修改；
                    #     image_name = timestr + ".png"  # 图像命名：时间戳.jpg
                    #     image_name=os.path.join(image0, image_name)
                    #     cv2.imwrite(image_name, cv_image)  # 保存；
                    #     print image_name
                    # elif topic == "/image/19279251":  # 图像的topic；
                    #     try:
                    #         cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    #     except CvBridgeError as e:
                    #         print e
                    #     timestr = "%.6f" % msg.header.stamp.to_sec()
                    #     # %.6f表示小数点后带有6位，可根据精确度需要修改；
                    #     image_name = timestr + ".png"  # 图像命名：时间戳.jpg
                    #     image_name=os.path.join(image1, image_name)
                    #     cv2.imwrite(image_name, cv_image)  # 保存；
                    #     print image_name
                    if topic == "/image/19279252":  # 图像的topic；
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                        except CvBridgeError as e:
                            print e
                        timestr = "%.6f" % msg.header.stamp.to_sec()
                        # %.6f表示小数点后带有6位，可根据精确度需要修改；
                        image_name = timestr + ".png"  # 图像命名：时间戳.jpg
                        image_name=os.path.join(image2, image_name)
                        cv2.imwrite(image_name, cv_image)  # 保存；
                        print image_name
                    # elif topic == "/image/19279253":  # 图像的topic；
                    #     try:
                    #         cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    #     except CvBridgeError as e:
                    #         print e
                    #     timestr = "%.6f" % msg.header.stamp.to_sec()
                    #     # %.6f表示小数点后带有6位，可根据精确度需要修改；
                    #     image_name = timestr + ".png"  # 图像命名：时间戳.jpg
                    #     image_name=os.path.join(image3, image_name)
                    #     cv2.imwrite(image_name, cv_image)  # 保存；
                    #     print image_name
                    elif topic == "/pandar_points":  # laser的topic；
                        timestr = "%.6f" % msg.header.stamp.to_sec()
                        laser_data_name = timestr + ".txt"
                        laser_data_path = os.path.join(pcd, laser_data_name)
                        lidar = pc2.read_points(msg)
                        points = np.array(list(lidar))
                        # points=points[:,:3]
                        np.savetxt(laser_data_path,points)
                        print laser_data_path
                except:
                    continue


if __name__ == '__main__':

    # rospy.init_node(PKG)

    try:
        image_creator = ImageCreator()
    except rospy.ROSInterruptException:
        pass
