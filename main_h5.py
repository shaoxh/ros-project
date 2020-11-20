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
import h5py
import tqdm
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
import datetime

# path = '/media/tongji-survey/data/data0929/image250//'  # 存放图片的位置
# bag_path='/media/tongji-survey/data/data0929/2020-09-29-11-07-33.bag'

bag_path=glob.glob('*.bag')[0]
print bag_path
folder=os.path.split(bag_path)[0]
h5=bag_path[:-4]+'.h5'
print h5
# image0=os.path.join(folder,'image_250')
# image1=os.path.join(folder,'image_251')
# image2=os.path.join(folder,'image_252')
# image3=os.path.join(folder,'image_253')
# pcd=os.path.join(folder,'pcd_pandar')
h5 = h5py.File(h5, 'w')
# for p in [image0,image1,image2,image3,pcd]:
#     if(not os.path.exists(p)):
#         os.mkdir(p)

# sys.exit()
pcd = h5.create_group('pcds')
image250 = h5.create_group('image250')
image251 = h5.create_group('image251')
image252 = h5.create_group('image252')
image253 = h5.create_group('image253')
image18254730 = h5.create_group('image18254730')
image18303551 = h5.create_group('image18303551')
image18304070 = h5.create_group('image18304070')
image18304180 = h5.create_group('image18304180')
image18304181 = h5.create_group('image18304181')
image18304860 = h5.create_group('image18304860')

scan = h5.create_group('scan')
aaa = h5.create_group('aaa')

laser_projection=LaserProjection()

class ImageCreator():

    def __init__(self):
        self.bridge = CvBridge()
        with rosbag.Bag(bag_path, 'r') as bag:  # 要读取的bag文件；
            print bag
            for topic, msg, t in tqdm.tqdm(bag.read_messages()):
                # print "%.6f" % msg.header.stamp.to_sec()
                try:
                    if topic == "/image/19279250":  # 图像的topic；
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                        except CvBridgeError as e:
                            print e

                        timestr = "%.6f" % msg.header.stamp.to_sec()
                        # print cv_image
                        image250.create_dataset(timestr,data=cv_image)
                        print topic+','+timestr

                    elif topic == "/scan":  # 图像的topic；
                        cloud_out = laser_projection.projectLaser(msg)
                        lidar = pc2.read_points(cloud_out)
                        timestr = "%.6f" % msg.header.stamp.to_sec()
                        points = np.array(list(lidar))
                        # points=points[:,:3]
                        # np.savetxt(timestr+'.txt',points)
                        # print laser_data_path
                        scan.create_dataset(timestr, data=points)
                        # print points
                        print topic + ',' + timestr



                    elif topic == "/aaa":  # 图像的topic；
                        cloud_out = laser_projection.projectLaser(msg)
                        lidar = pc2.read_points(cloud_out)
                        timestr = "%.6f" % msg.header.stamp.to_sec()
                        points = np.array(list(lidar))
                        # points=points[:,:3]
                        # np.savetxt(timestr+'.txt',points)
                        # print laser_data_path
                        aaa.create_dataset(timestr, data=points)
                        print topic+','+timestr


                    elif topic == "/image/18254730":  # 图像的topic；
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                        except CvBridgeError as e:
                            print e
                        timestr = "%.6f" % msg.header.stamp.to_sec()
                        image18254730.create_dataset(timestr,data=cv_image)
                        # print 'image251'+timestr
                        print topic+','+timestr


                    elif topic == "/image/18303551":  # 图像的topic；
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                        except CvBridgeError as e:
                            print e
                        timestr = "%.6f" % msg.header.stamp.to_sec()
                        image18303551.create_dataset(timestr, data=cv_image)
                        # print 'image251'+timestr
                        print topic+','+timestr


                    elif topic == "/image/18304070":  # 图像的topic；
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                        except CvBridgeError as e:
                            print e
                        timestr = "%.6f" % msg.header.stamp.to_sec()
                        image18304070.create_dataset(timestr, data=cv_image)
                        # print 'image251'+timestr
                        print topic+','+timestr


                    elif topic == "/image/18304180":  # 图像的topic；
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                        except CvBridgeError as e:
                            print e
                        timestr = "%.6f" % msg.header.stamp.to_sec()
                        image18304180.create_dataset(timestr, data=cv_image)
                        # print 'image251'+timestr
                        print topic+','+timestr


                    elif topic == "/image/18304181":  # 图像的topic；
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                        except CvBridgeError as e:
                            print e
                        timestr = "%.6f" % msg.header.stamp.to_sec()
                        image18304181.create_dataset(timestr, data=cv_image)
                        # print 'image251'+timestr
                        print topic+','+timestr


                    elif topic == "/image/18304860":  # 图像的topic；
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                        except CvBridgeError as e:
                            print e
                        timestr = "%.6f" % msg.header.stamp.to_sec()
                        image18304860.create_dataset(timestr, data=cv_image)
                        # print 'image251'+timestr
                        print topic+','+timestr


                    elif topic == "/image/19279251":  # 图像的topic；
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                        except CvBridgeError as e:
                            print e
                        timestr = "%.6f" % msg.header.stamp.to_sec()
                        image251.create_dataset(timestr, data=cv_image)
                        # print 'image251'+timestr
                        print topic+','+timestr


                        # print image_name
                    elif topic == "/image/19279252":  # 图像的topic；
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                        except CvBridgeError as e:
                            print e
                        timestr = "%.6f" % msg.header.stamp.to_sec()
                        image252.create_dataset(timestr,data=cv_image)
                        # print 'image252'+timestr
                        print topic+','+timestr


                        # print image_name
                    elif topic == "/image/19279253":  # 图像的topic；
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                        except CvBridgeError as e:
                            print e
                        timestr = "%.6f" % msg.header.stamp.to_sec()
                        image253.create_dataset(timestr,data=cv_image)
                        # print image_name
                        # print 'image253'+timestr
                        print topic+','+timestr


                    elif topic == "/pandar_points":  # laser的topic；
                        timestr = "%.6f" % msg.header.stamp.to_sec()
                        # laser_data_name = timestr + ".txt"
                        # laser_data_path = os.path.join(pcd, laser_data_name)
                        lidar = pc2.read_points(msg)
                        points = np.array(list(lidar))
                        # points=points[:,:3]
                        # np.savetxt(laser_data_path,points)
                        # print laser_data_path
                        pcd.create_dataset(timestr,data=points)
                        # print 'pcd'+timestr
                        print topic+','+timestr


                except:
                    # print e
                # except:
                    continue


if __name__ == '__main__':

    # rospy.init_node(PKG)

    try:
        image_creator = ImageCreator()
    except rospy.ROSInterruptException:
        pass

