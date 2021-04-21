# coding:utf-8

import os

import cv2
import rosbag
import rospy
from cv_bridge import CvBridge
import sys

if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit("未指定 rosbag 包的地址")

    bag_file = sys.argv[1]
    bag = rosbag.Bag(bag_file, "r")

    info = bag.get_type_and_topic_info()
    print(info)

    folder = os.path.split(bag_file)[0]

    with rosbag.Bag(bag_file, 'r') as bag:  # 要读取的bag文件；
        try:
            for topic, msg, t in bag.read_messages():
                if topic == "/mynteye/left/image_raw":
                    img_dir = os.path.join(folder, 'mynteye/mav0/cam0/data')
                    if not os.path.exists(img_dir):
                        print (img_dir)
                        os.makedirs(img_dir)
                    try:
                        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
                    except Exception as e:
                        print (e)
                    timestr = "%.6f" % msg.header.stamp.to_sec()
                    # %.6f表示小数点后带有6位，可根据精确度需要修改；
                    image_name = timestr + ".png"  # 图像命名：时间戳.jpg
                    image_name = os.path.join(img_dir, image_name)
                    # print (image_name)

                    cv2.imwrite(image_name, cv_image)  # 保存；
                if topic == "/mynteye/left/image_raw":
                    img_dir = os.path.join(folder, 'mynteye/mav0/cam1/data')
                    if not os.path.exists(img_dir):
                        os.makedirs(img_dir)
                    try:
                        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
                    except Exception as e:
                        print (e)
                    timestr = "%.6f" % msg.header.stamp.to_sec()
                    # %.6f表示小数点后带有6位，可根据精确度需要修改；
                    image_name = timestr + ".png"  # 图像命名：时间戳.jpg
                    image_name = os.path.join(img_dir, image_name)
                    # print (image_name)
                    cv2.imwrite(image_name, cv_image)  # 保存；
                if topic == '/mynteye/depth/image_raw':
                    img_dir = os.path.join(folder, 'mynteye/depth/')
                    if not os.path.exists(img_dir):
                        os.makedirs(img_dir)
                    try:
                        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
                    except Exception as e:
                        print (e)
                    timestr = "%.6f" % msg.header.stamp.to_sec()
                    # %.6f表示小数点后带有6位，可根据精确度需要修改；
                    image_name = timestr + ".png"  # 图像命名：时间戳.jpg
                    image_name = os.path.join(img_dir, image_name)
                    # print (image_name)
                    cv2.imwrite(image_name, cv_image)  # 保存；
                if topic== '/mynteye/left_rect/image_rect':
                    img_dir = os.path.join(folder, 'mynteye/rect/cam0/data')
                    if not os.path.exists(img_dir):
                        os.makedirs(img_dir)
                    try:
                        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
                    except Exception as e:
                        print (e)
                    timestr = "%.6f" % msg.header.stamp.to_sec()
                    # %.6f表示小数点后带有6位，可根据精确度需要修改；
                    image_name = timestr + ".png"  # 图像命名：时间戳.jpg
                    image_name = os.path.join(img_dir, image_name)
                    # print (image_name)
                    cv2.imwrite(image_name, cv_image)  # 保存；
                if topic == '/mynteye/imu/data_raw':
                    print msg
        except rospy.ROSInterruptException:
            pass
