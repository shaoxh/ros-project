# coding:utf-8

import os
import sys
sys.path.append('/home/godu/PycharmProjects/rosbag_io/dist-packages')

import cv2
import rosbag
import rospy
from cv_bridge import CvBridge


if __name__ == '__main__':
    ## 使用发给你发 python2 tjrobot_2_mynteye_lidar.py '/media/shawn/Bookup/Data/leggedrobot/0115/1136/full_2022-01-14-11-27-03.bag' false
    print ("instruction command : python2 tjrobot_2_mynteye_lidar.py '/media/shawn/Bookup/Data/leggedrobot/0115/1136/full_2022-01-14-11-27-03.bag' false")
    if len(sys.argv) < 3:
        print("命令参考：python2 tjrobot_2_mynteye_lidar.py /home/shawn/Data/1102/full_2021-11-02-16-26-16.bag  true")
        sys.exit("请指定 rosbag 包绝对的地址 和 是否保存图像：true or false")


    isWrite = sys.argv[2]

    if isWrite not in {"true", "false"}:
        exit("请输入是否输出图像的参数：true or false")

    bag_file = sys.argv[1]
    print ("reading bag file: " + bag_file)
    bag = rosbag.Bag(bag_file, "r")

    info = bag.get_type_and_topic_info()
    print(info)

    folder = os.path.split(bag_file)[0]

    img_dir_mynteye_left_rct = os.path.join(folder, 'mynteye/mav0/cam0/data')
    if not os.path.exists(img_dir_mynteye_left_rct):
        print (img_dir_mynteye_left_rct)
        os.makedirs(img_dir_mynteye_left_rct)
    img_dir_mynteye_right_rct = os.path.join(folder, 'mynteye/mav0/cam1/data')
    if not os.path.exists(img_dir_mynteye_right_rct):
        os.makedirs(img_dir_mynteye_right_rct)

    img_dir_zed_left_rct = os.path.join(folder, 'zed2/mav0/cam0/data')
    if not os.path.exists(img_dir_zed_left_rct):
        os.makedirs(img_dir_zed_left_rct)
    img_dir_zed_right_rct = os.path.join(folder, 'zed2/mav0/cam1/data')
    if not os.path.exists(img_dir_zed_right_rct):
        os.makedirs(img_dir_zed_right_rct)

    img_dir_mynteye_depth = os.path.join(folder, 'mynteye/mav0/depth')
    if not os.path.exists(img_dir_mynteye_depth):
        os.makedirs(img_dir_mynteye_depth)
    time_zed2 = os.path.join(os.path.join(folder, 'zed2/mav0'), 'time.txt')
    time_mynteye_list = list()
    time_zed2_list = list()
    with rosbag.Bag(bag_file, 'r') as bag:  # 要读取的bag文件;
        try:
            for topic, msg, t in bag.read_messages():
                if topic == '/mynteye/left_rect/image_rect':
                    try:
                        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
                    except Exception as e:
                        print (e)
                    timestr = "%.6f" % msg.header.stamp.to_sec()
                    # print (msg.header.seq)
                    # %.6f表示小数点后带有6位，可根据精确度需要修改；
                    image_name = timestr + ".jpg"  # 图像命名：时间戳.jpg
                    image_name = os.path.join(img_dir_mynteye_left_rct, image_name)
                    # print (image_name)
                    if isWrite == 'true':
                        cv2.imwrite(image_name, cv_image)  # 保存；
                    time_mynteye_list.append(timestr)
                if topic == "/mynteye/right_rect/image_rect":
                    try:
                        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
                    except Exception as e:
                        print (e)
                    timestr = "%.6f" % msg.header.stamp.to_sec()
                    # %.6f表示小数点后带有6位，可根据精确度需要修改；
                    image_name = timestr + ".jpg"  # 图像命名：时间戳.jpg
                    image_name = os.path.join(img_dir_mynteye_right_rct, image_name)
                    # print (image_name)
                    if isWrite == 'true':
                        cv2.imwrite(image_name, cv_image)  # 保存；
                if topic == '/mynteye/depth/image_raw':
                    try:
                        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
                    except Exception as e:
                        print (e)
                    timestr = "%.6f" % msg.header.stamp.to_sec()
                    # %.6f表示小数点后带有6位，可根据精确度需要修改；
                    image_name = timestr + ".jpg"  # 图像命名：时间戳.jpg
                    image_name = os.path.join(img_dir_mynteye_depth, image_name)
                    # print (image_name)
                    if isWrite == 'true':
                        cv2.imwrite(image_name, cv_image)  # 保存；
                if topic == '/mynteye/imu/data_raw':
                    pass
                    # print msg

                if topic == '/zed2/zed_node/left/image_rect_color':
                    try:
                        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
                        cv2.namedWindow('zed2-left', 0)
                        cv2.resizeWindow('zed2-left', 800, 600)  # 自己设定窗口图片的大小
                        cv2.imshow('zed2-left', cv_image)
                        cv2.waitKey(1)
                    except Exception as e:
                        print (e)
                    timestr = "%.6f" % msg.header.stamp.to_sec()
                    time_zed2_list.append(timestr)
                    # %.6f表示小数点后带有6位，可根据精确度需要修改；
                    image_name = timestr + ".jpg"  # 图像命名：时间戳.jpg
                    image_name = os.path.join(img_dir_zed_left_rct, image_name)
                    print ("on saving image {}".format(image_name))
                    if isWrite == 'true':
                        cv2.imwrite(image_name, cv_image)  # 保存；
                if topic == '/zed2/zed_node/right/image_rect_color':
                    try:
                        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
                    except Exception as e:
                        print (e)
                    timestr = "%.6f" % msg.header.stamp.to_sec()
                    # %.6f表示小数点后带有6位，可根据精确度需要修改；
                    image_name = timestr + ".jpg"  # 图像命名：时间戳.jpg
                    image_name = os.path.join(img_dir_zed_right_rct, image_name)
                    # print (image_name)
                    if isWrite == 'true':
                        cv2.imwrite(image_name, cv_image)  # 保存；
        except rospy.ROSInterruptException:
            pass
        print ("Save time string list as {}".format(time_zed2))
        file = open(time_zed2, 'a')
        for time in time_zed2_list:
            file.write(time)
            file.write('\n')
        file.close()
