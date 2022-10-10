# coding=utf-8

import cv2
import time, sys, os
from ros import rosbag
import roslib
import rospy
from cv_bridge import CvBridge
from numpy import asarray

# python2.7 make_rosbag_from_imgs/build_mynteye_rosbag.py /media/godu/Element/Data/1204/mynteye/mav0/cam0/data /media/godu/Element/Data/1204/mynteye/mav0/cam0/data /media/godu/Element/Data/1204/mynteye/mav0/time-full.txt /media/godu/Element/Data/1204/bag-mynteye-1204.bag

roslib.load_manifest('sensor_msgs')

# import ImageFile
from PIL import Image as ImagePIL


def CompSortFileNamesNr(f):
    g = os.path.splitext(os.path.split(f)[1])[0]  # get the file of the
    numbertext = ''.join(c for c in g if c.isdigit())
    return int(numbertext)


def ReadImages(filename):
    '''Generates a list of files from the directory'''
    print("Searching directory %s" % filename)
    all = []
    left_files = []
    right_files = []
    files = os.listdir(filename)
    for f in sorted(files):
        if os.path.splitext(f)[1] in ['.bmp', '.png', '.jpg', '.pgm']:
            '''
            if 'left' in f or 'left' in path:
                left_files.append(os.path.join(path, f))
            elif 'right' in f or 'right' in path:
                right_files.append(os.path.join(path, f))
            '''
            all.append(os.path.join(filename, f))
    return all


def ReadIMU(filename):
    '''return IMU data and timestamp of IMU'''
    file = open(filename, 'r')
    all = file.readlines()
    timestamp = []
    imu_data = []
    for f in all:
        line = f.rstrip('\n').split(' ')
        timestamp.append(line[0])
        imu_data.append(line[1:])
    return timestamp, imu_data


def CreateBag(args, captured_seq):  # img,imu, bagname, timestamps
    '''read time stamps'''
    img_left = ReadImages(args[0])
    img_right = ReadImages(args[1])
    '''Creates a bag file with camera images'''
    if not os.path.exists(args[2]):
        os.system(r'touch %s' % args[2])
    bag = rosbag.Bag(args[2], 'w')

    try:
        # left imgs
        for i in range(captured_seq[0], captured_seq[1], 1):
            print("Adding %s" % img_left[i])

            '''read image size'''
            # imgpil = ImagePIL.open(img_left[i]).convert("L")
            imgpil = ImagePIL.open(img_left[i])
            data = asarray(imgpil)
            br = CvBridge()
            Stamp = rospy.rostime.Time.from_sec(float(img_left[i][-21:-4]))
            print ("Timestamp %s" % img_left[i][-21:-4])
            img_msg = br.cv2_to_imgmsg(data)
            img_msg.header.stamp = Stamp
            img_msg.header.frame_id = "camera"
            bag.write('/zed2/zed_node/left/image_rect_color', img_msg, Stamp)
        # right imgs
        for i in range(captured_seq[0], captured_seq[1], 1):
            print("Adding %s" % img_right[i])

            '''read image size'''
            imgpil = ImagePIL.open(img_right[i])
            Stamp = rospy.rostime.Time.from_sec(float(img_right[i][-21:-4]))
            print ("Timestamp %s" % img_right[i][-21:-4])
            data = asarray(imgpil)
            br = CvBridge()
            img_msg = br.cv2_to_imgmsg(data)
            img_msg.header.stamp = Stamp
            img_msg.header.frame_id = "camera"
            bag.write('/zed2/zed_node/right/image_rect_color', img_msg, Stamp)
    finally:
        bag.close()


def readImgsByTimeList(img_left_path, img_right_path, time_list):
    lefts = list()
    rights = list()
    for time_str in time_list:
        img_name = time_str + ".jpg"
        abs_img_left = os.path.join(img_left_path, img_name)
        abs_img_right = os.path.join(img_right_path, img_name)
        if os.path.exists(abs_img_left) and os.path.exists(abs_img_right):
            lefts.append(abs_img_left)
            rights.append(abs_img_right)
    return lefts, rights


def CreateBag(img_left, img_right, output_bag):
    '''Creates a bag file with camera images'''
    if not os.path.exists(output_bag):
        os.system(r'touch %s' % output_bag)
    bag = rosbag.Bag(output_bag, 'w')
    try:
        for i in range(len(left_imgs)):
            img_time = img_left[i][-21:-4]
            print ("writing timestamp %s to bag" % img_time)
            try:
                br = CvBridge()
                Stamp = rospy.rostime.Time.from_sec(float(img_left[i][-21:-4]))
                '''read image left'''
                imgpil_lef = ImagePIL.open(img_left[i])
                data_left = asarray(imgpil_lef)
                img_msg_left = br.cv2_to_imgmsg(data_left)
                img_msg_left.header.stamp = Stamp
                img_msg_left.header.frame_id = "camera"
                '''read image right'''
                imgpil_right = ImagePIL.open(img_right[i])
                data_right = asarray(imgpil_right)
                img_msg_right = br.cv2_to_imgmsg(data_right)
                img_msg_right.header.stamp = Stamp
                img_msg_right.header.frame_id = "camera"

                bag.write('/zed2/zed_node/left/image_rect_color', img_msg_left, Stamp)
                bag.write('/zed2/zed_node/right/image_rect_color', img_msg_right, Stamp)
            except Exception, e:
                print("errors in writing img: {} to bag".format(img_time))
                print e
                continue
    finally:
        bag.close()
    pass


if __name__ == "__main__":
    if len(sys.argv) < 4:
        exit("please input \n left img path: \nright img path: \noutput bag file:, \ntime.txt: ")
    print ("absolute left img path: " + sys.argv[1])
    print ("absolute right img path: " + sys.argv[2])
    print ("output bag file path: " + sys.argv[3])
    print ("time string txt here: {}".format(sys.argv[4]))
    img_left_path = sys.argv[1]
    img_right_path = sys.argv[2]
    time_path = sys.argv[4]
    with open(time_path) as time_file:
        lines_str = time_file.readlines()
    time_str_list = list()
    for line in lines_str:
        if line.isspace():
            continue
        else:
            time_str = line.replace("\n", "")
            time_str_list.append(time_str)
    output_bag_path = sys.argv[3]
    output_bag = os.path.join(output_bag_path, "output.bag")
    left_imgs, right_imgs = readImgsByTimeList(img_left_path, img_right_path, time_str_list)
    CreateBag(left_imgs, right_imgs, output_bag)
    print ("FINISH & OUTPUT bag file at " + output_bag)
