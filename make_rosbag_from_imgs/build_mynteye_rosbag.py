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
            bag.write('/mynteye/left/image_raw', img_msg, Stamp)
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
            bag.write('/mynteye/right/image_raw', img_msg, Stamp)
    finally:
        bag.close()


if __name__ == "__main__":
    print ("absolute left img path: " + sys.argv[1])
    print ("absolute right img path: " + sys.argv[2])
    print ("find bag file here: " + sys.argv[3])
    output_bag_head = sys.argv[3]
    captured_seq = [[4280, 4420],  # 第一
                    [6960, 7170],  # 第二
                    [14760, 14980],  # 第三
                    [19105, 19705],  # 第四
                    [39400, 39850],  # 第十一
                    [57590, 57710],  # 第十
                    [71100, 71420],  # 第十二
                    [21000, 22200],  # 第五
                    [24200, 24400],  # 第六
                    [21750, 21870]]  # 第十三
    # for i in range(len(captured_seq)):
    for i in {0, 1, 3, 4, 5, 6}:
        output_bag = output_bag_head + "-" + str(i + 1) + ".bag"
        sys.argv[3] = output_bag
        CreateBag(sys.argv[1:], captured_seq=captured_seq[i])
        print ("FINISH & OUTPUT bag file at " + sys.argv[3])
