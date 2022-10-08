# coding:utf-8

import os
import sys

import cv2

if __name__ == '__main__':
    ## 使用发给你发 python2 tjrobot_2_mynteye_lidar.py '/media/shawn/Bookup/Data/leggedrobot/0115/1136/full_2022-01-14-11-27-03.bag' false
    print(
        "instruction command : python2 skip_exatract_img.py ABS_IMG_PATH SKIP_NUM")
    if len(sys.argv) < 3:
        print(
            "命令参考：python2 tjrobot_2_mynteye_lidar.py \n /home/tjch114/Data/2022-10-04/0942-3-2/zed2/mav0/cam0/data \n "
            "60")
        sys.exit("请指定 imgs 绝对的地址和跳过多少张: 60")
    img_file = sys.argv[1]
    skip_num = sys.argv[2]
    print("imgs 的绝对地址：{}，跳过：{} 张图像".format(img_file, skip_num))
    file_names = os.listdir(img_file)
    base_img_path = os.path.dirname(img_file)
    print("reading img {} files: ".format(len(file_names)))
    skip_dir_path = os.path.join(base_img_path, "skip-data")
    print("skip imgs will be written to: \n{}".format(skip_dir_path))
    if not os.path.exists(skip_dir_path):
        os.makedirs(skip_dir_path)
    skip_threshold = int(skip_num)
    skip_index = 1
    is_skip = True
    for img_name in file_names:
        if is_skip:
            skip_index += 1
            is_skip = skip_index < skip_threshold
            continue
        else:
            skip_index = 1
            is_skip = True
        abs_img_path = os.path.join(img_file, img_name)
        img = cv2.imread(abs_img_path)
        skip_img_path = os.path.join(skip_dir_path, img_name)
        cv2.imwrite(skip_img_path, img)
