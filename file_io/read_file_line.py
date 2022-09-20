# coding=utf-8
import gzip
import chardet


if __name__ == '__main__':
    file_path = "/home/godu/Documents/orbslam-save-map/ORB_SLAM2/MapPointandKeyFrame.bin"
    file = open(file_path, "rb")
    lines = file.readlines()
    kkk = lines[0]
    for line in lines:
        kkk = line.decode("utf-8")
        print line
