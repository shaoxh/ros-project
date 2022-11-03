import os.path
import sys
import numpy as np

np.set_printoptions(suppress=True)

if __name__ == '__main__':
    if len(sys.argv) < 3:
        sys.exit("please input time.txt and tum_pose.txt")

    time_file_path = sys.argv[1]
    pose_file_path = sys.argv[2]

    with open(time_file_path, 'r') as time_file:
        time_strings = time_file.readlines()
    pose_file = np.loadtxt(pose_file_path)
    pixel4d_pose = list()
    for time_str in time_strings:
        time = float(time_str)
        idx = np.argmin(np.abs(pose_file[:, 0] - time))
        if np.abs(pose_file[idx, 0] - time) < 0.001:
            line = list()
            xyz = pose_file[idx, 1:4]
            line.append(time_str[0:-2] + ".jpg")
            line.append(str(xyz[0]))
            line.append(str(xyz[1]))
            line.append(str(xyz[2]))
            pixel4d_pose.append(line)
    pixel4d_pose_path = os.path.join(os.path.dirname(time_file_path), 'pixel4d_pose.txt')
    pixel4d_pose_file = open(pixel4d_pose_path, 'w')
    for line in pixel4d_pose:
        for elements in line:
            pixel4d_pose_file.write(elements)
            pixel4d_pose_file.write("\t")
        pixel4d_pose_file.write("\n")
