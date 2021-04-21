import os

if __name__ == '__main__':
    img_dir = "/media/godu/Samsung_T5/0402-visual-project/mynteye/mav0/cam0/data"
    time_dir = "/media/godu/Samsung_T5/0402-visual-project/mynteye/mav0/time.txt"
    time_list = list()
    img_files = os.listdir(img_dir)
    time_file = open('time.txt', 'w')
    for file_name in img_files:
        time = file_name[0:-4]
        time_list.append(time)
    time_list.sort()
    for time in time_list:
        time_file.write(time)
        time_file.write("\n")
    time_file.close()
