#coding:utf-8
import numpy as np
import serial
import rospy
# import PySpin
# rospy.init_node('pointgrey')
# print(rospy.Time.now())
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import serial
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation
import time
import tqdm

pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=200)
rospy.init_node('imu_node', anonymous=True)
rate = rospy.Rate(200)

class SerialDecoder(object):
    """
    初始化参数：
        bufsize：缓存区长度，可根据需要解析的包大小调节
        com:串口号，是字符串，例如'COM9'
        band：波特率，
        debug：是否输出调试信息
    功能：
        实现对串口协议的解析，协议内容：
            帧头：
                0xAA 0xAA, 两个字节
            功能字：
                0x02, 一个字节
            数据长度：
                0x0A, 一个字节，代表数据内容长度，必定为偶数
            数据内容：
                字节个数不确定，每两个字节组成一个16位无符号整形数
                高位在前，地位在后
            校验和：
                一个字节，为前面所以字节的和取低八位
    """

    def __init__(self, bufsize=200, com='COM1', band=9600, debug=False):
        """
        输入参数：
            bufsize：缓存区长度，可根据需要解析的包大小调节
            com:串口号，是字符串，例如'COM9'
            band：波特率，
            debug：是否输出调试信息
        """
        self.current = 0  # 当前buf中数据长度
        self.bufsize = bufsize
        self.buf = [0 for i in range(bufsize)]  # 保存数据buf,必须按照长度初始化
        self.data_signal = []  # 解析到的有效数据
        self.debug = debug  # 是否输出调试信息
        self.newdata = False  # 自上次取获取数据后是否解析出新的的数据
        self.ser = serial.Serial(com, band)

        NMEA = [0xAA, 0x55, 0x00, 0x00, 0x07, 0x00, 0x34, 0x3B, 0x00]
        STOP = [0xAA, 0x55, 0x00, 0x00, 0x07, 0x00, 0xFE, 0x05, 0x01]
        QuatData = [0xAA, 0x55, 0x00, 0x00, 0x07, 0x00, 0x36, 0x3D, 0x00]

        self.ser.write(STOP)
        print '关闭imu'
        time.sleep(2)
        self.ser.write(QuatData)
        print '开启QuatData格式'
        print '初始化中,等待30s,保持imu不动'
        for i in tqdm.tqdm(range(30)):
            time.sleep(1)


    def getdata(self):  # 获取解析到的数据
        serialbuf = self.ser.read_all()
        self.append(serialbuf)
        newdata = self.newdata
        self.newdata = False
        return newdata, self.data_signal

    def append(self, serialbuf):  # 把串口数据加入到buf中，加入完成后尝试进行解析
        filebuf = np.fromstring(serialbuf, dtype=np.uint8)
        if self.current + filebuf.size > self.bufsize:
            self.buf[self.current:self.bufsize] =   \
                filebuf[0:self.bufsize - self.current - filebuf.size]
        else:
            self.buf[self.current:self.current + filebuf.size] = filebuf
        self.current += filebuf.size
        self.process()

    def hex2signedint(self,index0):
        result = (self.buf[index0+1] * 256 + self.buf[index0])
        if result >= 0x8000:
            result -= 0x10000
        return result

    def process(self):  # 进行数据解析
        while self.current > 129:  # 如果队列中数据多于2包数据
            if self.buf[0] == 170 and self.buf[1] == 85 and self.buf[2] == 1 and self.buf[3] == 54 :
                # if self.debug:
                #     print("检测到帧头，功能字是" + str(self.buf[2]))

                quat_w=self.hex2signedint(6)/10000.0
                quat_x=self.hex2signedint(8)/10000.0
                quat_y=self.hex2signedint(10)/10000.0
                quat_z=self.hex2signedint(12)/10000.0

                # quat_x=(self.buf[9]*256+self.buf[8])/10000.0
                # quat_y=(self.buf[11]*256+self.buf[10])/10000.0
                # quat_z=(self.buf[13]*256+self.buf[12])/10000.0

                gyro_x=self.hex2signedint(14)/50.0
                gyro_y=self.hex2signedint(16)/50.0
                gyro_z=self.hex2signedint(18)/50.0

                acc_x=self.hex2signedint(20)/4000.0
                acc_y=self.hex2signedint(22)/4000.0
                acc_z=self.hex2signedint(24)/4000.0

                header = Header(stamp=rospy.Time.now())
                header.frame_id = 'pandar'
                imu = Imu()
                imu.header = header
                # quat = Rotation.from_euler('ZYX', [heading, pitch, roll], degrees=True).as_quat()
                imu.orientation.w = quat_w
                imu.orientation.x = quat_x
                imu.orientation.y = quat_y
                imu.orientation.z = quat_z

                imu.angular_velocity.x=gyro_x
                imu.angular_velocity.y=gyro_y
                imu.angular_velocity.z=gyro_z

                imu.linear_acceleration.x=acc_x
                imu.linear_acceleration.y=acc_y
                imu.linear_acceleration.z=acc_z
                # print 0
                # str1=str(rospy.Time.now())+','+str1
                pub.publish(imu)

                print quat_w,quat_x,quat_y,quat_z,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z
                # print quat_w,quat_x,quat_y,quat_z

                # datalength = self.buf[3]  # 有效数据长度
                framelength = self.buf[4]+2  # 帧长度
                # datasum = np.sum(self.buf[0:framelength - 1]) % 256
                # if datasum == self.buf[framelength - 1]:  # 校验通过
                #     self.data_signal = self.buf[4:4 + datalength]
                #     self.data_signal = np.array(
                #         self.data_signal, dtype='uint8')
                #     self.data_signal = self.data_signal.reshape(-1, 2)
                #     self.data_signal = self.data_signal[:, 0] * 256 +   \
                #         self.data_signal[:, 1]
                #     self.newdata = True
                #
                #     if self.debug:
                #         print(self.data_signal)

                self.buf = np.roll(self.buf, -framelength)
                self.current -= framelength
                if self.debug:
                    print("解析到一帧数据")
                # else:  # 校验失败
                #     if self.debug:
                #         print("校验和错误")
                #
                #     if 170 in self.buf[2:self.current]:  # 帧头对，但是校验和错误
                #         temparray = self.buf[2:self.current]
                #         if not isinstance(temparray, list):
                #             temparray = temparray.tolist()
                #         offset = temparray.index(170)
                #
                #         self.buf = np.roll(self.buf, -offset)
                #         self.current -= offset
            # 如果解析不到，舍弃前面的数据，直到data[0] == 170
            elif 170 in self.buf[0:self.current]:
                if self.debug:
                    print("接收到无效数据")

                # temparray = self.buf[0:self.current]
                # if not isinstance(temparray, list):
                #     temparray = temparray.tolist()
                # offset = temparray.index(170)
                for i in range(len(self.buf)):
                    if self.buf[i+0] == 170 and self.buf[i+1] == 85 and self.buf[i+2] == 1 and self.buf[i+3] == 54 :

                        self.buf = np.roll(self.buf, -i)
                        self.current -= i
                        break


if __name__ == '__main__':
    serialdecoder = SerialDecoder(10000, '/dev/ttyUSB0', 115200, False)
    count = 0
    while 1:
        newdata, signal = serialdecoder.getdata()
        # if newdata:
        #     count += 1
        #     # print(str(count) + ":" + str(signal))
        #     print(str(count) + ":" + str(signal))
