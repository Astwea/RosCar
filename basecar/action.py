import cv2
import rospy
import time
from threading import Thread
from .camera import Camera
import numpy as np
from std_msgs.msg import String
import serial


class Action():
    def __init__(self, shoot_topic, baudrate) -> None:
        self.result = 0
        self.serial_port = serial.Serial(shoot_topic, baudrate)
        self.runing = False

    def color(self, frame, up, low, width=100):  # 获取中心点图像颜色置信度
        img = frame.crop_center(width, width)
        if not isinstance(up, np.ndarray):
            up = np.array(up)
        if not isinstance(low, np.ndarray):
            low = np.array(low)
        mask = cv2.inRange(img, low, up)
        white_pixels = cv2.countNonZero(mask)
        cv2.imshow("test", mask)
        return white_pixels / (width * width)

    def shoot(self):  # 开炮
        buf = bytearray([0x55, 0x01, 0x12, 0x00, 0x00, 0x00, 0x01, 0x69])
        self.serial_port.write(buf)
        time.sleep(0.1)
        buf = bytearray([0x55, 0x01, 0x11, 0x00, 0x00, 0x00, 0x01, 0x68])
        self.serial_port.write(buf)
    def shoot_stop(self):  # 停止
        buf = bytearray([0x55, 0x01, 0x11, 0x00, 0x00, 0x00, 0x01, 0x68])
        self.serial_port.write(buf)


if __name__ == "__main__":
    cam = Camera(0)
    cam.open()
    detect = Shoot_Detect()
    while 1:
        frame = cam.get()
        if frame:
            score = detect.color(frame, [130, 255, 255], [110, 50, 50])
            print("{:.3f}".format(score))
            if cv2.waitKey(10) == ord('q'):
                detect.shoot()
                time.sleep(0.3)
                detect.shoot_stop()
                break
    cv2.destroyAllWindows()
    cam.close()