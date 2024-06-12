import cv2
import rospy
import time
from threading import Thread
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from ar_track_alvar_msgs.msg import AlvarMarkers


class Image_():
    def __init__(self, img) -> None:
        """
        :param img: 原始图像
        .crop_center : 从中心进行裁剪
        """

        self.data = img
        self.h, self.w = img.shape[:2]
        self.x = self.w // 2
        self.y = self.h // 2

    def __call__(self):  # 调用实例返回图像
        return self.data

    def crop_center(self, crop_width, crop_height):  # 从中心裁剪图像

        """
        :param crop_width: 裁剪宽度
        :param crop_height: 裁剪高度
        """

        start_x = max(0, self.x - crop_width // 2)
        start_y = max(0, self.y - crop_height // 2)
        end_x = min(self.w, self.x + crop_width // 2)
        end_y = min(self.h, self.y + crop_height // 2)
        return self.data[start_y:end_y, start_x:end_x]


class Camera():
    def __init__(self, source, P=[516.422974, 0.000000, 372.887246, 0.000000,
                                  0.000000, 520.513672, 226.444701, 0.000000,
                                  0.000000, 0.000000, 1.000000, 0.000000],
                 R=[1.000000, 0.000000, 0.000000,
                    0.000000, 1.000000, 0.000000,
                    0.000000, 0.000000, 1.000000],
                 K=[514.453865, 0.000000, 368.311232,
                    0.000000, 514.428903, 224.580904,
                    0.000000, 0.000000, 1.000000],
                 D=[0.031723, -0.045138, 0.004146, 0.006205, 0.000000]) -> None:

        """
        :param source: 视频流地址
        .open   :打开视频流线程
        .close  :关闭视频流线程
        .show   :展示视频内容
        .get    :获取视频内容
        """
        self.image_pub = rospy.Publisher("/usb_cam/image_raw", Image, queue_size=1)
        self.image_pub1 = rospy.Publisher("/usb_cam/camera_info", CameraInfo, queue_size=1)
        self.P = P
        self.R = R
        self.K = K
        self.D = D
        self.cap = cv2.VideoCapture(source)
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 1)
        self.frame = None
        self.stop = False
        self.id = []
        self.pose = []

    def cv2_to_imgmsg(self, cv_image):
        B = Header()
        B.frame_id = "usb_cam"
        img_msg = Image()
        img_msg.header = B
        img_msg.height = cv_image.shape[0]
        img_msg.width = cv_image.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = 0
        img_msg.data = cv_image.tostring()
        img_msg.step = len(
            img_msg.data) // img_msg.height  # That double line is actually integer division, not a comment
        return img_msg

    def ar_marker_subscriber(self):
        def ar_marker_callback(msg):
            id = []
            pose = []
            for marker in msg.markers:
                id.append(marker.id)
                p = marker.pose.pose
                pose.append([p.position.x, p.position.y, p.position.z])
            self.id = id
            self.pose = pose

        self.sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, ar_marker_callback, queue_size=7)
        rospy.spin()

    def thread(self, cap):
        while not self.stop:
            _, frame = cap.read()
            try:
                self.image_pub.publish(self.cv2_to_imgmsg(frame))
                msg = CameraInfo()
                A = Header()
                A.frame_id = "usb_cam"
                msg.header = A
                msg.height = 720
                msg.width = 1280
                msg.P = self.P
                msg.R = self.R
                msg.K = self.K
                msg.D = self.D
                self.image_pub1.publish(msg)
            except Exception as e:
                print(e)

            self.frame = Image_(frame)
            time.sleep(0.02)

    def open(self):  # 打开摄像头线程
        thread_cam = Thread(target=self.thread, args=(self.cap,))
        thread_ar = Thread(target=self.ar_marker_subscriber)
        thread_cam.start()
        thread_ar.start()

    def close(self):  # 关闭摄像头线程
        self.sub.unregister()
        self.stop = True

    def show(self):  # 展示摄像头图像
        if self.frame is not None:
            cv2.imshow("raw", self.frame())

    def get(self):  # 获取摄像头图像
        return self.frame

    def get_ar(self):
        return self.id, self.pose


if __name__ == "__main__":
    cam = Camera(0)
    cam.open()
    while 1:
        cam.show()
        if cv2.waitKey(10) == ord('q'):
            break
    cv2.destroyAllWindows()
    cam.close()