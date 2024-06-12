import rospy
from threading import Thread
from geometry_msgs.msg import Twist

class PID():
    def __init__(self, kp, ki, kd, target) -> None:

        """
        :param kp: 比例系数
        :param ki: 积分系数
        :param kd: 微分系数
        :param target: 目标值
        """

        self.kp, self.ki, self.kd = kp, ki, kd
        self.x, self.t = 0, target
        self.i = 0
        self.past_e = 0

    def update(self, x, target):

        """
        :param x: 更改后的位置
        :param target: 更改后的目标值
        """

        self.x = x
        if self.t != target:
            self.i = 0
        self.t = target

    def calculate(self):
        e = self.x - self.t
        self.i += e
        cnt = self.kp * e + self.ki * self.i + self.kd * (e - self.past_e)
        return cnt

class Vel_Pub():
    def __init__(self, vel_topic) -> None:

        """
        :param vel_topic: ROS车速度话题
        .start          : 开启速度线程
        .close          : 关闭速度线程
        .update         : 更新速度
        """

        self.stop = False
        self.pub = rospy.Publisher(vel_topic, Twist, queue_size=10)
        self.vx = 0
        self.vy = 0
        self.vz = 0
    
    def thread(self):
        rate = rospy.Rate(10)
        while not self.stop:
            msg = Twist()
            msg.linear.x = self.vx
            msg.linear.y = self.vy
            msg.angular.z = self.vz
            self.pub.publish(msg)
            rate.sleep()

    def update(self,x, y, yaw):
        self.vx = x
        self.vy = y
        self.vz = yaw
    
    def start(self):
        pub_thread = Thread(target=self.thread)
        pub_thread.start()
    
    def close(self):
        self.stop = True