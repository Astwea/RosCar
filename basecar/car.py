import subprocess
import psutil
import time
from basecar.control import Vel_Pub
import rospy
from sensor_msgs.msg import LaserScan
from threading import Thread
import tf


def is_roscore_running():
    """Check if roscore is running."""
    for proc in psutil.process_iter(['pid', 'name']):
        if 'ros' in proc.info['name']:
            return True
    return False


def topic_exists(topic_name):
    """Check if a ROS topic exists."""
    try:
        # List all topics
        result = subprocess.run(['rostopic', 'list'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True,
                                universal_newlines=True)
        topics = result.stdout.split('\n')

        # Check if the topic is in the list
        return topic_name in topics
    except subprocess.CalledProcessError as e:
        print(f"Error checking topic: {e}")
        return False


def start_roscore():
    """Start roscore if it's not already running."""
    if not is_roscore_running():
        print("Starting roscore...")
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', 'roscore; exec bash'])
        time.sleep(5)  # Give roscore some time to start
    else:
        print("roscore is already running.")


class RosCar:
    def __init__(self, vel_topic):

        """
        :param vel_topic: ROS车速度话题
        .vel_set        : 设置小车速度
        .lidar_get      : 获取雷达数据
        .close          : 关闭小车实例
        """

        start_roscore()
        if topic_exists(vel_topic):
            print("ROS Car initialized.")
            self.front, self.left, self.right, self.x, self.y, self.yaw = [-1] * 6
            self.lidar_thread = None
            self.sub = None
            self.stop = 0
            self.vel_control = Vel_Pub(vel_topic)
            self.vel_control.start()
        else:
            print("请检测底盘是否开启或话题名错误")
            exit(0)

        def car_pose():
            rate = rospy.Rate(10)
            listener = tf.TransformListener()
            while not self.stop:
                rate.sleep()
                try:
                    (trans, rot) = listener.lookupTransform("odom", "base_link", rospy.Time(0))
                    # 小车坐标
                    self.x = trans[0]
                    self.y = trans[1]
                    euler = tf.transformations.euler_from_quaternion(rot)
                    self.yaw = euler[2]  # 第三个元素是yaw角
                except Exception as e:
                    print("连接tf中.......")
        pose_thread = Thread(target=car_pose)
        pose_thread.start()

    def vel_set(self, x, y, yaw):

        """
        :param x: x方向速度
        :param y: y方向速度
        :param yaw: z轴转向速度
        """

        self.vel_control.update(x, y, yaw)

    def lidar_get(self, lidar_path, front, left, right):

        """
        :param lidar_path: 雷达订阅话题
        :param front: 雷达前向数(因雷达安装及分辨率不同，该参数自己标定后填入)
        :param left: 雷达左向数
        :param right: 雷达右向数
        """

        self.lidar_thread = Thread(target=self.listen, args=(lidar_path,front,left,right))
        self.lidar_thread.start()

    def listen(self, path, front, left, right):
        def get_laserscan(scan):
            if scan.ranges[front] != float('inf') and scan.ranges[left] != float('inf') \
                    and scan.ranges[right] != float('inf'):
                self.front = scan.ranges[front]
                self.left = scan.ranges[left]
                self.right = scan.ranges[right]
        self.sub = rospy.Subscriber(path, LaserScan, get_laserscan, queue_size=7)

    def close(self):
        self.vel_control.close()
        self.sub.unregister()
        self.stop = 1


# Example usage
if __name__ == "__main__":
    car = RosCar("/cmd_vel")
    car.lidar_get('/scan_filtered', 359, 638, 100)
    rospy.sleep(3)
    print(car.front, car.left, car.right, car.x, car.y, car.yaw)
    car.close()
