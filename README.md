# RosCar

基于全向ROS小车的开发工具包。项目封装了常用的速度控制、雷达读取和图像采集等功能，方便在 ROS 环境中快速构建小车应用。

## 依赖环境
- Python >= 3.6
- 已配置好的 ROS，并需要启动 `roscore`
- Python 包：`numpy`、`opencv-python`、`rospy`、`psutil`、`pyserial`

## 安装
1. 安装依赖：
   ```bash
   pip install numpy opencv-python rospy psutil pyserial
   ```
2. 安装库：
   ```bash
   pip install basecar-0.1.5-py3-none-any.whl
   ```
   或者在源码目录执行：
   ```bash
   pip install .
   ```

## 模块简介
- **`basecar.car.RosCar`**：管理小车运动和雷达数据，提供 `vel_set`、`lidar_get`、`close` 等接口。
- **`basecar.camera.Camera`**：包装 `cv2.VideoCapture` 并发布 ROS 图像话题，提供 `open`、`show`、`get`、`close` 方法。
- **`basecar.control.Vel_Pub`**：后台线程循环发布速度控制消息。
- **`basecar.action.Action`** 与 **`basecar.detect.Shoot_Detect`**：用于图像颜色检测和射击控制。

## 使用示例
### 控制小车
```python
from basecar import RosCar
import rospy

car = RosCar("/cmd_vel")
car.lidar_get('/scan_filtered', 359, 638, 100)
rospy.sleep(3)
print(car.front, car.left, car.right, car.x, car.y, car.yaw)
car.close()
```
上述代码摘自 `basecar/car.py`【F:basecar/car.py†L121-L127】

### 相机采集
```python
from basecar import Camera
import cv2

cam = Camera(0)
cam.open()
while True:
    cam.show()
    if cv2.waitKey(10) == ord('q'):
        break
cv2.destroyAllWindows()
cam.close()
```
示例位于 `basecar/camera.py`【F:basecar/camera.py†L142-L150】

### 射击控制
```python
from basecar import Camera, Action
import time
import cv2

cam = Camera(0)
cam.open()
detect = Action("/dev/ttyUSB0", 115200)
while True:
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
```
示例位于 `basecar/action.py`【F:basecar/action.py†L39-L54】

## 目录结构
```
RosCar/
├── basecar/                       # 主程序源码
│   ├── camera.py
│   ├── car.py
│   ├── action.py
│   ├── control.py
│   └── __init__.py
├── basecar-0.1.5-py3-none-any.whl # 打包好的安装文件
├── setup.py
└── README.md
```

## 快速开始
1. 启动 `roscore` 并确保小车硬件及各传感器正常工作。
2. 按上述步骤安装后即可在脚本中导入 `basecar` 进行开发。
