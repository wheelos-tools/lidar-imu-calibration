## lidar-imu-calibration（中文）
**LI-Init** 是一套稳健的实时激光‑惯性初始化方法。我们对原始代码做了修改，使其支持 Apollo 系统里常见的“无 ring 字段”点云。

### 处理 Apollo record
#### 录制 Apollo record
在 Apollo 容器内先启动激光和 GNSS 驱动（下面命令是我的环境示例，你的环境可能需要不同 DAG，关键是 GNSS 和 LiDAR 正常出数据）：
```
mainboard -d modules/drivers/gnss/dag/gnss.dag
mainboard -d modules/drivers/lidar/vanjeelidar/dag/vanjeelidar.dag
```

然后用 cyber_recorder 录 GNSS 和 LiDAR：
```
mkdir -p record/apollo && cyber_recorder record -a -o record/apollo/cal
```

录制时给足激励，例如：
* 启动后保持静止 ≥5–10 秒（别动，先积累初始地图）。
* 上/下坡各 2 次（地库/路肩坡道），产生 pitch。
* 左右各压减速带/低路拱 2 次，或斜压过去，产生 roll。
* 低速 S 弯/环岛 2–3 圈，产生 yaw（最好有快慢交替的转向/加减速）。
* 2/3/4 可按需重复，总时长约 500 秒。

录完停止 cyber_recorder，把 `record` 文件夹拷到 Li-Init 根目录。

#### 将 record 转 rosbag
先安装 bag_convert：
```
pip3 install bag_convert
```
为满足 LI-Init 的输入格式，需要改动 bag_convert 的代码。
⚠️ 路径基于我的环境，请按你自己的 Python 环境定位 pointcloud2.py 与 imu.py 文件进行修改。

##### pointcloud 处理
```
vim /home/soon/.local/lib/python3.10/site-packages/bag_convert/record2bag/pointcloud2.py
```
替换为：
```python
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
from sensor_msgs import point_cloud2

from bag_convert.record2bag.header import to_header


def to_pointcloud(cyber_pointcloud):
    ros_header = Header()
    to_header(ros_header, cyber_pointcloud.header)
    ros_header.stamp = rospy.Time.from_sec(cyber_pointcloud.point[0].timestamp * 1e-9)

    points = []
    i = 0
    for point_xyzit in cyber_pointcloud.point:
        r = int(i / 1800)
        points.append([point_xyzit.x, point_xyzit.y,
                       point_xyzit.z, int(point_xyzit.intensity),
                       point_xyzit.timestamp * 1e-9, r])
        i = i + 1
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('intensity', 12, PointField.UINT8, 1),
              PointField('timestamp', 13, PointField.FLOAT64, 1),
              PointField('ring', 21, PointField.UINT16, 1)]

    return point_cloud2.create_cloud(ros_header, fields, points)
```

##### IMU 处理
```
vim /home/soon/.local/lib/python3.10/site-packages/bag_convert/record2bag/imu.py
```
替换为：
```python
from sensor_msgs.msg import Imu

from bag_convert.record2bag.header import add_header


@add_header
def to_imu(cyber_imu):
    ros_imu = Imu()

    pose = cyber_imu.imu
    ros_imu.orientation.x = 0
    ros_imu.orientation.y = 0
    ros_imu.orientation.z = 0
    ros_imu.orientation.w = 1
    ros_imu.orientation_covariance[0] = -1.0

    ros_imu.angular_velocity.x = pose.angular_velocity.x
    ros_imu.angular_velocity.y = pose.angular_velocity.y
    ros_imu.angular_velocity.z = pose.angular_velocity.z

    ros_imu.linear_acceleration.x = pose.linear_acceleration.x
    ros_imu.linear_acceleration.y = pose.linear_acceleration.y
    ros_imu.linear_acceleration.z = pose.linear_acceleration.z
    return ros_imu
```

转换 record 为 rosbag：
```
# 确认当前在 record 目录
mkdir rosbag
for i in `ls apollo`; do bag_convert -m=r2b -r="apollo/${i}" -b="rosbag/${i}.bag"; done
```

### 使用 Docker 开始标定
本 Dockerfile 在 Ubuntu 22.04、CUDA 11.6、NVIDIA RTX 3060 下测试。

#### 构建 LI-Init 镜像
终端切到 `/docker`，执行：
```
cd docker
docker build -t li_init:1.0 .
```
镜像完成后可用 `docker images` 查看。

#### 创建 LI-Init 容器
创建容器需要 GUI 和文件共享参数。先在本机终端运行：
```
xhost +local:docker
```
确认 $DISPLAY（通常 0 或 1），回到项目根目录，创建容器：
```
cd ..
docker run --gpus all --privileged -it \
           -e NVIDIA_DRIVER_CAPABILITIES=all \
           -e NVIDIA_VISIBLE_DEVICES=all \
           --volume=${PWD}:/home/catkin_ws/src \
           --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
           --net=host \
           --ipc=host \
           --shm-size=1gb \
           --name=cal \
           --env="DISPLAY=$DISPLAY" \
           li_init:1.0 /bin/bash
```
成功后终端类似：
```
================Docker Env Ready================
root@...:/home/catkin_ws#
```

#### 启动 LI-Init ROS 包
先在 `config/vanjee_16_line.yaml` 或 `config/vanjee_32_line.yaml` 设置 topic（基于你的激光雷达线数修改相应的配置）：
```yaml
common:
    lid_topic:  /apollo/sensor/vanjeelidar/up/PointCloud2
    imu_topic:  "/apollo/sensor/gnss/corrected_imu"
```
确保 rosbag 里的 topic 与之匹配。

在容器中开 roscore：
```bash
roscore
```

换另一个容器终端，编译并启动（根据线数选 launch）：
```bash
catkin_make
source devel/setup.bash
roslaunch src/launch/vanjee_16_line.launch   # 或 vanjee_32_line.launch
```

初始化和优化完成后，结果写入 `catkin_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt`。

### 标定效果
16 线万集激光雷达

#### 原始外参（手动测量）
```txt
 0 -1  0  0.0
 1  0  0  0.43
 0  0  1  0.28
 0  0  0  1.0 
```
植物内部结构模糊，涂抹感强，柱子点云较粗并发散。
<div align="center"><img src="image/tree_origin.png" width=100% /></div>

花坛边缘点云发散，厚度相对较大。
<div align="center"><img src="image/edge_origin.png" width=100% /></div>

#### 优化外参
```txt
-0.046183 -0.998895  0.008723 -0.138772
 0.998876 -0.046085  0.011122  0.376353
-0.010707  0.009227  0.999900  0.101143
 0.000000  0.000000  0.000000  1.000000
```
植物内部结构清晰，柱子点云收敛、清晰。
<div align="center"><img src="image/tree_refined.png" width=100% /></div>

花坛边缘厚度优于手动测量外参。
<div align="center"><img src="image/edge_refined.png" width=100% /></div>
