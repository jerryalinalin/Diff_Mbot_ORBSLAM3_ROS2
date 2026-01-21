# ORB-SLAM3-ROS2 差速小车仿真项目 README
## 项目简介

本项目实现了 **ORB-SLAM3** 与 **ROS2** 的深度集成，基于 **Gazebo** 仿真环境验证差速小车的视觉 SLAM 定位与建图功能。支持单目相机图像输入，通过 **ROS2 话题机制** 实现数据传输，结合 **ORB-SLAM3** 强大的特征提取、回环检测与优化能力，实现高精度实时定位与地图构建。

## 环境与依赖

### 依赖项

| 依赖项 | 版本 / 信息 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 LTS |
| ROS2 版本 | Humble Hawksbill |
| 编译器 | GCC 11.4.0 |
| CMake 标准 | C++14 |
| ORB-SLAM3 | master 分支（2025 年 1 月 5 日获取） |
| Eigen | 5.0.1（源码编译安装） |
| Pangolin | 0.9.4（源码编译安装） |
| OpenCV | 4.2.0（源码编译安装，禁用 LAPACK 以解决兼容性问题） |
| 其他依赖 | g2o、DBoW2、Sophus（包含在 ORB-SLAM3 Thirdparty 目录，自动编译） |
| ROS2 依赖包 | vision_opencv、message_filters、gazebo_ros_pkgs |

## ORB-SLAM3 获取方式

### 1. 核心源码获取

```bash
# 克隆 ORB-SLAM3 官方仓库
gitclone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3

# 可选：切换到对应 commit（确保版本一致性）
git checkout master# 默认分支，2025年1月5日获取的版本
```

### 2. ROS2 适配包获取

本项目基于开源 **ROS2 wrapper** 适配，仓库地址：

```bash
# 在 ROS2 工作空间 src 目录下克隆
cd ~/ros2_ws/src
gitclone https://github.com/zang09/ORB_SLAM3_ROS2.git orbslam3_ros2
```

## 编译步骤

### 前置准备：创建工作目录

```bash
# 创建 SLAM 主工作目录（用于存放 ORB-SLAM3 及依赖）
mkdir -p ~/SLAM

# 创建 ROS2 工作空间
mkdir -p ~/ros2_ws/src
```

### 第一步：安装核心依赖库

### 1. Eigen 数学库

```bash
cd ~/SLAM
gitclone https://gitlab.com/libeigen/eigen.git
cd eigen
mkdir build &&cd build
cmake ..
sudo make install
# 验证：ls /usr/local/include | grep Eigen
```

### 2. Pangolin 可视化库

```bash
cd ~/SLAM
gitclone https://github.com/stevenlovegrove/Pangolin.git

# 安装依赖
sudo apt-get install libglew-dev libboost-dev libboost-thread-dev libboost-filesystem-dev

cd Pangolin
mkdir build &&cd build
cmake ..
make
sudo make install
# 验证：ls /usr/local/include | grep pangolin
```

### 3. OpenCV 图像处理库

```bash
cd ~/SLAM
gitclone https://github.com/opencv/opencv.git
cd opencv
git checkout 4.2.0# 切换到 4.2.0 版本
gitclone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib
git checkout 4.2.0
cd ..

# 安装依赖
sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python3-dev python3-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev
sudo apt install cmake-qt-gui

# 编译（禁用 LAPACK 解决兼容性问题，指定 C++14 标准）
mkdir build &&cd build
cmake -D WITH_LAPACK=OFF \
      -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D CMAKE_CXX_STANDARD=14 \
      -D CMAKE_CXX_STANDARD_REQUIRED=ON \
      -D BUILD_EXAMPLES=OFF \
      -D BUILD_TESTS=OFF \
      -D BUILD_PERF_TESTS=OFF \
      -D BUILD_opencv_java=OFF \
      -D BUILD_opencv_python2=OFF \
      -D BUILD_opencv_python3=ON \
      ..
make -j$(nproc)# 多核编译
sudo make install
sudo ldconfig
# 验证：opencv_version（应输出 4.2.0）
```

### 第二步：编译 ORB-SLAM3 核心库

```bash
cd ~/SLAM/ORB_SLAM3
chmod +x build.sh
./build.sh# 自动编译 Thirdparty 依赖及 ORB-SLAM3 本体

# 编译成功提示：[100%] Built target stereo_inertial_tum_vi_old
```

### 第三步：编译 ROS2 工作空间

### 1. 修改路径配置

编辑 `~/ros2_ws/src/orbslam3_ros2/CMakeLists.txt`：将第 5 行路径修改为本地 ROS2 site-packages 路径（例如 `/opt/ros/humble/lib/python3.10/site-packages`）。

编辑 `~/ros2_ws/src/orbslam3_ros2/CMakeModules/FindORB_SLAM3.cmake`：将第 8 行路径修改为 ORB-SLAM3 安装路径（例如 `~/SLAM/ORB_SLAM3`）。

### 2. 编译 ROS2 功能包

```bash
# 安装 ROS2 依赖
sudo apt install ros-$ROS_DISTRO-vision-opencv ros-$ROS_DISTRO-message-filters

# 编译
colcon build --symlink-install --packages-select orbslam3_ros2 mbot_diff

# 加载工作空间环境
source install/setup.bash
```

## 运行步骤

### 前置准备：加载环境变量

```bash
# 加载 ROS2 环境（若未自动加载）
source /opt/ros/humble/setup.bash

# 加载 Gazebo 环境变量（解决启动失败问题）
source /usr/share/gazebo/setup.sh

# 加载本项目工作空间环境
source ~/ros2_ws/install/setup.bash
```

### 第一项：启动 Gazebo 仿真节点（相机 + 差速小车）

```bash
# 启动 Gazebo 仿真环境，加载小车模型和相机
ros2 launch mbot_diff load_mbot_camera_into_gazebo.launch.py

# 成功启动后，Gazebo 会打开室内场景，小车搭载单目相机，发布 /camera/image_raw 话题
```

### 第二项：启动 ORB-SLAM3 ROS2 节点（图像订阅 + SLAM 处理）

```bash
# 打开新终端，加载环境变量（同上）
source ~/ros2_ws/install/setup.bash

# 运行单目 SLAM 节点（参数：词汇表路径、配置文件路径）
ros2 run orbslam3_ros2 mono \
~/SLAM/ORB_SLAM3/Vocabulary/ORBvoc.txt \
~/ros2_ws/src/orbslam3_ros2/config/monocular/TUM1.yaml
```

### 第三项：控制小车运动（测试轨迹与建图）

```bash
# 打开新终端，运行键盘控制节点（需确保 mbot_diff 包包含 teleop 功能）
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 按终端提示操作，控制小车完成移动与回环
```

### 第四项：可视化结果

### Pangolin 实时可视化

启动 ORB-SLAM3 节点后自动打开，可查看轨迹、关键帧、地图点云。

### RViz2 可视化

```bash
ros2 run rviz2 rviz2

# 在 RViz2 中添加话题：/camera_pose（位姿）、/map_points（地图点云），选择合适的固定坐标系
```

## 关键文件说明

| 文件路径 | 功能描述 |
| --- | --- |
| `orbslam3_ros2/config/monocular/TUM1.yaml` | 单目相机参数配置文件（可根据实际调整） |
| `orbslam3_ros2/src/monocular/mono.cpp` | ROS2 图像订阅节点核心代码 |
| `mbot_diff/launch/load_mbot_camera_into_gazebo.launch.py` | 仿真环境启动脚本 |
| `mbot_diff/urdf/mbot_with_camera_gazebo.xacro` | 差速小车 + 相机模型定义 |

## 常见问题解决

### 1. Gazebo gzclient 启动失败

**解决方法**：运行 launch 文件前执行 `source /usr/share/gazebo/setup.sh` 加载环境变量。

### 2. OpenCV 编译报错（LAPACK 兼容问题）

**解决方法**：编译时添加 `-D WITH_LAPACK=OFF` 参数。

### 3. ORB-SLAM3 特征点不足

**解决方法**：修改配置文件 `TUM1.yaml`，降低 FAST 阈值（`iniThFAST=8`）、增加特征点数（`nFeatures=2000`）。

### 4. 图像读取失败

**解决方法**：确保运行命令时数据集 / 仿真话题路径正确，优先使用绝对路径。

## 参考资料

- [ORB-SLAM3 官方仓库](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [ROS2 适配包](https://github.com/zang09/ORB_SLAM3_ROS2)
- [TUM RGB-D 数据集](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download)
