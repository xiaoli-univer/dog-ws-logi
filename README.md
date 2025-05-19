底盘工作空间Dog_ws：

1.柔顺控制板块（chassis_dog）
chassis_dog\scripts\force_control_csv.py
功能：实现机器人的柔顺控制，并将速度、力和期望速度数据记录到 CSV 文件中。
依赖：ROS、Modbus 通信库、Matplotlib 等。
主要模块：
odom_callback：订阅 /odom 话题，更新机器人的当前位置和速度。
force_control 类：实现柔顺控制算法，通过对交互力的处理控制速度变化量，使速度在设定值的基础上柔顺波动。
vel_pub 和 vel_pub_triger：发布速度指令到 /cmd_vel 话题。
callback：订阅 /exp_vels 话题，获取期望速度。
使用方法：运行该脚本前确保 ROS 环境已正确配置，运行后会在指定路径生成包含速度、力和期望速度数据的 CSV 文件,并实现柔顺控制。
chassis_dog\scripts\ pymodbus.py
功能：实现 Modbus 通信，读取传感器数据并进行滤波处理，并绘制传感器实时图表。
依赖：Modbus_tk、Serial 等。
主要函数：
modbus_init：初始化 Modbus 主站。
read_once：读取一次传感器数据。
get_init_force：获取初始力的偏移值。
get_filt_out：获取滤波后的传感器输出。
使用方法：运行该脚本，会显示传感器读数的实时图表，并将力数据保存为 pickle 文件。
chassis_dog\launch\control.launch
功能：通过打开节点mini_control.cpp订阅 /cmd_vel 话题，将速度指令转换为特定格式的数据，并通过 InputSocket 发送到 QNX 系统。
使用方法：编译并运行该节点，确保 InputSocket 类已正确实现，节点将持续监听 /cmd_vel 话题并发送数据。
chassis_dog \src\input.cc
功能：提供 ROS 环境下 UDP 通信的基础接口

2.建图与地图定位（FAST_LIO、Fast_LIO Localization）
FAST-LIO（Fast LiDAR-Inertial Odometry，快速激光雷达-惯性里程测量）是一款计算效率高、鲁棒性强的激光雷达-惯性里程测量软件包。它使用紧密耦合的迭代扩展卡尔曼滤波器将激光雷达特征点与 IMU 数据融合在一起，从而在快速运动、噪声或杂乱环境中实现稳健导航。

3. 地图格式转换（pcd_to_rviz、pcd_viewer、pcd2pgm_package）
将三维点云地图转换为三维栅格地图octomap，之后再对该三维栅格地图进行压缩变成二维栅格地图。

4.遥控模块（logi_teleop）
使用遥控器控制机器人以及机械臂运动，同时负责将机器人的实时位置和机械臂的关节角度写入数据库。

5.3D代价地图（spatio_temporal_voxel_layer）
该软件包利用机器人在本地成本地图衰减时间之前提供的所有信息。它并没有为本地规划器设定一个确定的、离散的空间障碍，而是依靠用户对图层的配置，使其具有较短的体素衰减时间（1-30 秒），从而只在相关空间内进行规划。这是一种有意识的设计要求，因为在速度较快或较慢的情况下，本地规划器应经常使用更多信息。本机实现了针对速度的动态成本图缩放。用户有责任为机器人的本地规划器选择一个合理的衰减时间。我发现，对于大多数开源本地规划器插件来说，5-15 秒的衰减时间都比较合适。

6.底层驱动与软件支持
joystick_drivers：遥控器驱动，用于支持各种类型的手柄输入，并从底层操作系统信息中生成 ROS 信息。
livox、rslidar_sdk、rs_to_velodyne：激光雷达驱动以及消息转换。
serial、yesense_ros_driver：IMU驱动。