# odom
```bash
rosrun message_transformer message_transformer.py
roslaunch receive_robot_status receive_robot_status.launch
```
# IMU
TODO:空白
roslaunch hdl_localization local_rslidar_imu.launch

```bash
gnome-terminal -x bash -c "source ~/.bashrc; roscore; exec bash;"
sleep 5
gnome-terminal -t "msg_trans" -x bash -c "source devel/setup.bash; rosrun message_transformer message_transformer.py; exec bash;"
sleep 2
gnome-terminal -t "navigation" -x bash -c "sudo chmod 777 /dev/ttyUSB0; source devel/setup.bash; roslaunch navigation.launch; exec bash;"
```
