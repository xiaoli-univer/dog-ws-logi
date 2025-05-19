import _thread
import pickle
import sys
import threading
import time
from pprint import pformat, pprint
from time import sleep

import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

sys.path.append("/home/nianfei/dos_ws/src/chassis_dog/scripts")
from pymodbus import get_filt_out, get_init_force, modbus_init

# from scipy.interpolate import make_interp_spline as spline


class force_control:
    def __init__(self, md=10.0, bd=200.0, kd=1200.0, t=0.005):
        self.md = md
        self.bd = bd
        self.kd = kd
        self.t = t
        self.xr = 0  # 预期的机器人位置
        self.xr_pre = 0
        self.xr_dot_pre = 0

    def imp_ctrl(self, xd, xd_dot, xd_ddot, f):
        # 上一采样时间 机器人末端的实际运动加速度
        self.xr_ddot_pre = xd_ddot + 1 / self.md * (
            f + self.bd * (xd_dot - self.xr_dot_pre) + self.kd * (xd - self.xr_pre)
        )
        self.xr_dot = self.xr_ddot_pre * self.t + self.xr_dot_pre
        self.xr = self.xr_pre + self.xr_dot * self.t
        # for interation
        self.xr_dot_pre = self.xr_dot
        self.xr_pre = self.xr
        # 限幅至[-10,10]并映射到[-1,1]内
        # self.xr_dot = 10 if self.xr_dot > 10 else self.xr_dot
        # self.xr_dot = -10 if self.xr_dot < -10 else self.xr_dot
        # self.xr_dot = -1 + 2 * (self.xr_dot + 1) / 20

        return self.xr, self.xr_dot

    def send_vel_smooth(self, vel_now, vel_goal, time):
        # TODO:https://blog.csdn.net/weixin_42782150/article/details/107176500
        pass
        # x = np.linspace(0, time, 10)
        # y = np.linspace(vel_now, vel_goal, 10)
        # y_smooth = spline(x, y)(x)

    def position_ctrl(self, goal_pos):
        rospy.init_node("force_control_odom")
        odom_msg = rospy.wait_for_message("/odom", Odometry, None)
        # listen_tf = tf.TransformListener()
        # while not rospy.is_shutdown():
        #     try:
        #         trans, rot = listen_tf.lookupTransform(
        #             "base_link", "odom", rospy.Time(0.5)
        #         )
        #     except (
        #         tf.LookupException,
        #         tf.ConnectivityException,
        #         tf.ExtrapolationException,
        #     ):
        #         print("failed to get transform from odom to base_link,try again")
        #         continue
        # print(trans, rot)
        pprint(odom_msg.pose.pose.position)
        pprint(odom_msg.pose.pose.orientation)


def vel_pub(pub, vel, t=0.05):
    pub_time = 0
    vel = 2 if vel > 2 else vel
    vel = -2 if vel < -2 else vel
    while not rospy.is_shutdown() and pub_time < 0.5:
        msg = Twist()
        # max vel:0.75
        msg.linear.x = vel
        msg.angular.z = 0
        pub.publish(msg)
        rospy.sleep(t / 100)
        pub_time += t / 100


def vel_pub_triger(pub, vel):
    global out_vel
    global stop_pub
    vel = out_vel
    vel = 1 if vel > 1 else vel
    vel = -1 if vel < -1 else vel
    msg = Twist()
    # max vel:0.75
    msg.linear.x = out_vel
    data = rospy.wait_for_message("/exp_vels", Twist, 3)
    msg.angular.z = data.angular.z

    pub.publish(msg)
    if not stop_pub:
        threading.Timer(0.01, vel_pub_triger, (pub, out_vel)).start()
    else:
        sys.exit()


def test_vel(pub):
    for i in range(10):
        vel_pub(pub, i * 0.02, 0.5)
        sleep(0.3)
        vel_pub(pub, -i * 0.04, 0.5)
        sleep(0.3)

    # vel_pub(pub, -0.2)


def figure():
    x_index.append(i)
    plt.clf()
    plt.plot(x_index, vel_log, ".-", label="vel value")
    plt.plot(x_index, force_log, label="press value")
    plt.title("sensor reading")
    plt.xlabel("T")
    plt.ylabel("results")
    plt.xticks(rotation=45)
    plt.legend()
    plt.grid(True)
    plt.ylim((-20, 20))
    plt.pause(0.1)
    plt.ioff()


def callback(data):
    global exp_vel
    exp_vel = data.linear.x
    rospy.loginfo("Received velocity command:")
    rospy.loginfo(
        "Linear Components: [%s, %s, %s]"
        % (data.linear.x, data.linear.y, data.linear.z)
    )
    rospy.loginfo(
        "Angular Components: [%s, %s, %s]"
        % (data.angular.x, data.angular.y, data.angular.z)
    )


if __name__ == "__main__":
    # md=10, bd=200, kd=1200, t=0.05
    # Bd:减小容易振荡，增大会减小力变化的作用
    # md=10, bd=250, kd=180, t=t
    vel_log = []
    force_log = []
    exp_vel_log = []
    x_index = []
    t = 0.01

    f_ctrl = force_control(md=1, bd=35, kd=0.0, t=t)
    # f_ctrl = force_control(md=10, bd=35, kd=0.0, t=t)
    # f_ctrl = force_control(md=10, bd=250, kd=180, t=t)

    rospy.init_node("force_control")

    modbus_master = modbus_init()
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    data = rospy.wait_for_message("/exp_vels", Twist, 3)
    exp_vel = data.linear.x
    out_vel = 0
    rospy.sleep(3)
    init_force = get_init_force(modbus_master)
    print(f"get force offset:{init_force}")

    stop_pub = False
    vel_pub_triger(pub, out_vel)

    f_b = 0
    i = -1
    curtime = time.strftime("_%d_%H_%M", time.localtime())
    fn = "vel" + curtime + ".pkl"
    fn2 = "force" + curtime + ".pkl"
    fn3 = "exp_vel" + curtime + ".pkl"
    while True:
        i += 1
        data = rospy.wait_for_message("/exp_vels", Twist, 3)
        exp_vel = data.linear.x
        force = get_filt_out(modbus_master) - init_force
        force = -force * 7
        if force - f_b > 50:
            force = f_b
        f_b = force
        out_pos, out_vel = f_ctrl.imp_ctrl(exp_vel * i * t, exp_vel, 0, force)
        out_vel = round(out_vel, 4)
        out_vel = 0 if out_vel < 0 else out_vel

        print(
            f"force:{force:.4f}",
            f"exp_pos:{i*t*exp_vel:.2f}",
            f"exp_vel:{exp_vel:.2f}",
            f"out_pos:{out_pos:.2f}",
            f"out_vel:{out_vel}",
        )
        vel_log.append(out_vel)
        force_log.append(force)
        exp_vel_log.append(exp_vel_log)
        sleep(t)
        with open(r"/home/sdu/dos_ws/src/chassis_dog/doc/" + fn, "wb") as f:
            pickle.dump(vel_log, f)

        with open(r"/home/sdu/dos_ws/src/chassis_dog/doc/" + fn2, "wb") as f:
            pickle.dump(force_log, f)

        with open(r"/home/sdu/dos_ws/src/chassis_dog/doc/" + fn3, "wb") as f:
            pickle.dump(exp_vel_log, f)

    stop_pub = True
    with open(r"/home/sdu/dos_ws/src/chassis_dog/doc/log.txt", "w+") as f:
        f.write(pformat(vel_log) + "\n" + pformat(force_log))
