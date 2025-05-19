import pickle
import pprint
import time
from datetime import date

import matplotlib.pyplot as plt

plt.style.use("seaborn-poster")


def figure(fn="06_21_49"):
    with open(
        r"/home/sdu/dos_ws/src/chassis_dog/doc/" + "vel_" + fn + ".pkl", "rb"
    ) as f:
        vel = pickle.load(f)
        # pprint.pprint(vel)
    with open(
        r"/home/sdu/dos_ws/src/chassis_dog/doc/" + "force_" + fn + ".pkl", "rb"
    ) as f:
        force = pickle.load(f)
        # pprint.pprint(force)
    # print(vel[:117] + vel[170:])

    for i, v in enumerate(vel):
        vel[i] = v * 10

    x_index = range(len(vel))
    plt.clf()
    plt.plot(x_index, vel, ".-", label="vel(m/s) ")
    plt.plot(x_index, force, ".-", label="press(N)")
    plt.title("sensor reading")
    plt.xlabel("T")
    plt.ylabel("results")
    plt.xticks(rotation=45)
    plt.legend()
    plt.grid(True)
    # plt.ylim((-20, 20))
    plt.ioff()
    # plt.pause(0.1)
    plt.show()


def figuresim(fn="22_17_02"):
    with open(
        r"/home/sdu/dos_ws/src/chassis_dog/doc/" + "vel_sim_" + fn + ".pkl", "rb"
    ) as f:
        vel = pickle.load(f)
        # pprint.pprint(vel)
    with open(
        r"/home/sdu/dos_ws/src/chassis_dog/doc/" + "force_sim_" + fn + ".pkl", "rb"
    ) as f:
        force = pickle.load(f)
        # pprint.pprint(force)
    # print(vel[:117] + vel[170:])

    for i, v in enumerate(vel):
        vel[i] = v * 10

    x_index = range(len(vel))
    plt.clf()
    plt.plot(x_index, vel, ".-", label="vel value")
    plt.plot(x_index, force, ".-", label="press value")
    plt.title("sensor reading")
    plt.xlabel("T")
    plt.ylabel("results")
    plt.xticks(rotation=45)
    plt.legend()
    plt.grid(True)
    # plt.ylim((-20, 20))
    plt.ioff()
    # plt.pause(0.1)
    plt.show()


def figurecom():
    with open(r"/home/sdu/dos_ws/src/chassis_dog/doc/" + "vel_22_16_44.pkl", "rb") as f:
        velt = pickle.load(f)
    with open(
        r"/home/sdu/dos_ws/src/chassis_dog/doc/" + "force_22_16_44.pkl", "rb"
    ) as f:
        forcet = pickle.load(f)
    with open(
        r"/home/sdu/dos_ws/src/chassis_dog/doc/" + "force_sim_22_17_02.pkl", "rb"
    ) as f:
        forcesim = pickle.load(f)
    # for i, v in enumerate(vel):
    #     vel[i] = v * 10
    vel1 = velt[240:350]
    force1 = forcet[240:350]
    force2 = forcesim[20:130]

    x_index = range(len(vel1))
    fig = plt.figure()
    ax = plt.subplot(211)
    av1 = sum(vel1) / len(vel1)
    print(av1)
    s = 0
    for i, v in enumerate(vel1):
        if v > 0.3:
            s += 1
    print(1 - s / len(vel1), len(vel1))
    plt.plot(x_index, vel1, label="$\\dot{x_r}$")
    plt.plot(x_index, [0.3] * len(vel1), label="$\\dot{x_d}$")
    # plt.plot(vel1, force1)

    ax2 = plt.subplot(212)
    av1 = sum(force1) / len(force1)
    av2 = sum(force2) / len(force2)
    df = []
    for i in range(len(force1) - 1):
        df.append(force1[i] - force2[i])
    print(av1, av2, sum(df) / len(df))
    plt.plot(x_index, force1, label="$F_{e1}$", color="cornflowerblue")
    plt.plot(x_index, force2, label="$F_{e2}}$", color="yellowgreen")
    plt.plot(
        x_index,
        [av1] * len(force1),
        color="cornflowerblue",
        linestyle="--",
        label="$\\overline{F}_{e1}$",
    )
    plt.plot(
        x_index,
        [av2] * len(force2),
        color="yellowgreen",
        linestyle="--",
        label="$\\overline{F}_{e2}$",
    )
    ax.set_ylabel("velocity(m/s)", fontsize=25)
    ax.legend(fontsize=23, loc="lower right")  # 设置图表图例在左上角
    ax.grid(True, alpha=0.2)  # 绘制网格dodgerblue
    ax2.set_ylabel("$Force(N)$", fontsize=25)
    ax2.legend(fontsize=23, loc="lower right")  # 设置图表图例在左上角
    ax2.grid(True, alpha=0.2)  # 绘制网格
    ax.tick_params(axis="both", labelsize=23)
    ax2.tick_params(axis="both", labelsize=23)
    plt.xlabel("Time(s)", fontsize=25)
    # plt.subplots_adjust(top=1, bottom=0, right=1, left=0, hspace=0.2, wspace=0.2)
    # plt.margins(0, 0)
    # plt.savefig(r"C:\Users\Nianf\OneDrive\文档\exp1.pdf")
    plt.show()
    # plt.clf()
    # plt.subplot(211)
    # plt.plot(x_index, vel1, label="vel")
    # plt.plot(x_index, [0.3] * len(vel1),  label="vel")
    # plt.subplot(212)
    # plt.plot(x_index, force1,  label="force")
    # plt.plot(x_index, force2,  label="force without control")
    # plt.title("sensor reading")
    # plt.xlabel("T")
    # plt.ylabel("results")
    # plt.xticks(rotation=45)
    # plt.legend()
    # plt.grid(True)
    # # plt.ylim((-20, 20))
    # plt.ioff()
    # # plt.pause(0.1)
    # plt.show()


if __name__ == "__main__":
    figure()
    # figuresim("22_17_02")  # no control 0.3 x=:150
    # figure("22_16_44")  # 0.3 x=200:
    # figure("22_16_50")  # 0.5

    exit()
    # fn = "vel_22_15_14.pkl"
    # fn2 = "force_22_15_14.pkl"
    fn = "vel_sim_22_17_02.pkl"
    fn2 = "force_sim_22_17_02.pkl"
    # fn2="force"+time.strftime("_%d_%H_%M", time.localtime())+".pkl"

    with open(r"/home/sdu/dos_ws/src/chassis_dog/doc/" + fn, "rb") as f:
        vel = pickle.load(f)
        pprint.pprint(vel)
    with open(r"/home/sdu/dos_ws/src/chassis_dog/doc/" + fn2, "rb") as f:
        force = pickle.load(f)
        pprint.pprint(force)
    print(vel[:117] + vel[170:])

    for i, v in enumerate(vel):
        vel[i] = v * 10

    x_index = range(vel)
    plt.clf()
    plt.plot(x_index, vel, ".-", label="vel value")
    plt.plot(x_index, force, ".-", label="press value")
    plt.title("sensor reading")
    plt.xlabel("T")
    plt.ylabel("results")
    plt.xticks(rotation=45)
    plt.legend()
    plt.grid(True)
    # plt.ylim((-20, 20))
    plt.ioff()
    # plt.pause(0.1)
    plt.show()
