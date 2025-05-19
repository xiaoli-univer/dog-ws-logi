import ast
import os
from time import sleep, time

import matplotlib as mpl
import matplotlib.pyplot as plt
import modbus_tk.defines as cst
import serial
from modbus_tk import modbus_rtu


def comp2true(comp):
    sign = comp >> 31
    if sign:
        hex_str = hex(comp)
        while len(hex_str) - 2 < 8:
            hex_str = hex_str.replace("0x", "0xF")
        comp = ast.literal_eval(hex_str)
        return -((comp - 1) ^ 0xFFFFFFFF)
    else:
        return comp


def modbus_init(PORT="/dev/ttyUSB0"):
    # os.system('echo %s | sudo -S %s' % (password, command))
    os.system("sudo chmod 777 " + PORT)
    master = modbus_rtu.RtuMaster(
        serial.Serial(port=PORT, baudrate=9600, bytesize=8, parity="N", stopbits=1)
    )
    master.set_timeout(1.0)
    master.set_verbose(True)

    return master


def read_once(master):
    high, low = master.execute(
        slave=1,
        function_code=cst.READ_HOLDING_REGISTERS,
        starting_address=6,
        quantity_of_x=2,
    )

    comp = int(hex(high) + hex(low)[2:], 16)
    output = comp2true(comp)
    output = -500 if output > 100000 else output
    return output


def get_init_force(master):
    init = []
    for item in range(100):
        init.append(read_once(master))
    return (sum(sorted(init)[5:-5]) / len(init[5:-5])) * 10000 / (2**24 - 1)


def get_filt_out(master):
    buffer = []
    for _ in range(12):
        buffer.append(read_once(master))
    return (sum(sorted(buffer)[1:-1]) / len(buffer[1:-1])) * 10000 / (2**24 - 1)


def plot_conf():
    plt.rcParams["font.sans-serif"] = ["SimHei"]  # 设置字体，不然中文无法显示
    mpl.rcParams["axes.unicode_minus"] = False  # 用来正常显示负数  "-"
    plt.rcParams["figure.figsize"] = (8.0, 4.0)  # 设置figure_size尺寸
    # figsize(12.5, 4) # 设置 figsize
    plt.rcParams["savefig.dpi"] = 300  # 保存图片分辨率
    plt.rcParams["figure.dpi"] = 300  # 分辨率
    # 默认的像素：[6.0,4.0]，分辨率为100，图片尺寸为 600&400
    # 指定dpi=200，图片尺寸为 1200*800
    # 指定dpi=300，图片尺寸为 1800*1200
    plt.rcParams["image.interpolation"] = "nearest"  # 设置 interpolation style
    plt.rcParams["image.cmap"] = "gray"  # 设置 颜色 style
    plt.savefig("plot1.png", dpi=300)  # 指定分辨率保存


if __name__ == "__main__":
    results = []
    x_index = []
    i = 0
    plt.figure()
    plt.ion()
    plt.style.use("ggplot")  # 给图片换不同的风格
    modbus_master = modbus_init()

    init_force = get_init_force(modbus_master)
    print(f"get force offset:{init_force}")
    # while True:
    #     buffer = []

    #     print(get_filt_out(modbus_master))
    #     # print(buffer)
    #     # print(sum(sorted(buffer)[1:-1]) / len(buffer[1:-1]))

    while True:
        force = get_filt_out(modbus_master)
        output = force - init_force
        print(force, output)
        results.append(output)
        x_index.append(i)
        plt.clf()
        plt.plot(x_index, results, ".-", label="press value")
        plt.title("sensor reading")
        plt.xlabel("T")
        plt.ylabel("results")
        plt.xticks(rotation=45)
        plt.legend()
        plt.grid(True)
        # plt.ylim((-20, 20))
        plt.pause(0.1)
        plt.ioff()

        i += 1
        sleep(0.1)
