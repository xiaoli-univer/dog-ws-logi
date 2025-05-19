import matplotlib.pyplot as plt

# 读取数据
data = []
with open("/home/nianfei/work_dict/dog_ws/src/mpic/src/data.txt", "r") as file:
    for line in file:
        values = line.strip().split()
        data.append([float(value) for value in values])

# 提取列数据
i = [row[0] for row in data]
vref = [row[1] for row in data]
vout = [row[2] for row in data]
f = [row[3] for row in data]

# 绘制曲线图
plt.plot(i, vref, label="vref")
plt.plot(i, vout, label="vout")
plt.plot(i, f, label="f")

# 设置标题和轴标签
plt.title("vref and vout")
plt.xlabel("i")
plt.ylabel("Value")

# 显示图例
plt.legend()

# 保存图像文件
plt.savefig("/home/nianfei/work_dict/dog_ws/src/mpic/src/curve_f.png")

# 显示图像
plt.show()
