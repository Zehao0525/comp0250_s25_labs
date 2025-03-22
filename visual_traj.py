import pandas as pd

# 读取用户上传的轨迹文件
file_path = "trajectory.csv"
df = pd.read_csv(file_path)

# 显示前几行数据和列名供检查
df.head(), df.columns.tolist()

import matplotlib.pyplot as plt

# 提取时间和主机械臂关节数据
time = df["time_from_start"].to_numpy()
joint_names = [col for col in df.columns if col.startswith("panda_joint")]
plt.figure(figsize=(12, 6))

for joint in joint_names:
    plt.plot(time, df[joint].to_numpy(), label=joint)

plt.xlabel("Time from start (s)")
plt.ylabel("Joint Position (rad)")
plt.title("Franka Emika Panda Arm - Joint Trajectory")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
