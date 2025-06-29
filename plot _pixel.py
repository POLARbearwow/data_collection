import pandas as pd
import matplotlib.pyplot as plt
import cv2  # 仅用来读取图像尺寸，可用任意帧文件或手动填

# === 1. 读取 CSV ===
csv_path = '/home/niu/Desktop/data_collection/records/pixels_20250626_231913.csv'       # 修改为实际文件
df = pd.read_csv(csv_path)

# 保留有效行
ball_ok  = df[['ball_u','ball_v']].ne(-999).all(axis=1)
aruco_ok = df[['aruco_u','aruco_v']].ne(-999).all(axis=1)

# === 2. 获取图像尺寸 H ===
# 若身边有一帧图片可直接读取；否则手动填写
img_example = cv2.imread('sample.jpg')        # 或 camera 帧
H = img_example.shape[0] if img_example is not None else 1080

# === 3. 坐标变换 ===
df.loc[ball_ok,  'ball_x']  =  df.loc[ball_ok,  'ball_u']
df.loc[ball_ok,  'ball_y']  =  H - df.loc[ball_ok,  'ball_v']
df.loc[aruco_ok, 'aruco_x'] =  df.loc[aruco_ok, 'aruco_u']
df.loc[aruco_ok, 'aruco_y'] =  H - df.loc[aruco_ok, 'aruco_v']

# === 4. 绘制 ===
plt.figure(figsize=(10,6))
plt.scatter(df.loc[ball_ok,'ball_x'],
            df.loc[ball_ok,'ball_y'],
            s=10,c='red', label='Ball Center')
plt.scatter(df.loc[aruco_ok,'aruco_x'],
            df.loc[aruco_ok,'aruco_y'],
            s=10,c='blue',label='ArUco Center')

plt.gca().set_aspect('equal')
plt.xlabel('x (pixels, origin at left-bottom)')
plt.ylabel('y (pixels, origin at left-bottom)')
plt.title('Pixel Trajectory')
plt.legend()
plt.grid(True)
plt.show()