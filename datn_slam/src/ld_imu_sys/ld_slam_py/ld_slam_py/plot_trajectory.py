import pandas as pd
import matplotlib.pyplot as plt

# Đọc dữ liệu
df = pd.read_csv('robot_trajectory.csv')

plt.figure(figsize=(10, 8))

# Vẽ quỹ đạo Ground Truth (Hình chữ nhật 1.65x1.65 mét mô phỏng)
plt.plot([0, 1.65, 1.65, 0, 0], [0, 0, -1.65, -1.65, 0], 'k--', linewidth=2, label='Ground Truth (Băng keo 1.65x1.65m)')

# Vẽ quỹ đạo EKF (Odom)
#plt.plot(df['EKF_X'], df['EKF_Y'], 'b-', linewidth=1.5, alpha=0.7, label='Quỹ đạo EKF (odom)')

# Vẽ quỹ đạo SLAM (Map)
plt.plot(df['SLAM_X'], df['SLAM_Y'], 'r-', linewidth=1.5, label='Quỹ đạo SLAM (map)')

plt.title('Đánh giá Quỹ đạo Di chuyển của Robot')
plt.xlabel('Trục X (mét)')
plt.ylabel('Trục Y (mét)')
plt.legend()
plt.grid(True)
plt.axis('equal') # Giữ tỉ lệ X-Y chuẩn xác
plt.show()
