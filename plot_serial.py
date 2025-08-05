import serial
import matplotlib.pyplot as plt
from collections import deque

# ⚠️ Cập nhật đúng cổng COM trên máy của bạn
PORT = 'COM9'       # ← Đổi đúng port Serial (VD: COM3, COM4, v.v.)
BAUD = 115200

# Thử kết nối Serial
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"✅ Đã kết nối {PORT} @ {BAUD}")
except:
    print(f"❌ Không mở được cổng {PORT}")
    exit(1)

# Cấu hình vùng đệm (số điểm hiển thị)
MAX_POINTS = 200
pid_data  = deque(maxlen=MAX_POINTS)
err_data = deque(maxlen=MAX_POINTS)
yaw_data = deque(maxlen=MAX_POINTS)

# Cấu hình biểu đồ
plt.ion()
fig, ax = plt.subplots()
line_pid,  = ax.plot([], [], label='PID output', color='blue')
line_err, = ax.plot([], [], label='Error (err2)', color='red')
line_yaw,  = ax.plot([], [], label='Yaw (deg)', color='green')

ax.set_xlim(0, MAX_POINTS)
ax.set_ylim(-100, 100)  # ← Chỉnh giới hạn trục y nếu cần
ax.set_title("Biểu đồ phản ứng PID theo thời gian thực")
ax.set_xlabel("Thời gian (tick)")
ax.set_ylabel("Giá trị")
ax.legend(loc='upper right')
plt.grid(True)

while True:
    try:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue

        parts = line.split(',')
        if len(parts) != 4:
            continue

        # Parse dữ liệu
        rel, err, pid, yaw = map(float, parts)

        # Lưu vào vùng đệm
        pid_data.append(pid)
        err_data.append(err)
        yaw_data.append(yaw)

        # Cập nhật dữ liệu vẽ
        x = list(range(len(pid_data)))
        line_pid.set_data(x, pid_data)
        line_err.set_data(x, err_data)
        line_yaw.set_data(x, yaw_data)

        ax.set_xlim(0, MAX_POINTS)
        ax.relim()
        ax.autoscale_view(True, True, True)

        plt.pause(0.01)

    except KeyboardInterrupt:
        print("\n🛑 Dừng vẽ.")
        break

    except Exception as e:
        print("Lỗi:", e)
