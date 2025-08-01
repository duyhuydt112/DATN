import serial
import matplotlib.pyplot as plt
import time

# 🛠️ Thay cổng COM dưới đây bằng cổng của ESP32 bạn
PORT = 'COM9'          # ⚠️ THAY BẰNG COM CỦA BẠN
BAUD = 115200

# Kết nối serial
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)  # Đợi ESP32 khởi động

# Các danh sách lưu dữ liệu
x_vals = []
err_vals = []
pid_vals = []
pitch_vals = []

plt.ion()  # Bật chế độ tương tác
fig, ax = plt.subplots()

print("⏳ Đang nhận dữ liệu từ ESP32... Nhấn Ctrl+C để dừng.")

while True:
    try:
        line = ser.readline().decode('utf-8').strip()
        if not line:
            continue

        # Tách dữ liệu theo dấu phẩy
        parts = line.split(",")
        if len(parts) < 3:
            continue

        # Chuyển sang float
        err = float(parts[0])
        pid = float(parts[1])
        pitch = float(parts[2])

        # Cập nhật dữ liệu
        x_vals.append(len(x_vals))
        err_vals.append(err)
        pid_vals.append(pid)
        pitch_vals.append(pitch)

        # Giới hạn độ dài dữ liệu
        if len(x_vals) > 200:
            x_vals.pop(0)
            err_vals.pop(0)
            pid_vals.pop(0)
            pitch_vals.pop(0)

        # Cập nhật biểu đồ
        ax.clear()
        ax.plot(x_vals, err_vals, label="Error")
        ax.plot(x_vals, pid_vals, label="PID Output")
        ax.plot(x_vals, pitch_vals, label="Pitch (deg)")
        ax.set_title("PID Gimbal Visualization")
        ax.set_xlabel("Time (samples)")
        ax.set_ylabel("Values")
        ax.legend()
        ax.grid(True)
        plt.pause(0.01)

    except KeyboardInterrupt:
        print("\n🛑 Dừng nhận dữ liệu.")
        break

ser.close()
