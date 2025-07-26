import csv
import matplotlib
matplotlib.use('TkAgg')  # Pastikan backend GUI stabil di Windows

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import os

csv_file = "./wp_2_jalur.csv"
# csv_file = "./wp_jalur_tunggal.csv"

def read_csv(file_path):
    x_vals, y_vals = [], []
    try:
        with open(file_path, mode='r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                x_vals.append(float(row["x"]))
                y_vals.append(float(row["y"]))
    except Exception as e:
        print(f"Error reading CSV: {e}")
    return x_vals, y_vals

# Setup plot
fig, ax = plt.subplots(figsize=(10, 10))
line, = ax.plot([], [], 'bo-', label='Trajectory')
ax.set_title("Real-Time 2D Trajectory from x,y")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.axis("equal")
ax.grid(True)
ax.legend()

# Fungsi update otomatis
def update(frame):
    x_vals, y_vals = read_csv(csv_file)
    line.set_data(x_vals, y_vals)
    ax.relim()
    ax.autoscale_view()
    return line,

# Jalankan update tiap 500ms (0.5 detik)
ani = FuncAnimation(fig, update, interval=500)

# Tampilkan GUI plot yang interaktif
plt.show()
