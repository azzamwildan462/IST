import csv
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt

# Step 1: Load original data
x_vals, y_vals = [], []

with open("waypoint_copy.csv", mode='r') as file:
    reader = csv.DictReader(file)
    for row in reader:
        x_vals.append(float(row["x"]))
        y_vals.append(float(row["y"]))

# Step 2: Apply smoothing
window_size = 5  # must be odd and <= len(data)
poly_order = 2
x_smooth = savgol_filter(x_vals, window_size, poly_order)
y_smooth = savgol_filter(y_vals, window_size, poly_order)

# # Step 3: Print smoothed points
# print("Smoothed Points:")
# for x, y in zip(x_smooth, y_smooth):
#     print(f"{x:.5f}, {y:.5f}")

# Step 4: (Optional) Save to CSV
with open("smoothed_waypoints2.csv", mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["x", "y"])
    for x, y in zip(x_smooth, y_smooth):
        writer.writerow([x, y])

# Step 5: Plot for visualization
plt.figure(figsize=(10, 8))
plt.plot(x_vals, y_vals, 'bo--', label='Original')
plt.plot(x_smooth, y_smooth, 'r-', linewidth=2, label='Smoothed')
plt.title("Trajectory Smoothing")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.show()
