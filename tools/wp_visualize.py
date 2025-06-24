import csv
import matplotlib.pyplot as plt

# Load x, y data from the CSV
x_vals = []
y_vals = []

with open("waypoint22.csv", mode='r') as file:
    reader = csv.DictReader(file)
    for row in reader:
        x_vals.append(float(row["x"]))
        y_vals.append(float(row["y"]))

# Plot the trajectory
plt.figure(figsize=(10, 8))
plt.plot(x_vals, y_vals, 'bo-', label='Trajectory')
plt.title("2D Trajectory from x,y")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.show()



'''
6.9 2.6 stop 
6.4 16.4 belokan
-12.8 25.0 lurus
-16.6 41.3 belokan
-48.6 42.4 lurus
-97.2 39.8 belokan
-110 45.2 belokan
-191.5 45.5 stop 2
-127.5 46.6 lurus
-182 40.4 belokan
-80.5 40.1 lurus
-14.6 -11.9 belokan
'''
