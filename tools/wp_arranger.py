import csv
import math
import numpy as np

def load_waypoints(path):
    with open(path, mode='r') as f:
        reader = csv.DictReader(f)
        return [(float(row['x']), float(row['y'])) for row in reader]

def angle_between(v1, v2):
    v1_u = v1 / (np.linalg.norm(v1) + 1e-8)
    v2_u = v2 / (np.linalg.norm(v2) + 1e-8)
    dot = np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)
    return np.arccos(dot)  # radians

def arrange_directionally(waypoints, start_pos, start_angle_deg, angle_threshold_deg=90, dist_threshold=10.0):
    remaining = waypoints.copy()
    arranged = []
    current_pos = np.array(start_pos)
    current_dir = np.array([math.cos(math.radians(start_angle_deg)),
                            math.sin(math.radians(start_angle_deg))])

    while remaining:
        # Find candidate points within angle threshold
        best_idx = None
        best_dist = float('inf')
        for i, wp in enumerate(remaining):
            vec = np.array(wp) - current_pos
            if np.linalg.norm(vec) == 0:
                continue
            angle = math.degrees(angle_between(current_dir, vec))
            dist = np.linalg.norm(vec)

            if angle < angle_threshold_deg and dist < best_dist and dist < dist_threshold:
                best_dist = dist
                best_idx = i

        if best_idx is None:
            # No directionally valid point — relax constraint or break
            print("No valid waypoint found within angle threshold. Stopping.")
            break

        next_wp = remaining.pop(best_idx)
        arranged.append(next_wp)

        # Update direction and position
        next_pos = np.array(next_wp)
        current_dir = next_pos - current_pos
        current_pos = next_pos

    return arranged



import matplotlib.pyplot as plt

def visualize_waypoints(waypoints, start_pos=None, show_index=True, title="Waypoints Path"):
    # Add start position to the front if given
    if start_pos:
        waypoints = [start_pos] + waypoints

    xs = [p[0] for p in waypoints]
    ys = [p[1] for p in waypoints]

    plt.figure(figsize=(15, 15))
    plt.plot(xs, ys, 'o-', label="Path")  # lines and dots

    # Draw arrows for each segment starting at each point
    for i in range(len(waypoints) - 1):
        x0, y0 = waypoints[i]
        x1, y1 = waypoints[i + 1]
        dx = x1 - x0
        dy = y1 - y0
        plt.arrow(x0, y0, dx, dy,
                  head_width=0.1, head_length=0.15, fc='r', ec='r', length_includes_head=True)

    if show_index:
        for i, (x, y) in enumerate(waypoints):
            plt.text(x, y + 0.15, str(i), fontsize=9, ha='center')

    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.grid(True)
    plt.axis("equal")
    plt.title(title)
    plt.legend()
    plt.show()

waypoints = load_waypoints("waypoint22.csv")

start = (7.52400016784668,0.01889999397099018)
heading = 90.0 

ordered = arrange_directionally(waypoints, start, heading)
# visualize_waypoints(ordered, start_pos=start)


# # Save to CSV
with open("waypoint222.csv", mode='w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["x", "y", "fb_velocity", "fb_steering"])
    for p in ordered:
        writer.writerow([p[0], p[1], 0.5, 0.0])

print("✅ Waypoints saved to waypoint_ordered.csv")
