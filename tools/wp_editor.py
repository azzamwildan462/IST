import pandas as pd
import matplotlib.pyplot as plt

# Load waypoints
# filename = "wp_2_jalurbenar_1.csv"
filename = "wp_bagus.csv"
df = pd.read_csv(filename)
undo_stack = []

# Initial plot
fig, ax = plt.subplots()
ax.set_aspect('equal', adjustable='datalim')  # Equal scale on both axes
sc = ax.scatter(df["x"].to_numpy(), df["y"].to_numpy(), color='blue', s=100)
line, = ax.plot(df["x"].to_numpy(), df["y"].to_numpy(), linestyle='dotted', color='gray')

# App state
edit_mode = 'normal'  # 'normal', 'group', 'move'
selected_idxs = set()
selected_idx_single = None
drag_start_pos = None
rect_artist = None
rect_start = None

# --- Update Plot ---
def update_plot():
    colors = ['red' if i in selected_idxs else 'blue' for i in range(len(df))]
    sc.set_offsets(df[["x", "y"]].to_numpy())
    sc.set_color(colors)
    line.set_data(df["x"].to_numpy(), df["y"].to_numpy())
    ax.set_title(f"Mode: {edit_mode.upper()} • Alt+G: group • Alt+M: move • Alt+N: normal • Alt+S: save • Alt+Z: undo")
    fig.canvas.draw_idle()

# --- Undo ---
def undo_last_move():
    global df
    if undo_stack:
        df[["x", "y"]] = undo_stack.pop()
        print("↩️ Undo complete")
        update_plot()
    else:
        print("⚠️ Nothing to undo")

# --- Key Handler ---
def on_key(event):
    global edit_mode
    key = event.key.lower()

    if key == 'alt+s':
        df.to_csv(filename, index=False)
        print("💾 Saved to ", filename)

    elif key == 'alt+z':
        undo_last_move()

    elif key == 'alt+g':
        if edit_mode != 'group':
            edit_mode = 'group'
            print("🟡 Group select mode enabled")
        update_plot()

    elif key == 'alt+m':
        if edit_mode != 'move':
            edit_mode = 'move'
            print("🟣 Move group mode enabled")
        update_plot()

    elif key == 'alt+n':
        if edit_mode != 'normal':
            edit_mode = 'normal'
            print("⚪ Normal mode")
        update_plot()

# --- Mouse Press ---
def on_press(event):
    global drag_start_pos, selected_idx_single, rect_start, rect_artist

    if event.xdata is None or event.ydata is None:
        return

    drag_start_pos = (event.xdata, event.ydata)

    if edit_mode == 'normal':
        distances = ((df["x"] - event.xdata)**2 + (df["y"] - event.ydata)**2).pow(0.5)
        closest = distances.idxmin()
        if distances[closest] < 0.5:
            selected_idx_single = closest
            undo_stack.append(df[["x", "y"]].copy(deep=True))
            print(f"🎯 Selected point {closest}")
    elif edit_mode == 'move' and selected_idxs:
        undo_stack.append(df[["x", "y"]].copy(deep=True))
    elif edit_mode == 'group':
        rect_start = (event.xdata, event.ydata)
        if rect_artist:
            rect_artist.remove()
            rect_artist = None
        rect_artist = plt.Rectangle(rect_start, 0, 0, linewidth=1, edgecolor='green', facecolor='none', linestyle='--')
        ax.add_patch(rect_artist)

# --- Mouse Drag ---
def on_motion(event):
    global drag_start_pos, rect_artist

    if event.xdata is None or event.ydata is None or drag_start_pos is None:
        return

    dx = event.xdata - drag_start_pos[0]
    dy = event.ydata - drag_start_pos[1]

    if edit_mode == 'normal' and selected_idx_single is not None:
        df.at[selected_idx_single, "x"] += dx
        df.at[selected_idx_single, "y"] += dy
        drag_start_pos = (event.xdata, event.ydata)
        update_plot()

    elif edit_mode == 'move' and selected_idxs:
        for i in selected_idxs:
            df.at[i, "x"] += dx
            df.at[i, "y"] += dy
        drag_start_pos = (event.xdata, event.ydata)
        update_plot()

    elif edit_mode == 'group' and rect_artist:
        x0, y0 = rect_start
        rect_artist.set_width(event.xdata - x0)
        rect_artist.set_height(event.ydata - y0)
        update_plot()

# --- Mouse Release ---
def on_release(event):
    global drag_start_pos, selected_idx_single, rect_artist, rect_start, selected_idxs

    if edit_mode == 'group' and rect_start and event.xdata and event.ydata:
        x0, y0 = rect_start
        x1, y1 = event.xdata, event.ydata
        x_min, x_max = sorted([x0, x1])
        y_min, y_max = sorted([y0, y1])

        selected_idxs = {
            i for i, (x, y) in enumerate(zip(df["x"], df["y"]))
            if x_min <= x <= x_max and y_min <= y <= y_max
        }

        if selected_idxs:
            print(f"📦 Selected {len(selected_idxs)} points")
        else:
            print("⚠️ No points selected")

        if rect_artist:
            rect_artist.remove()
            rect_artist = None
        rect_start = None
        update_plot()

    drag_start_pos = None
    selected_idx_single = None

# --- Connect Events ---
fig.canvas.mpl_connect('key_press_event', on_key)
fig.canvas.mpl_connect('button_press_event', on_press)
fig.canvas.mpl_connect('motion_notify_event', on_motion)
fig.canvas.mpl_connect('button_release_event', on_release)

# Show plot
update_plot()
plt.show()
