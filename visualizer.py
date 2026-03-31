from dora import Node
import pandas as pd
import matplotlib.pyplot as plt
import logging

def main():

    node = Node()

    try:
        map_df = pd.read_csv('data/map.csv')
        map_data = map_df[['X', 'Y']].to_numpy()

    except Exception as e:
        logging.error(f"Unable to read map file: {e}")
        return

    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 8))

    ax.plot(map_data[:, 0], map_data[:, 1], 'k.', markersize=1, label='Map', alpha=0.3)

    path_line, = ax.plot([], [], 'r-', linewidth=1, label='Actual Path')
    current_point, = ax.plot([], [], 'go', markersize=8, label='Current Position') # Màu xanh lá cho xe

    ax.legend()
    ax.set_title("Dora Real-time Visualizer")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True, linestyle=':', alpha=0.6)

    x_history, y_history = [], []

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "updated_ekf":
                msg = event["value"].to_pylist()[0]
                pose = msg.get("pose") # Giả định là [x, y]

                matched_xy = pose[:2] if pose else None

                # logging.info(f"Received matched position at time {msg.get('time')}: {matched_xy}")
                if matched_xy:
                    x, y = matched_xy[0], matched_xy[1]
                    x_history.append(x)
                    y_history.append(y)

                    if len(x_history) > 100:
                        x_history.pop(0)
                        y_history.pop(0)

                    path_line.set_data(x_history, y_history)
                    current_point.set_data([x], [y])

                    ax.set_xlim(x - 1, x + 1)
                    ax.set_ylim(y - 1, y + 1)

                    plt.pause(0.001)
        elif event["type"] == "STOP":
            break

    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()