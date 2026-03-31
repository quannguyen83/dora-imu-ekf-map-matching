from dora import Node
import pandas as pd
import pyarrow as pa
import logging
import numpy as np

def main():
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            input_id = event["id"]
            if input_id == "predicted_state":
                predicted_state = event["value"].to_pylist()[0]
                param = predicted_state['param']
                P = np.asarray(param, dtype=float)

                var_x = P[0][0]
                var_y = P[1][1]

                sigma_scale = 1.0

                avg_var = 0.5 * (var_x + var_y)
                uncertainty_radius = sigma_scale * (avg_var ** 0.5)

                # logging.info(f"Predicted searching radius {uncertainty_radius} at time {predicted_state['time']}")

                node.send_output('searching_radius', pa.array([{
                    'time': predicted_state['time'],
                    'search_radius': uncertainty_radius
                }]))

if __name__ == "__main__":
    main()