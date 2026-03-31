from dora import Node
import logging
import numpy as np
import pyarrow as pa

def main():
    node = Node()
    
    H = np.array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0]
        ], dtype=float)

    R = np.diag([0.1, 0.1])  # Measurement noise covariance (tuning parameter)
    position_msgs = {}
    matched_position_msgs = {}
    P_predicted_msgs = {}

    for event in node:
        
        if event["type"] == "INPUT":
            input_id = event["id"]
            
            msg = event["value"].to_pylist()[0]
            timestamp = msg['time']

            if input_id == "position_estimate":
                position_estimate = msg["pose"]
                position_msgs[timestamp] = position_estimate

            elif input_id == "matched_position":
                matched_position = msg["matched_xy"]
                matched_position_msgs[timestamp] = matched_position

            elif input_id == "predicted_state":
                P_predicted = msg["param"]
                P_predicted_msgs[timestamp] = P_predicted

            # Synchronize messages based on timestamp for EKF update
            if timestamp in position_msgs and timestamp in matched_position_msgs and timestamp in P_predicted_msgs:

                position_estimate = np.array(position_msgs.pop(timestamp))
                matched_position = np.array(matched_position_msgs.pop(timestamp))
                P_predicted = P_predicted_msgs.pop(timestamp)
                
                P = np.asarray(P_predicted, dtype=float)
                
                # EKF update
                K = P @ H.T @ np.linalg.inv(H @ P @ H.T + R) 
                updated_pose = position_estimate + K @ (matched_position - H @ position_estimate)
                P_updated = (np.eye(5) - K @ H) @ P

                P_u = P_updated.tolist()

                if not timestamp:
                    next_ts_str = 0
                else:
                    # next_ts_str = "{:.6e}".format(float(timestamp) + 0.02)
                    next_ts_str = round(timestamp + 0.02, 2)
                
                node.send_output('updated_ekf', pa.array([{
                    'time': next_ts_str,
                    'pose': updated_pose,
                    'covariance': P_u
                }]))

if __name__ == "__main__":
    main()