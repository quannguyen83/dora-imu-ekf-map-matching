from dora import Node
import math
import logging
import pyarrow as pa
import numpy as np
import time

def predict_ekf(previous_state, imu_data, P, Q, dt):
    """
    Predict the next state using an Extended Kalman Filter (EKF) based on the previous state and IMU data.
    """
    # Unpack previous state
    _, _, v, pitch_deg, yaw_deg = previous_state
    ax, ay, az = imu_data['acc']

    # sin/cos pitch and yaw for later use
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    sin_pitch = math.sin(pitch)
    cos_pitch = math.cos(pitch)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    # Predict velocity
    pred_v = v + dt * (cos_pitch * ay - sin_pitch * az)

    # Distance increment based on average velocity during the time step
    dist_increment = 0.5 * (v + pred_v) * dt

    # State jacobian for Extended Kalman Filter
    F = np.array([
        [1, 0, -sin_yaw * dt, 0, -dist_increment * cos_yaw],
        [0, 1,  cos_yaw * dt, 0, -dist_increment * sin_yaw],
        [0, 0, 1, -dt * (sin_pitch * ay + cos_pitch * az), 0],
        [0, 0, 0, 1, 0],
        [0, 0, 0, 0, 1]
    ])

    # Predicted covarience P
    P_predicted = F @ P @ F.T + Q # Assuming process noise is identity for simplicity

    return P_predicted

def main():
    node = Node()

    previous_state = [0, 0, 0, 0, 0] 
    P_init = np.diag([1, 1, 0.1, 1, 1])  # Initial state covariance
    Q = np.diag([0.1, 0.1, 0.1, 0.1, 0.1])  # Process noise covariance

    imu_data_msgs = {}
    cov_P_msgs = {}

    # timestamp_init = "{:.6e}".format(0.02)
    timestamp_init = 0.02

    cov_P_msgs[timestamp_init] = {
        'covariance': P_init,
        'pose': previous_state
    }

    for event in node:
        if event["type"] == "INPUT":
            input_id = event["id"]

            msg = event["value"].to_pylist()[0]
            timestamp = msg["time"]

            # logging.info(f"Input id: {input_id} at time {timestamp}")

            if input_id == "imu":
                imu_data_msgs[timestamp] = msg
            
            elif input_id == "previous_covariance":
                cov_P_msgs[timestamp] = msg

            elif input_id == "system_init":
                node.send_output('predicted_state', pa.array([{
                    'time': 0,
                    'param': P_init.tolist()
                }]))

            if timestamp in imu_data_msgs and timestamp in cov_P_msgs:                
                updated_state = cov_P_msgs.pop(timestamp)
                imu_data = imu_data_msgs.pop(timestamp)

                P = updated_state['covariance']
                previous_state = updated_state['pose']

                P_predicted = predict_ekf(
                    previous_state=previous_state, 
                    imu_data=imu_data, 
                    P=P, Q=Q, 
                    dt=0.02
                )

                P_pred = P_predicted.tolist()  # Update P for the next iteration

                # logging.info(f"Predicted EKF covariance at time {timestamp}")

                node.send_output('predicted_state', pa.array([{
                    'time': imu_data['time'],
                    'param': P_pred
                }]))

if __name__ == "__main__":
    main()