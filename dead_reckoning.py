from dora import Node
import pandas as pd
import pyarrow as pa
import logging
from typing import Sequence
import math

def dead_reckoning(
        previous_pose : Sequence[float],
        imu_data : Sequence[float],
        dt
):
    """
    Perform dead reckoning to estimate the pose based on IMU data.
    """
    px, py, v, pitch_deg, yaw_deg = previous_pose
    ax, ay, az = imu_data['acc']
    wx, wy, wz = imu_data['gyro']

    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    pred_a = math.cos(pitch) * ay - math.sin(pitch) * az
    pred_v = v + pred_a * dt
    
    pred_px = px - 0.5 * (v + pred_v) * math.sin(yaw) * dt
    pred_py = py + 0.5 * (v + pred_v) * math.cos(yaw) * dt

    pred_pitch_deg = pitch_deg + wx * dt
    pred_yaw_deg   = yaw_deg   + wz * dt
    
    pred_state = [
        float(pred_px),
        float(pred_py),
        float(pred_v),
        float(pred_pitch_deg),
        float(pred_yaw_deg)
    ]

    return pred_state

def main():
    node = Node()

    previous_pose = [0, 0, 0, 0, 0]

    # timestamp_init = "{:.6e}".format(0.02)
    timestamp_init = 0.02

    imu_data_msgs = {}
    previous_pose_msgs = {}

    previous_pose_msgs[timestamp_init] = previous_pose 

    for event in node:

        if event["type"] == "INPUT":
            input_id = event["id"]
            msg = event["value"].to_pylist()[0]
            timestamp = msg['time']

            # logging.info(f"Received input at timestamp: {timestamp} from input: {input_id}")
            # logging.info(f"Size: pose({len(previous_pose_msgs)}) and imu({len(imu_data_msgs)})")

            if input_id == "imu":
                imu_data = msg
                imu_data_msgs[timestamp] = imu_data

            elif input_id == "previous_pose":
                previous_pose = msg['pose']
                previous_pose_msgs[timestamp] = previous_pose

                # logging.info(f"Updated previous pose at time {msg['time']}")

            elif input_id == "system_init":
                node.send_output('position_estimate', pa.array([{
                    'time': 0,
                    'pose': previous_pose
                }]))

            if timestamp in imu_data_msgs and timestamp in previous_pose_msgs:
                previous_pose = previous_pose_msgs.pop(timestamp)
                imu_data = imu_data_msgs.pop(timestamp)

                predicted_state = dead_reckoning(previous_pose=previous_pose, imu_data=imu_data, dt=0.02)
                # logging.info(f"Predicted state at time {timestamp}: [{predicted_state[0]}, {predicted_state[1]}]")
                # previous_pose = predicted_state
                node.send_output('position_estimate', pa.array([{
                    'time': imu_data['time'],
                    'pose': predicted_state
                }]))

if __name__ == "__main__":
    main()