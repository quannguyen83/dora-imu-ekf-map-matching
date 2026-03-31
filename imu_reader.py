import random
from dora import Node
import pandas as pd
import logging
import pyarrow as pa
import time

def main():
    node = Node()

    df = pd.read_csv('data/02_onelap.csv')
    started = False

    for event in node:
        if event["type"] == "INPUT":
            input_id = event["id"]
            if input_id == "start":
                started = True

        if started:  
            for _ in range(2):
                logging.info("Send init message ... ...")
                node.send_output('system_init', pa.array([{
                    'time': 0.0, 
                    'acc': [0.0, 0.0, 0.0],
                    'gyro': [0.0, 0.0, 0.0]
                }]))
                node.next(timeout=1)

            for row in df.itertuples():
                start_time = time.time()

                t = row.time
                acc = [row.ax, row.ay, row.az]
                gyro = [row.wx, row.wy, row.wz]

                node.send_output('imu_data', pa.array([{
                    'time': t,
                    'acc': acc,
                    'gyro': gyro
                }]))

                elapsed = time.time() - start_time
                sleep_time = max(0, 0.02 - elapsed)
                time.sleep(sleep_time)
                

            logging.info("Finished sending IMU data")
            break

if __name__ == "__main__":
    main()


