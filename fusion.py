from dora import Node
import logging

def main():
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            input_id = event["id"]
            if input_id == "imu":
                imu_data = event["value"].to_pylist()[0]
                # logging.info(f"Received IMU data at time {imu_data['time']}")
            elif input_id == "map":
                map_data = event["value"].to_pylist()[0]
                # logging.info("Received map data")

if __name__ == "__main__":
    main()