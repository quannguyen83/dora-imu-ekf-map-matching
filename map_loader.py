from dora import Node
import pandas as pd
import pyarrow as pa
import logging

def main():
    node = Node()

    df = pd.read_csv('data/map.csv')
    map_data = df.to_numpy(dtype=float)

    node.send_output('map_data', pa.array([{
        'time': 0,
        'map': map_data.flatten()
    }]), metadata={'shape': str(map_data.shape)})

    logging.info("Sent map data")

if __name__ == "__main__":
    main()