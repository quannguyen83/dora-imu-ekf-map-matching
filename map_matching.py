import pandas as pd
from dora import Node
import logging
import numpy as np
import pyarrow as pa
from types import SimpleNamespace

def get_candidate_points(previous_state, radius, center):
    """
    Get candidate points from the map data based on the predicted position.
    This is a placeholder function and should be implemented with actual logic to extract candidate points from the map.
    """
    map_points = previous_state.map
    start_idx = previous_state.mapIdx
    total_points = len(map_points)

    current_position = np.asarray(center, dtype=float)
    
    candidate_indexes = [start_idx]

    cursor = start_idx + 1
    search_counter = 0
    
    while search_counter < total_points:
        if cursor >= total_points:
            cursor = 0
        
        if cursor == start_idx:
            break

        if np.linalg.norm(map_points[cursor, :2] - current_position) > radius:
            break

        candidate_indexes.append(cursor)
        cursor += 1
        search_counter += 1

    candidate_points = map_points[candidate_indexes]

    return candidate_points, np.array(candidate_indexes, dtype=int)

def filter_candidates(candidate_points, candidate_idx, candidate_distances, previous_state, epsilon):
    """
    Filter candidate points based on distance and direction criteria.
    This is a placeholder function and should be implemented with actual filtering logic.
    """
    filtered_candidates = []

    # Y axis unit vector for direction filtering
    B = np.array([0.0, 1.0])
    points_xy = candidate_points[:, :2]
    candidate_vectors = points_xy - previous_state.position[:2]

    dots_B = candidate_vectors @ B
    normB = np.linalg.norm(B)

    # Calculate dot products for direction 
    dots_C = candidate_vectors @ previous_state.vector[:2]

    # Candidate angles
    safe_dists = np.maximum(np.linalg.norm(candidate_vectors, axis=1), 1e-6)
    ratio = dots_B / (normB * safe_dists)
    ratio = np.clip(ratio, -1.0, 1.0)
    angles = np.degrees(np.arccos(ratio))

    # Combined mask
    mask = (candidate_distances > 1e-6) & (dots_C >= 0)

    if not np.any(mask):
        return None, None, None, None
    
    filtered_candidates = candidate_points[mask]
    angles = angles[mask]
    candidate_distances = candidate_distances[mask]
    valid_sort_idx = candidate_idx[mask]

    return filtered_candidates, candidate_distances, valid_sort_idx, angles

def score_candidates(candidate_distances, angles, estimated_distance_on_speed, estimated_pose, filtered_candidates):
    """
    Score candidate points based on distance, angle, and position criteria.
    This is a placeholder function and should be implemented with actual scoring logic.
    """
    scores = []

    # Scoring
    dists_diff = np.abs(candidate_distances - estimated_distance_on_speed)
    angles_diff = np.abs(angles - estimated_pose[4])
    pos_diff = np.linalg.norm(filtered_candidates - estimated_pose[:2], axis=1)

    def normalize(v):
        max_v = np.max(v)
        if max_v > 0:
            return v / max_v
        return v

    scores = (
        0.7 * normalize(dists_diff) 
        + 0.2 * normalize(angles_diff)
        + 0.1 * normalize(pos_diff)
    )
    
    return scores

def map_matching(est_pose, previous_state, r, dt):
    """
    Perform map matching to refine the predicted pose based on the map data.
    This is a placeholder function and should be implemented with actual map matching logic.
    """
    epsilon = 1e-5 

    estimated_distance_on_speed = abs(est_pose[2] * dt)
    logging.info(f"Estimated distance on speed: {int(estimated_distance_on_speed * 1000)} mm")
    logging.info(f"Estimated distance on pose: {int(np.linalg.norm(est_pose[:2] - previous_state.position) * 1000)} mm")

    if estimated_distance_on_speed < epsilon:
        # logging.info("Estimated distance is less than epsilon, skipping map matching")
        return previous_state.position, previous_state

    xy_estimated = est_pose[:2]
        
    candidates, candidate_idx = get_candidate_points(previous_state, r, xy_estimated)
    candidates = np.asarray(candidates, dtype=float)

    # Candidate distances and vectors to previous map position for distance 
    # and direction filtering
    candidate_vectors = candidates - previous_state.position
    candidate_distances = np.linalg.norm(candidate_vectors, axis=1)

    filtered_candidates, candidate_distances, valid_candidate_idx, angles = filter_candidates(candidates, candidate_idx, candidate_distances, previous_state, epsilon)

    if filtered_candidates is None:
        # logging.info("No valid candidates after filtering, skipping map matching")
        return previous_state.position, previous_state
    
    scores = score_candidates(
        candidate_distances, 
        angles, 
        estimated_distance_on_speed, 
        est_pose, 
        filtered_candidates
    )

    matched_idx = int(np.argmin(scores))
    matched_position = filtered_candidates[matched_idx]

    logging.info(f"Distance to previous position: {int(np.linalg.norm(matched_position - previous_state.position) * 1000)} mm")

    previous_state.mapIdx = valid_candidate_idx[matched_idx]
    previous_state.vector = matched_position - previous_state.position
    previous_state.position = matched_position

    return matched_position, previous_state

def main():
    node = Node()

    map_df = pd.read_csv('data/map.csv')
    map_data = map_df.to_numpy(dtype=float)

    # Initialize previous state
    previous_state = SimpleNamespace(
        position=np.array([0.0, 0.0]),
        vector=np.array([0.0, 1.0]),
        map=map_data,
        mapIdx=0
    )

    node.send_output('start_stream', pa.array([{
        'time': 0,
        'start': True
    }]))

    radius_msgs = {}
    pose_msgs = {}

    for event in node:
        if event["type"] == "INPUT":

            input_id = event["id"]
            msg = event["value"].to_pylist()[0]
            timestamp = msg['time']

            if input_id == "searching_radius":
                search_radius = msg['search_radius']
                radius_msgs[timestamp] = search_radius
            elif input_id == "position_estimate":
                est_pose = np.asarray(msg["pose"], dtype=float)
                pose_msgs[timestamp] = est_pose

            if timestamp in radius_msgs and timestamp in pose_msgs:
                logging.info(f"Time ...... :: {timestamp}")
                search_radius = radius_msgs.pop(timestamp)
                est_pose = pose_msgs.pop(timestamp)

                matched_position, previous_state = map_matching(est_pose, previous_state, r=search_radius, dt=0.02)

                distance_to_estimate = np.linalg.norm(matched_position - est_pose[:2])
                # logging.info(f"Distance to estimate: {int(distance_to_estimate * 1000)} mm at time {timestamp}")

                # logging.info(f"Matched position at time {timestamp}: {matched_position}")
                
                node.send_output('matched_position', pa.array([{
                    'time': timestamp,
                    'matched_xy': matched_position
                }]))

if __name__ == "__main__":
    main()
