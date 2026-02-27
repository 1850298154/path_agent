import json
import matplotlib.pyplot as plt

with open('uav_positions_over_time.json', 'r') as f:
    data = json.load(f)
    uav_data = data.get('uavs', {})
    n_uavs = len(uav_data)
    
    print(f"UAV count: {n_uavs}")
    for step in [5, 10, 15]:
        print(f"Step {step}: plotting...")
