import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.neighbors import NearestNeighbors
from sklearn.metrics import silhouette_score
from sklearn.model_selection import ParameterGrid
from scipy.spatial.transform import Rotation
import time
import serial
import json
import matplotlib.pyplot as plt

# Config
COLLECTION_DURATION = 30  # [sec]
NUM_FRAMES = 10
VOXEL_GRID_SIZE = 50.0 
WORK_ENV_SIZE = np.array([1500, 1400, 1600])
VOXEL_MATRIX_SHAPE = (WORK_ENV_SIZE / VOXEL_GRID_SIZE).astype(int)
TIME_TOLERANCE = 1.0
STATIC_HIT_THRESHOLD = 100 
COM_PORT = 'COM3' # may vary
SENSORS_ON_RING = 7

start_time = time.time()

# Sensor Setup
# Cartesian  Position
sensor_positions = np.array([
    [760.0, 700.0, 800], [737.410, 653.092, 800], [686.650, 641.506, 800],
    [645.946, 673.966, 800],[645.946, 726.034, 800], [686.650, 758.494, 800],
    [737.410, 746.908, 800],
])

# Robot Base Rotation
sensor_rotation = np.array([
    [0.0, 0.0, 0.0], [0.0, 0.0, 308.57], [0.0, 0.0, 257.14],
    [0.0, 0.0, 205.71], [0.0, 0.0, 154.29], [0.0, 0.0, 102.86],
    [0.0, 0.0, 51.43],
])

GLOBAL_ROTATION_OFFSET = np.array([[0.0, 0.0, 45.0]])
for i in range(len(sensor_rotation)):
    sensor_rotation[i] = sensor_rotation[i] + GLOBAL_ROTATION_OFFSET

SENSOR_MATRIX_WIDTH = 8

# Voxel Grid
voxel_matrix = np.zeros(VOXEL_MATRIX_SHAPE, dtype=np.float64)
static_voxel_matrix = np.zeros(VOXEL_MATRIX_SHAPE, dtype=np.uint32)

def get_voxel_coordinates(real_coord):
    return np.round(real_coord / VOXEL_GRID_SIZE).astype(int)

def get_sensor_rotation_obj(sensor_id):
    angles = sensor_rotation[sensor_id]
    return Rotation.from_euler('xyz', angles, degrees=True)

def get_active_voxel_indices(current_time):
    """
    Finding updated voxels within the time tolerance
    """
    indices = np.where((current_time - voxel_matrix) < TIME_TOLERANCE)
    return np.column_stack(indices).astype(int)

def is_valid_point(point):
    return all(0 <= int(coord) < int(max_dim) for coord, max_dim in zip(point, VOXEL_MATRIX_SHAPE))

def add_measurement(measured_real_coords, timestamp):
    measured_voxels = get_voxel_coordinates(measured_real_coords)
    
    for voxel in measured_voxels:
        if is_valid_point(voxel):
            coord_tuple = tuple(voxel)
            voxel_matrix[coord_tuple] = timestamp
            
            # Threshold
            if static_voxel_matrix[coord_tuple] < STATIC_HIT_THRESHOLD:
                static_voxel_matrix[coord_tuple] += 1

def process_sensor_data(json_input, current_time, serial_conn):
    try:
        data = json.loads(json_input)
    except json.JSONDecodeError:
        if serial_conn:
            serial_conn.reset_input_buffer()
        return

    if not isinstance(data, dict):
        return

    for sensor_id in range(SENSORS_ON_RING):
        sensor_key = f"sensor{sensor_id}"
        if sensor_key not in data:
            continue
            
        # Extract distances
        dist_array = np.array([item[0] for item in data[sensor_key]])
        dist_matrix = dist_array.reshape((SENSOR_MATRIX_WIDTH, SENSOR_MATRIX_WIDTH))
        
        valid_mask = dist_matrix.flatten() > 0
        if not np.any(valid_mask):
            continue

        # Sensor geometry and transformation
        pos_x, pos_y, pos_z = sensor_positions[sensor_id]
        rot_obj = get_sensor_rotation_obj(sensor_id)
        
        i, j = np.meshgrid(np.arange(SENSOR_MATRIX_WIDTH), np.arange(SENSOR_MATRIX_WIDTH), indexing='ij')
        
        # Local coordinates based on ToF sensor FOV logic
        coords_x = np.tan(np.radians(60.0 / 7.0 * i - 30.0)) * dist_matrix
        coords_y = dist_matrix
        coords_z = np.tan(np.radians(30.0 - 60.0 / 7.0 * j)) * dist_matrix
        
        points_local = np.stack([coords_x, coords_y, coords_z], axis=-1)
        points_valid = points_local.reshape(-1, 3)[valid_mask]
        
        points_global = rot_obj.apply(points_valid) + np.array([pos_x, pos_y, pos_z])
        
        add_measurement(points_global, current_time)

def run_optimization():
    print(f"Starting data collection for {COLLECTION_DURATION} seconds...")
    
    try:
        serial_conn = serial.Serial(port=COM_PORT, baudrate=460800, timeout=1) 
    except serial.SerialException as e:
        print(f"Error opening COM port: {e}")
        return

    collected_frames = []
    end_time = time.time() + COLLECTION_DURATION
    capture_interval = COLLECTION_DURATION / NUM_FRAMES
    next_capture = time.time() + capture_interval

    # Collection Loop
    while time.time() < end_time:
        now = time.time()
        raw_line = serial_conn.readline().decode('utf-8').rstrip()
        
        if not raw_line:
            time.sleep(0.001)
            continue
            
        process_sensor_data(raw_line, now, serial_conn)
        
        if time.time() >= next_capture:
            print(f"...Frame {len(collected_frames) + 1}/{NUM_FRAMES} captured at t={now - start_time:.1f}s")
            
            active_indices = get_active_voxel_indices(now)
            
            if active_indices.size > 0:
                dynamic_voxels = []
                for idx in active_indices:
                    if static_voxel_matrix[tuple(idx)] < STATIC_HIT_THRESHOLD:
                        dynamic_voxels.append(idx)
                
                if dynamic_voxels:
                    collected_frames.append(np.array(dynamic_voxels))
            
            next_capture += capture_interval

    serial_conn.close()
    print("\nData collection completed.")

    if not collected_frames:
        print("ERROR: No valid frames collected.")
        return

    dataset = np.vstack(collected_frames)
    
    if dataset.shape[0] > 10000:
        print(f"Dataset is large ({dataset.shape[0]} points). Processing might be slow.")

    # Hyperparameter Optimization

    # Method A: k-Distance Graph /Elbow Method
    k_min = 3
    k_max = 10
    
    print(f"\n### Method A: k-Distance Graph (k={k_min} to {k_max}) ###")
    print("Calculating nearest neighbors...")
    
    nn = NearestNeighbors(n_neighbors=k_max)
    nn.fit(dataset)
    distances, _ = nn.kneighbors(dataset)
    
    plt.figure(figsize=(12, 7))
    
    for k in range(k_min, k_max + 1):
        # The distance to the k-th neighbor (index k-1)
        k_dist_sorted = np.sort(distances[:, k - 1], axis=0)
        plt.plot(k_dist_sorted, label=f'k={k}')

    plt.title(f'k-Distance Graph (Dynamic Data Only)')
    plt.xlabel('Points sorted by distance')
    plt.ylabel('Distance to k-th Nearest Neighbor (Epsilon candidate)')
    plt.legend() 
    plt.grid(True)
    
    filename = 'k_distance_plot_filtered.png'
    print(f"Plot saved to '{filename}'. Please look for the 'elbow' in the graph.")
    plt.savefig(filename)
    plt.show()

    # Method B: Grid Search with Silhouette Score
    print("\n### Method B: Grid Search (Silhouette Score) ###")
    print("Searching for optimal DBSCAN parameters...")

    # Range Adjustment Based on Method A
    param_grid = {
        'eps': [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5], 
        'min_samples': [3, 5, 8, 10, 12, 15]   
    }

    best_score = -1
    best_params = {}

    for params in ParameterGrid(param_grid):
        db = DBSCAN(eps=params['eps'], min_samples=params['min_samples'])
        labels = db.fit_predict(dataset)
        
        # Ignoring Noise (-1) for Cluster Counting
        unique_labels = set(labels) - {-1}
        
        if len(unique_labels) < 2:
            continue
            
        score = silhouette_score(dataset, labels)
        print(f"Params: {params} | Score: {score:.4f}")
        
        if score > best_score:
            best_score = score
            best_params = params

    print("\n### Optimization Results ###")
    if best_score == -1:
        print("No valid cluster configuration found (all noise or single cluster).")
    else:
        print(f"Best Silhouette Score: {best_score:.4f}")
        print(f"Optimal Parameters:")
        print(f"  eps = {best_params['eps']}")
        print(f"  min_samples = {best_params['min_samples']}")

if __name__ == "__main__":

    run_optimization()
