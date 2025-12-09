import numpy as np
from sklearn.cluster import DBSCAN
from scipy.spatial.transform import Rotation
import time
import serial
import open3d as o3d
import json
import threading


# Basic parameters
VOXEL_GRID_SIZE = 50.0  # Size of one voxel cube in mm
WORK_ENVIRONMENT_SIZE = np.array([1500, 1400, 1600])  # Total workspace size in mm
VOXEL_MATRIX_SHAPE = (WORK_ENVIRONMENT_SIZE / VOXEL_GRID_SIZE).astype(int)
SENSOR_MATRIX_WIDTH = 8 # Theoretically, the Sensor can also work with a 4x4 output
COM_PORT = 'COM6' # This is the port used to connect the sensor input.

# Parameters for object and motion detection:
DBSCAN_EPS = 2 # How far two voxels can be apart from another and still be grouped together by DBSCAN in voxel-space.
DBSCAN_MIN_SAMPLES = 8  # How many voxels an object needs to have minimally.
TOTAL_VELOCITY_TOLERANCE = 20.0 # How fast a motion has to be to not be regarded as noise in mm/s.
STATIC_HIT_THRESHOLD = 10 # How long an object needs to be static to be regarded as such in cycles.
TIME_TOLERANCE = 0.08  # How long a voxel is considered recently active in seconds.

# Parameters for visualisation via open3D:
DO_VISUALIZATION = True
SHOW_STATIC = False # Static Objects can be shown in white, but often clutter the screen.
VECTOR_SCALE_FACTOR = 1.0

all_cycle_times = [] # To track performance, all cycle times are saved for later analysis.

# Threading and Synchronization
# Threading is only used to separate the visualization process and the main program cycle to enable visualization with
# Open3D without lowering the performance of the main cycle.
shared_active_points = np.empty((0, 3))  # Voxel indices
shared_active_colors = np.empty((0, 3))  # Corresponding colors
shared_vector_lines = {'points': np.empty((0, 3)), 'lines': np.empty((0, 2))}
voxel_lock = threading.Lock()
running_event = threading.Event()
running_event.set()

start_time = time.time()

# Sensor positions
# The Sensor positions and rotations at the moment have to be manually prepared before running the main program.
# In the future, when receiving real time data from the moving robot, these matrices would need to be continuously
# updated or alternatively changed.
sensor_positions = np.array([
    [760.0, 700.0, 800],
    [737.410, 653.092, 800],
    [686.650, 641.506, 800],
    [645.946, 673.966, 800],
    [645.946, 726.034, 800],
    [686.650, 758.494, 800],
    [737.410, 746.908, 800],
])

ALL_SENSOR_SHIFT = np.array([[0.0, 0.0, 0.0]])
for i in range(len(sensor_positions)):
    sensor_positions[i] = sensor_positions[i] + ALL_SENSOR_SHIFT
# This is a different shifting-table for all sensors just for easier testing.

# Sensor rotations
sensor_rotation = np.array([
    [0.0, 0.0, 0.0],
    [0.0, 0.0, 308.57],
    [0.0, 0.0, 257.14],
    [0.0, 0.0, 205.71],
    [0.0, 0.0, 154.29],
    [0.0, 0.0, 102.86],
    [0.0, 0.0, 51.43],
])

ALL_SENSOR_ROTATION = np.array([[0.0, 0.0, 45.0]])
for i in range(len(sensor_rotation)):
    sensor_rotation[i] = sensor_rotation[i] + ALL_SENSOR_ROTATION
# This is a different turn-table for all sensors just for easier testing.

# Voxel-Matrices
static_voxel_matrix = np.zeros(VOXEL_MATRIX_SHAPE, dtype=np.uint32) # Matrix showing how often a voxel got activated.
voxel_timestamps = np.zeros(VOXEL_MATRIX_SHAPE, dtype=np.float64)  # Timestamp matrix showing when a voxel was most recently activated.


def get_voxel_coordinates(real_coordinate):
    """
    Converts real-world (mm) to voxel grid indices
    """
    voxel_coordinate = np.round(real_coordinate / VOXEL_GRID_SIZE)
    return voxel_coordinate.astype(int)


def get_real_coordinate(voxel_coordinate):
    """
    Converts voxel grid indices back to real-world (mm)
    """
    return voxel_coordinate.astype(float) * VOXEL_GRID_SIZE


def get_sensor_rot(sensor_id):
    """
    Get the pre-calculated rotation object for a sensor
    """
    angles = sensor_rotation[sensor_id]
    return Rotation.from_euler('xyz', angles, degrees=True)


def get_active_voxel_indices(time_now):
    """
    Find all voxels that are 'new' within TIME_TOLERANCE
    """
    # Using the timestamp matrix
    indices = np.where((time_now - voxel_timestamps) < TIME_TOLERANCE)
    return np.column_stack(indices).astype(int)


def is_valid_point(check_point):
    """
    Bounds check for the voxel matrix
    """
    return all(0 <= int(coord) < int(max_dim) for coord, max_dim in zip(check_point, VOXEL_MATRIX_SHAPE))


def read_json(json_input, time_now):
    """
    Parse the JSON file from the Pico and updating the main voxel grid.
    """
    SENSORS_ON_RING = 7
    try:
        json_data = json.loads(json_input)
    except json.JSONDecodeError:
        print("Invalid JSON received. Resetting buffer.")
        if serial_connection:
            serial_connection.reset_input_buffer()
        return

    if not isinstance(json_data, dict):
        return

    for sensor_id in range(SENSORS_ON_RING):
        sensor_name = f"sensor{sensor_id}"
        if sensor_name not in json_data:
            continue

        distance_array = np.array([item[0] for item in json_data[sensor_name]])
        distance_matrix = distance_array.reshape((SENSOR_MATRIX_WIDTH, SENSOR_MATRIX_WIDTH))

        valid_mask = distance_matrix.flatten() > 0
        if not np.any(valid_mask):
            continue

        x, y, z = sensor_positions[sensor_id]
        rot = get_sensor_rot(sensor_id)

        i, j = np.meshgrid(np.arange(SENSOR_MATRIX_WIDTH), np.arange(SENSOR_MATRIX_WIDTH), indexing='ij')

        koo_x = np.tan(np.radians(60.0 / 7.0 * i - 30.0)) * distance_matrix
        koo_y = distance_matrix
        koo_z = np.tan(np.radians(30.0 - 60.0 / 7.0 * j)) * distance_matrix

        all_points = np.stack([koo_x, koo_y, koo_z], axis=-1)
        valid_points = all_points.reshape(-1, 3)[valid_mask]

        transformed_points = rot.apply(valid_points) + np.array([x, y, z])
        add_measurement(transformed_points, time_now)


def add_measurement(measured_real_coordinates, measurement_time):
    """
    Updates the voxel grids with new measurements
    """
    measured_voxel_coordinates = get_voxel_coordinates(measured_real_coordinates)

    for coordinate in measured_voxel_coordinates:
        if is_valid_point(coordinate):
            coord_tuple = tuple(coordinate)
            voxel_timestamps[coord_tuple] = measurement_time

            if static_voxel_matrix[coord_tuple] < STATIC_HIT_THRESHOLD:
                static_voxel_matrix[coord_tuple] += 2


def get_dbscan_labels(voxel_indices):
    """
    Running DBSCAN on given points
    """
    if voxel_indices.size == 0:
        return np.array([])

    clustering = DBSCAN(eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES).fit(voxel_indices)
    return clustering.labels_


all_detected_objects = [] # List which holds all currently active and tracked moving objects.

class Detected_Object:
    """
    Tracking a single object over time by storing its voxel history
    """

    def __init__(self, initial_voxels_indices, current_time):
        all_detected_objects.append(self)
        self.listed_voxels_list = [initial_voxels_indices]
        self.time_list = [current_time]
        self.color = np.random.rand(3)
        self.last_time_updated = current_time

        self.all_movement_vectors = []

    def add_new_voxels(self, new_voxels_indices, current_time):
        self.listed_voxels_list.append(new_voxels_indices)
        self.time_list.append(current_time)
        self.last_time_updated = current_time

    def check_overlap(self, other_voxels_indices, current_time):
        """
        Checks if a *new* cluster (other_voxels) overlaps with *this* object's
        recent voxel history (within TIME_TOLERANCE)
        """
        current_voxels = []
        for i, voxel_array in enumerate(self.listed_voxels_list):
            if current_time - self.time_list[i] < TIME_TOLERANCE:
                current_voxels.extend(voxel_array)

        set1 = set(map(tuple, current_voxels))
        set2 = set(map(tuple, other_voxels_indices))
        return len(set1.intersection(set2)) > 0

    def check_nearby(self, other_voxels_indices, max_distance=2):
        """
        Check if clusters are nearby based on centroid distance.

        NOT OPTIMAL FOR COMPLEX GEOMETRY!
        """
        if not self.listed_voxels_list:
            return False

        current_center = np.mean(self.listed_voxels_list[-1], axis=0)
        other_center = np.mean(other_voxels_indices, axis=0)

        distance = np.linalg.norm(current_center - other_center)
        return distance <= max_distance

    def clean(self, current_time):
        """
        Clean old voxel data. If an object has no recent data, remove it from the global list.
        """
        # Entferne alte Voxel-Daten
        filtered_data = [
            (voxels, t) for voxels, t in zip(self.listed_voxels_list, self.time_list)
            if (current_time - t) < TIME_TOLERANCE
        ]

        if filtered_data:
            self.listed_voxels_list, self.time_list = zip(*filtered_data)
            self.listed_voxels_list = list(self.listed_voxels_list)
            self.time_list = list(self.time_list)
        else:
            if self in all_detected_objects:
                all_detected_objects.remove(self)

    def estimate_movement_vector(self, current_time):
        """
        Velocity estimation based on the difference between the centroids
        of the two most recent voxel sets. Although this is more prone to error with complex geometries it is very
        efficient otherwise.
        """

        if len(self.all_movement_vectors) > 0:
            last_movement_vector = self.all_movement_vectors[-1]
        else:
            last_movement_vector = np.array([0.0, 0.0, 0.0])
        this_movement_vector, current_center = self.get_center_difference(current_time)
        self.all_movement_vectors.append(this_movement_vector)

        movement_vector = np.mean([last_movement_vector, this_movement_vector], axis=0)
        return movement_vector, current_center


    def get_center_difference(self, current_time):
        """
        Returns the vector between the last and current centers. This is used as the estimated movement vector.
        """

        previous_center = None

        valid_times = [t for t in self.time_list if (current_time - t) < TIME_TOLERANCE]
        if not valid_times:
            return np.array([0.0, 0.0, 0.0]), None

        latest_time = max(valid_times)
        latest_voxels = [v for v, t in zip(self.listed_voxels_list, self.time_list) if t == latest_time]
        if latest_voxels:
            current_center = np.mean(get_real_coordinate(np.vstack(latest_voxels)), axis=0)
        else:
            return np.array([0.0, 0.0, 0.0]), None

        previous_times = [t for t in valid_times if t < latest_time]
        if previous_times:
            prev_time = max(previous_times)
            prev_voxels = [v for v, t in zip(self.listed_voxels_list, self.time_list) if t == prev_time]
            if prev_voxels:
                previous_center = np.mean(get_real_coordinate(np.vstack(prev_voxels)), axis=0)

        if current_center is not None and previous_center is not None:
            return (current_center - previous_center), current_center
        else:
            return np.array([0.0, 0.0, 0.0]), current_center

def data_processing_thread():
    """
    Pipeline: Read -> Filter -> Cluster -> Track. Runs in a separate thread.
    """
    global time_last_cycle_timestamp, shared_active_points, shared_active_colors, shared_vector_lines

    time_last_cycle_timestamp = time.time()
    print("Starting data processing thread...")

    timer_for_reset = time.time()

    while running_event.is_set():
        time_now = time.time()

        if timer_for_reset + 5 < time_now:
            timer_for_reset = time_now
            global static_voxel_matrix
            static_voxel_matrix[static_voxel_matrix < STATIC_HIT_THRESHOLD] = 0

        if serial_connection is None or not serial_connection.is_open:
            print("Serial connection lost.")
            running_event.clear()
            break

        new_raw_line = serial_connection.readline().decode('utf-8').rstrip()
        if not new_raw_line:
            time.sleep(0.001)
            continue

        read_json(new_raw_line, time_now)

        all_active_voxel_indices = get_active_voxel_indices(time_now)

        if all_active_voxel_indices.size == 0:
            with voxel_lock:
                shared_active_points = np.empty((0, 3))
                shared_active_colors = np.empty((0, 3))
                shared_vector_lines = {'points': np.empty((0, 3)), 'lines': np.empty((0, 2))}
            time_last_cycle_timestamp = time_now
            continue

        dynamic_voxel_indices_list = []
        for idx in all_active_voxel_indices:
            if static_voxel_matrix[tuple(idx)] >= STATIC_HIT_THRESHOLD:
                pass
            else:
                dynamic_voxel_indices_list.append(idx)
            if static_voxel_matrix[tuple(idx)] > 0:
                static_voxel_matrix[tuple(idx)] -= 1

        dynamic_voxel_indices = np.array(dynamic_voxel_indices_list)
        labels = get_dbscan_labels(dynamic_voxel_indices)

        # Overlap based object tracking
        unique_labels = set(labels) - {-1}
        for cluster_label in unique_labels:
            voxels_of_this_cluster_indices = dynamic_voxel_indices[labels == cluster_label]

            voxels_allocated = False
            for detected_object in list(all_detected_objects):
                if detected_object.check_overlap(voxels_of_this_cluster_indices, time_now):
                    detected_object.add_new_voxels(voxels_of_this_cluster_indices, time_now)
                    voxels_allocated = True
                    break

            if not voxels_allocated:
                # If no overlapping object is found, search for nearby objects
                for detected_object in list(all_detected_objects):
                    if detected_object.check_nearby(voxels_of_this_cluster_indices):
                        detected_object.add_new_voxels(voxels_of_this_cluster_indices, time_now)
                        voxels_allocated = True
                        break

            if not voxels_allocated:
                Detected_Object(voxels_of_this_cluster_indices, time_now)

        # Object Management (Velocity & Cleanup)
        time_delta = time_now - time_last_cycle_timestamp
        if time_delta == 0:
            time_delta = 0.01

        # Collect vector geometry
        vector_points_list = []
        vector_lines_list = []
        current_vector_point_index = 0

        for detected_object in list(all_detected_objects):
            mean_displacement, current_center = detected_object.estimate_movement_vector(time_now)

            if current_center is not None and np.any(mean_displacement):
                velocity_vector = mean_displacement / time_delta
                total_velocity = np.linalg.norm(velocity_vector)

                if total_velocity > TOTAL_VELOCITY_TOLERANCE:
                    print(f"t = {(time_now - start_time):.2f}s: Movement: {total_velocity:.0f} mm/s "
                          f"from center at [{current_center[0]:.0f}, {current_center[1]:.0f}, {current_center[2]:.0f}]")

                # Create vector geometry
                start_voxel = get_voxel_coordinates(current_center)
                scaled_displacement = mean_displacement * VECTOR_SCALE_FACTOR
                end_voxel = get_voxel_coordinates(current_center + scaled_displacement)

                vector_points_list.append(start_voxel)
                vector_points_list.append(end_voxel)
                vector_lines_list.append([current_vector_point_index, current_vector_point_index + 1])
                current_vector_point_index += 2

            detected_object.clean(time_now)

        # Visualization Buffer Prep
        all_points_to_draw = []
        all_colors_to_draw = []

        for obj in list(all_detected_objects):
            if obj.listed_voxels_list:
                latest_voxels = obj.listed_voxels_list[-1]
                all_points_to_draw.append(latest_voxels)
                color_array = np.tile(obj.color, (len(latest_voxels), 1))
                all_colors_to_draw.append(color_array)

        if all_points_to_draw:
            final_points = np.vstack(all_points_to_draw).astype(float)
            final_colors = np.vstack(all_colors_to_draw).astype(float)
        else:
            final_points = np.empty((0, 3))
            final_colors = np.empty((0, 3))

        # Create final vector arrays
        if vector_points_list:
            final_vector_points = np.vstack(vector_points_list).astype(float)
            final_vector_lines = np.array(vector_lines_list).astype(int)
        else:
            final_vector_points = np.empty((0, 3))
            final_vector_lines = np.empty((0, 2))

        # Pass the new geometry to the visualizer thread (thread-safe)
        with voxel_lock:
            shared_active_points = final_points
            shared_active_colors = final_colors
            shared_vector_lines['points'] = final_vector_points
            shared_vector_lines['lines'] = final_vector_lines

        all_cycle_times.append(time.time() - time_now)
        time_last_cycle_timestamp = time_now

    if serial_connection is not None and serial_connection.is_open:
        serial_connection.close()
    print("Data processing thread finished.")


def setup_visualization(vis):
    """
    Sets up the Open3D window, camera, and static geometry.
    """
    pcd_to_update = o3d.geometry.PointCloud()

    # Draw sensors as red spheres
    sensor_coords_scaled = sensor_positions / VOXEL_GRID_SIZE
    for pos in sensor_coords_scaled:
        sensor_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.5)
        sensor_sphere.paint_uniform_color([1.0, 0.0, 0.0])
        sensor_sphere.translate(pos)
        vis.add_geometry(sensor_sphere)

    # Draw workspace bounding box
    room_bounds = o3d.geometry.AxisAlignedBoundingBox([0, 0, 0], VOXEL_MATRIX_SHAPE)
    room_lines = o3d.geometry.LineSet.create_from_axis_aligned_bounding_box(room_bounds)
    room_lines.paint_uniform_color([0.5, 0.5, 0.5])
    vis.add_geometry(room_lines)

    # Draw Coordinate Frame
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=10.0, origin=[0, 0, 0]
    )
    vis.add_geometry(coord_frame)

    # Render options
    opt = vis.get_render_option()
    opt.background_color = np.array([0.15, 0.15, 0.18])
    opt.point_size = 3.0
    opt.line_width = 1.5
    opt.light_on = True

    # Camera setup
    ctr = vis.get_view_control()
    ctr.set_front([0.5, -0.5, 0.8])
    ctr.set_lookat(VOXEL_MATRIX_SHAPE / 2)
    ctr.set_up([0, 0, 1])
    ctr.set_zoom(0.8)

    vis.add_geometry(pcd_to_update)

    line_set_to_update = o3d.geometry.LineSet()
    vis.add_geometry(line_set_to_update)

    return pcd_to_update, line_set_to_update

# Serial Connection Setup
try:
    serial_connection = serial.Serial(port=COM_PORT, baudrate=460800, timeout=1)
except serial.SerialException as e:
    print(f"Failed to connect: {e}")
    running_event.clear()
    serial_connection = None

# Main Program loop:
if __name__ == "__main__":
    if DO_VISUALIZATION and running_event.is_set():
        print("Starting render thread (main thread)...")

        vis = o3d.visualization.Visualizer()
        vis.create_window("Open3D Live Voxel", width=1200, height=900)

        pcd, line_set = setup_visualization(vis)

        processing_thread = threading.Thread(target=data_processing_thread)
        processing_thread.start()

        print("Starting Open3D visualizer...")

        while running_event.is_set():
            # Get the latest point cloud data (this is thread-safe)
            with voxel_lock:
                points_data = shared_active_points
                colors_data = shared_active_colors
                vector_points_data = shared_vector_lines['points']
                vector_lines_data = shared_vector_lines['lines']

            # Update points_data and colors_data for stationary voxels
            static_voxels = np.where(static_voxel_matrix >= 1)#STATIC_HIT_THRESHOLD)

            # Visualize the static objects if desired.
            if len(static_voxels[0]) >= STATIC_HIT_THRESHOLD and SHOW_STATIC:
                static_points = np.stack(static_voxels, axis=1)

                static_colors = np.ones((len(static_points), 3))

                points_data = np.vstack([static_points, points_data]) if len(points_data) > 0 else static_points
                colors_data = np.vstack([static_colors, colors_data]) if len(colors_data) > 0 else static_colors

            # Update the point cloud
            if points_data.size > 0:
                pcd.points = o3d.utility.Vector3dVector(points_data)
                if len(points_data) == len(colors_data):
                    pcd.colors = o3d.utility.Vector3dVector(colors_data)
                else:
                    pcd.paint_uniform_color([0.1, 0.1, 0.9])
            else:
                pcd.points = o3d.utility.Vector3dVector(np.empty((0, 3)))
                pcd.colors = o3d.utility.Vector3dVector(np.empty((0, 3)))

            vis.update_geometry(pcd)

            # Update vectors
            if vector_points_data.size > 0:
                line_set.points = o3d.utility.Vector3dVector(vector_points_data)
                line_set.lines = o3d.utility.Vector2iVector(vector_lines_data)
                line_set.paint_uniform_color([1.0, 1.0, 0.0])
            else:
                line_set.points = o3d.utility.Vector3dVector(np.empty((0, 3)))
                line_set.lines = o3d.utility.Vector2iVector(np.empty((0, 2)))

            vis.update_geometry(line_set)

            if not vis.poll_events():
                running_event.clear()
                break
            vis.update_renderer()

            if not processing_thread.is_alive():
                print("Data thread has stopped.")
                running_event.clear()

            time.sleep(0.016)  # ~60 FPS cap

        vis.destroy_window()
        processing_thread.join()

    elif running_event.is_set():
        data_processing_thread()

    end_time = time.time()
    print(f"Total runtime: {end_time - start_time:.2f} sec.")
    if all_cycle_times:
        avg_cycle_ms = np.mean(all_cycle_times) * 1000
        print(f"Average cycle time: {avg_cycle_ms:.2f} ms ({len(all_cycle_times)} cycles)")
    else:
        print("No cycles processed.")
