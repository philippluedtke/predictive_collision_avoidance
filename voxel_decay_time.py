import math
import numpy as np
from sklearn.cluster import DBSCAN
from scipy.spatial.transform import Rotation
import time
import serial
from sklearn.neighbors import NearestNeighbors
import keyboard
import plotly.graph_objects as go


start_time = time.time()

IMAGE_TIMER = 5

VOXEL_GRID_SIZE = 50
# Defines the real distance between each voxel-center in mm.

WORK_ENVIRONMENT_SIZE = np.array([1500, 1400, 1600])
# Defines the real dimensions of the environment where the robot operates in mm.
# This must be defined beforehand, as the matrix in which all values are stored
# will be defined by these dimensions and the grid size.

VOXEL_MATRIX_SHAPE = (WORK_ENVIRONMENT_SIZE / VOXEL_GRID_SIZE).astype(int)
voxel_matrix = np.zeros(VOXEL_MATRIX_SHAPE, dtype=np.float32)
# The voxels are stored in this sparse matrix. This way, performance is increased,
# but nothing outside the work environment can be detected.

# The positions in mm and directions in degrees of all Sensors are stored here.
# Later, this information would be updated as the robot moves.
sensor_positions = np.array([
    [700.0, 700.0, 800.0]
])

sensor_rotation = np.array([
    [0.0, 0.0, 0.0]
])

SENSOR_MATRIX_WIDTH = 8
DBSCAN_EPS = 2
DBSCAN_MIN_SAMPLES = 2

ADD_UNCERTAINTY_SPHERE = False
# Because of how the Sensor returns the data, the volume of where any data point could be increases with the distance
# to the sensor. We can counteract this by increasing the amount of voxels activated by each reading.

TIME_TOLERANCE = 1
# How many seconds will an object remain "seen" at its last position?

def get_voxel_coordinates(real_coordinate):
    voxel_coordinate = np.round(real_coordinate / VOXEL_GRID_SIZE)
    return voxel_coordinate.astype(int)
def get_real_coordinate(voxel_coordinate):
    real_coordinate = voxel_coordinate * VOXEL_GRID_SIZE
    return real_coordinate

def get_sensor_rot(sensor_id):
    angles = sensor_rotation[sensor_id]
    euler_rotation_matrix = Rotation.from_euler('xyz', angles, degrees=True)
    return euler_rotation_matrix

def get_sphere_voxels(radius, center=(0, 0, 0)):
    """This returns all the coordinates of a sphere in a radius."""

    center = np.array(center)

    if radius <= 0: return [center]

    sphere_voxels = []

    int_radius = math.ceil(radius)

    for x in range(-int_radius, int_radius + 1):
        for y in range(-int_radius, int_radius + 1):
            for z in range(-int_radius, int_radius + 1):

                distance = math.sqrt(x ** 2 + y ** 2 + z ** 2)

                if distance <= radius:
                    sphere_voxels.append((x + center[0], y + center[1], z + center[2]))

    return np.array(sphere_voxels)
def get_uncertainty_radius(distance, detection_angle=60):
    """Because of the way the Sensor outputs the data, the volume in which the detected point could be
    increases with the distance. To reflect this, not only the 'hit' voxel is activated, but also a number of
    surrounding voxels inside a sphere with a radius which is calculated here."""

    radius_modifier = 2
    # Make this 2 for the resulting sphere not to overlap with voxels outside the possible volume (Safer).
    # Make this sqrt(2) for the resulting sphere to fill the whole possible volume (More closed Volumes).

    tan = math.tan(detection_angle / 2)
    field_of_view_width = tan * distance
    uncertainty_radius = -1 * field_of_view_width / (SENSOR_MATRIX_WIDTH * radius_modifier)

    return uncertainty_radius

def get_distance(coordinate_a, coordinate_b):
    distance = np.linalg.norm(coordinate_a - coordinate_b)
    return distance

def get_all_active(get_time=None):
    if not get_time: get_time = time.time()
    indices = np.where(get_time - voxel_matrix < TIME_TOLERANCE)
    this_active_voxels = np.column_stack(indices)
    return this_active_voxels
def is_valid_point(check_point):
    return all(0 <= coord < max_dim for coord, max_dim in zip(check_point, VOXEL_MATRIX_SHAPE))
def get_parsed_line_old(raw_line):
    line_items = raw_line.strip().split(' ')
    sensor_id = int(line_items[1])
    distance_array = np.array(line_items[2:2 + 64], dtype=np.float32)
    distance_matrix = distance_array.reshape((SENSOR_MATRIX_WIDTH, SENSOR_MATRIX_WIDTH))

    x, y, z = sensor_positions[sensor_id]
    rot = get_sensor_rot(sensor_id)

    i = np.arange(SENSOR_MATRIX_WIDTH)
    j = np.arange(SENSOR_MATRIX_WIDTH)
    i_grid, j_grid = np.meshgrid(i, j, indexing='ij')

    koo_x = ((i_grid - 3.5) / 4.95) * distance_matrix
    koo_y = distance_matrix
    koo_z = ((j_grid - 3.5) / 4.95) * distance_matrix

    all_points = np.stack([koo_x, koo_y, koo_z], axis=-1)

    transformed_points = rot.apply(all_points.reshape(-1, 3)) + np.array([x, y, z])

    parsed_line = transformed_points[distance_matrix.flatten() > 0]

    return parsed_line, sensor_id

def get_parsed_line(raw_line):
    line_items = raw_line.strip().split(' ')
    sensor_id = int(line_items[1])
    distance_array = np.array(line_items[2:2 + 64], dtype=np.float32)
    distance_matrix = distance_array.reshape((SENSOR_MATRIX_WIDTH, SENSOR_MATRIX_WIDTH))

    x, y, z = sensor_positions[sensor_id]
    rot = get_sensor_rot(sensor_id)

    i = np.arange(SENSOR_MATRIX_WIDTH)
    j = np.arange(SENSOR_MATRIX_WIDTH)
    i_grid, j_grid = np.meshgrid(i, j, indexing='ij')

    koo_x = np.tan(np.radians(60/7 * i_grid - 30)) * distance_matrix
    koo_y = distance_matrix
    koo_z = np.tan(np.radians(30 - 60/7 * j_grid)) * distance_matrix

    all_points = np.stack([koo_x, koo_y, koo_z], axis=-1)

    transformed_points = rot.apply(all_points.reshape(-1, 3)) + np.array([x, y, z])

    parsed_line = transformed_points[distance_matrix.flatten() > 0]

    return parsed_line, sensor_id

def add_measurement(measured_real_coordinates, sensor_id, measurement_time=None):

    measured_voxel_coordinates = get_voxel_coordinates(measured_real_coordinates)

    measurement_time = measurement_time if measurement_time else time.time()
    added_points = 0

    if ADD_UNCERTAINTY_SPHERE:
        all_new_voxels = set()

        sensor_coordinate = sensor_positions[sensor_id]

        for coordinate in measured_voxel_coordinates:

            distance = get_distance(coordinate, sensor_coordinate)
            print(f"distance for {coordinate} is {distance}")

            uncertainty_sphere_radius = get_uncertainty_radius(distance)
            print(f"radius for {coordinate} is {uncertainty_sphere_radius}")

            sphere_voxels = get_sphere_voxels(uncertainty_sphere_radius, coordinate)

            for voxel in sphere_voxels:
                if is_valid_point(voxel):
                    all_new_voxels.add(tuple(voxel))
                    added_points += 1

        all_new_voxels = np.array(list(all_new_voxels))

        for new_voxel in all_new_voxels:
            voxel_matrix[tuple(new_voxel)] = measurement_time

    else:
        for coordinate in measured_voxel_coordinates:
            if is_valid_point(coordinate):
                voxel_matrix[tuple(coordinate)] = measurement_time
                added_points += 1

    print(f"Updated {added_points} voxels at t={measurement_time - start_time}.")

def get_DBSCAN_clustering(voxels_to_scan):
    points = np.vstack(voxels_to_scan)

    clustering = DBSCAN(eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES).fit(points)
    return clustering

def visualize_plotly(points, sensor_positions, x_range=(0, 100), y_range=(0, 100), z_range=(0, 50)):
    fig = go.Figure()

    # Voxel-Punkte
    fig.add_trace(go.Scatter3d(
        x=points[:, 0],
        y=points[:, 1],
        z=points[:, 2],
        mode='markers',
        marker=dict(
            size=2,
            color=[voxel_matrix[int(x), int(y), int(z)] for x, y, z in points],  # Wert für jeden Punkt
            colorscale='Viridis',
            cmin=0,
            cmax=10,
            opacity=0.7
        ),
        name=f'Voxel ({len(points)})'
    ))

    # Sensoren
    fig.add_trace(go.Scatter3d(
        x=sensor_positions[:, 0]/VOXEL_GRID_SIZE,
        y=sensor_positions[:, 1]/VOXEL_GRID_SIZE,
        z=sensor_positions[:, 2]/VOXEL_GRID_SIZE,
        mode='markers',
        marker=dict(size=4, color='red', opacity=1.0, line=dict(color='darkgray', width=2)),
        name=f'Sensoren ({len(sensor_positions)})'
    ))

    fig.update_layout(
        title=f"3D Points: {len(points)} points",
        scene=dict(
            xaxis=dict(range=x_range, title='X'),
            yaxis=dict(range=y_range, title='Y'),
            zaxis=dict(range=z_range, title='Z'),
            bgcolor='lightgray',
            aspectmode='cube'
        )
    )
    fig.show()


all_detected_objects = []
class Detected_Object:
    """An object is defined by the voxels its made up of tracked over time."""

    def __init__(self, initial_voxels, current_time):
        all_detected_objects.append(self)
        self.listed_voxels_list = [initial_voxels]
        self.time_list = [current_time]

    def add_new_voxels(self, new_voxels, current_time):
        self.listed_voxels_list.append(new_voxels)
        self.time_list.append(current_time)

    def check_overlap(self, other_voxels, current_time):
        current_voxels = []
        for i, voxel_array in enumerate(self.listed_voxels_list):
            if current_time - self.time_list[i] < TIME_TOLERANCE:
                current_voxels.extend(voxel_array)

        set1 = set(map(tuple, current_voxels))
        set2 = set(map(tuple, other_voxels))
        return len(set1 & set2) > 0

    def clean(self, current_time):
        # To prevent clogging old measurements are removed here.
        # If there are no more voxels remaining, the object deletes the pointer to itself.

        filtered_data = [(voxels, measurement_time) for voxels, measurement_time in
                         zip(self.listed_voxels_list, self.time_list)
                         if current_time - measurement_time < TIME_TOLERANCE]

        if filtered_data:
            self.listed_voxels_list, self.time_list = zip(*filtered_data)
            self.listed_voxels_list = list(self.listed_voxels_list)
            self.time_list = list(self.time_list)
        else:
            all_detected_objects.remove(self)

    def check_nearby(self):
        pass

    def estimate_movement_vector(self, current_time):
        current_center = None
        previous_center = None

        for i in range(len(self.listed_voxels_list)):
            voxels = self.listed_voxels_list[i]
            if len(voxels) == 0:
                continue

            center = np.mean(voxels, axis=0)

            if self.time_list[i] == current_time:
                current_center = center
            elif current_time - self.time_list[i] < TIME_TOLERANCE:
                previous_center = center

        if current_center is not None and previous_center is not None:
            return current_center - previous_center
        else:
            return np.array([0, 0, 0])

serial_connection = serial.Serial(
    port='COM3',
    baudrate=460800,
    timeout=1
)

running = True
next_image_timer = start_time + IMAGE_TIMER
time_last_cycle = time.time()
while running:
    time_now = time.time()

    new_raw_line = serial_connection.readline().decode('utf-8').rstrip()
    parsed_line, sensor_id = get_parsed_line(new_raw_line)
    add_measurement(parsed_line, sensor_id, time_now)

    active_voxels = get_all_active(time_now)
    if active_voxels.size > 0:
        db_scan = get_DBSCAN_clustering(active_voxels)

        labels = db_scan.labels_

        cluster_labels = [label for label in np.unique(labels) if label != -1]

        for cluster_label in cluster_labels:
            voxels_of_this_cluster = active_voxels[labels == cluster_label]
            voxels_allocated = False

            for detected_object in all_detected_objects:
                if detected_object.check_overlap(voxels_of_this_cluster, time_now):

                    detected_object.add_new_voxels(voxels_of_this_cluster, time_now)

                    voxels_allocated = True
                    break
                    # Loop stops when voxels were successfully associated with an object.

            if not voxels_allocated:
                new_object = Detected_Object(voxels_of_this_cluster, time_now)
                print(f"New Object detected.")
                # If the voxels were not able to be allocated to an existing Object a new Object is detected.

        for detected_object in all_detected_objects:

            mean_displacement = detected_object.estimate_movement_vector(time_now)
            if mean_displacement != (0, 0, 0):
                velocity = mean_displacement / (time_now - time_last_cycle)
                print(f"Movement of Object detected: mean_displacement={mean_displacement}")

            detected_object.clean(time_now)


    if next_image_timer < time_now:
        next_image_timer = time_now + IMAGE_TIMER

        visualize_plotly(active_voxels, sensor_positions,
                         x_range=(0, VOXEL_MATRIX_SHAPE[0]),
                         y_range=(0, VOXEL_MATRIX_SHAPE[1]),
                         z_range=(0, VOXEL_MATRIX_SHAPE[2]))
    if time_now - start_time > 15:
        running = False

    time_last_cycle = time.time()

end_time = time.time()
print(f"Total time: {end_time - start_time:.6f} sec.")