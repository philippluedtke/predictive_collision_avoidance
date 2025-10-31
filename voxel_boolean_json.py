import numpy as np
from sklearn.cluster import DBSCAN
from scipy.spatial.transform import Rotation
import time
import serial
import json

import pygame
import random

pygame.init()
screen = pygame.display.set_mode((800, 600))
screen_width, screen_height = screen.get_width(), screen.get_height()
PYGAME_SCALE = 5

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

def visualize_all_voxels_in_pygame():
    screen.fill(BLACK)

    current_matrix = voxel_matrices[-1]
    voxel_coordinates = np.argwhere(current_matrix > 0)

    for voxel in voxel_coordinates:

        '''screen_koo = (screen_width/4 + (x + y*2/3) * PYGAME_SCALE,
                      screen_height/4 + (z + y*1/3) * PYGAME_SCALE)

        pygame.draw.circle(screen, WHITE, screen_koo, 1)'''

        pygame_draw_isometric(voxel)

    pygame.display.flip()

def visualize_objects_in_pygame():
    screen.fill(BLACK)

    for this_object in all_detected_objects:

        voxel_coordinates = this_object.listed_voxels_list[-1]

        for voxel in voxel_coordinates:
            pygame_draw_isometric(voxel, this_object.pygame_color)

    pygame.display.flip()

def pygame_draw_isometric(voxel_koo, color=None):
    x, y, z = voxel_koo
    if color is None: color = WHITE

    # X-View:
    screen_koo = (screen_width * 1 / 8 + y * PYGAME_SCALE,
                  screen_height * 1 / 8 + z * PYGAME_SCALE)
    pygame.draw.circle(screen, color, screen_koo, 1)

    # Y-View:
    screen_koo = (screen_width * 5 / 8 + x * PYGAME_SCALE,
                  screen_height * 1 / 8 + z * PYGAME_SCALE)
    pygame.draw.circle(screen, color, screen_koo, 1)

    # Z-View:-**
    screen_koo = (screen_width * 5 / 8 + x * PYGAME_SCALE,
                  screen_height * 5 / 8 + y * PYGAME_SCALE)
    pygame.draw.circle(screen, color, screen_koo, 1)


start_time = time.time()

MAX_RUNTIME = 60
# Defines how long the program will run before aborting, for testing purposes.

VOXEL_GRID_SIZE = 50
# Defines the real distance between each voxel-center in mm.

WORK_ENVIRONMENT_SIZE = np.array([1500, 1400, 1600])
# Defines the real dimensions of the environment where the robot operates in mm.
# This must be defined beforehand, as the matrix in which all values are stored
# will be defined by these dimensions and the grid size.

VOXEL_MATRIX_SHAPE = (WORK_ENVIRONMENT_SIZE / VOXEL_GRID_SIZE).astype(int)
voxel_matrices = []
# The voxels are stored in this sparse matrix. This way, performance is increased,
# but nothing outside the work environment can be detected.

# The positions in mm and directions in degrees of all Sensors are stored here.
# Later, this information would be updated as the robot moves.

sensor_positions = np.array([
    [760.0, 700.0, 800],
    [737.410, 746.908, 800],
    [686.650, 758.494, 800],
    [645.946, 726.034, 800],
    [645.946, 673.966, 800],
    [686.650, 641.506, 800],
    [737.410, 653.092, 800],
])

sensor_rotation = np.array([
    [0.0, 0.0, 0.0],
    [0.0, 0.0, 51.43],
    [0.0, 0.0, 102.86],
    [0.0, 0.0, 154.29],
    [0.0, 0.0, 205.71],
    [0.0, 0.0, 257.14],
    [0.0, 0.0, 308.57],
])

ALL_SENSOR_ROTATION = np.array([
    [0.0, 0.0, 135.0]
])
for i in range(len(sensor_rotation)):
    sensor_rotation[i] = sensor_rotation[i] + ALL_SENSOR_ROTATION
# This can adjust the rotation of all sensors for imporoved confirmability.

SENSOR_MATRIX_WIDTH = 8
# The VL53L7CX Sensor outputs an 8x8 Matrix. If the 4x4 option would be used, it could be changed here.

DBSCAN_EPS = 4
DBSCAN_MIN_SAMPLES = 5
# These variables define how the DBSCAN algorithm works:
# eps is the highest distance of two points before they are no longer grouped together.
# min_samples is the minimal amount to voxels needed to be detected as an object.
# These values were found by trial and error. Other values can the used, but we found
# that eps below 3.5 result in too many incorrect movement vectors.

TOTAL_VELOCITY_TOLERANCE = 150
# Defines the minimum velocity at which the program outputs a found velocity, everything lower will be regarded as an error.

OBJECT_DISTANCE_TOLERANCE = 125
# Defines the minimum velocity at which the program outputs a found velocity, everything lower will be regarded as an error.

OBJECT_LIFESPAN = 1
# Defines how long an object is saved after the last update in seconds.


def get_voxel_coordinates(real_coordinate):
    """Returns the voxel-coordinates of given real coordinates."""
    voxel_coordinate = np.round(real_coordinate / VOXEL_GRID_SIZE)
    return voxel_coordinate.astype(int)
def get_real_coordinate(voxel_coordinate):
    """Returns the real center coordinates of given voxel-coordinates."""
    real_coordinate = voxel_coordinate * VOXEL_GRID_SIZE
    return real_coordinate
def get_sensor_rot(sensor_id):
    """Returns the Euler-Rotation-matrices for a given sensor.
    It draws on the already defined sensor-rotations matrix."""
    angles = sensor_rotation[sensor_id]
    euler_rotation_matrix = Rotation.from_euler('xyz', angles, degrees=True)
    return euler_rotation_matrix

def is_valid_point(check_point):
    """Checks if a given voxel coordinate is inside the work environment."""
    return all(0 <= coord < max_dim for coord, max_dim in zip(check_point, VOXEL_MATRIX_SHAPE))


def read_and_input_new_data(json_input):
    """Reads the given json-data and updates the voxel_matrices.
     This is preset for the ring with seven sensors."""
    SENSORS_ON_RING = 7

    json_data = json.loads(json_input)

    this_frame_matrix = np.zeros(VOXEL_MATRIX_SHAPE, dtype=np.float32)

    if not isinstance(json_data, dict):  # Ensure the data is a dictionary
        print(f"json is not a dictionary: {json_data}")
        return None

    for sensor_id in range(SENSORS_ON_RING):
        sensor_name = f"sensor{sensor_id}"

        distance_array = np.array([item[0] for item in json_data[sensor_name]])

        distance_matrix = distance_array.reshape((SENSOR_MATRIX_WIDTH, SENSOR_MATRIX_WIDTH))

        x, y, z = sensor_positions[sensor_id]
        rot = get_sensor_rot(sensor_id)

        i = np.arange(SENSOR_MATRIX_WIDTH)
        j = np.arange(SENSOR_MATRIX_WIDTH)
        i_grid, j_grid = np.meshgrid(i, j, indexing='ij')

        koo_x = np.tan(np.radians(60 / 7 * i_grid - 30)) * distance_matrix
        koo_y = distance_matrix
        koo_z = np.tan(np.radians(30 - 60 / 7 * j_grid)) * distance_matrix

        all_points = np.stack([koo_x, koo_y, koo_z], axis=-1)

        transformed_points = rot.apply(all_points.reshape(-1, 3)) + np.array([x, y, z])

        parsed_line = transformed_points[distance_matrix.flatten() > 0]

        this_frame_matrix = add_measurement(parsed_line, this_frame_matrix)

    voxel_matrices.append(this_frame_matrix)

def add_measurement(measured_voxels, matrix):
    """Adds new voxels to a given matrix."""

    measured_voxel_coordinates = get_voxel_coordinates(measured_voxels)

    added_points = 0

    for coordinate in measured_voxel_coordinates:
        if is_valid_point(coordinate):
            matrix[tuple(coordinate)] = 1
            added_points += 1

    return matrix

    #print(f"Updated {added_points} voxels at t={measurement_time - start_time}.")

def get_DBSCAN_clustering(voxels_to_scan):
    """Returns the clustering the DBSCAN of a given list of voxels."""
    points = np.vstack(voxels_to_scan)

    clustering = DBSCAN(eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES).fit(points)
    return clustering


def dilate_voxels(voxels, tolerance):
    """Expands a voxel-volume by the given tolerance.
    This is used to more efficiently check if a grop of voxels is near another group of voxels
    in a process called Morphological Dilation.
    This function was created by AI."""

    dilated_set = set()

    tolerance_voxels = int(np.ceil(tolerance / VOXEL_GRID_SIZE))

    for voxel in voxels:

        for dx in range(-tolerance_voxels, tolerance_voxels + 1):
            for dy in range(-tolerance_voxels, tolerance_voxels + 1):
                for dz in range(-tolerance_voxels, tolerance_voxels + 1):

                    real_distance = np.sqrt((dx * VOXEL_GRID_SIZE) ** 2 +
                                            (dy * VOXEL_GRID_SIZE) ** 2 +
                                            (dz * VOXEL_GRID_SIZE) ** 2)

                    if real_distance <= tolerance:
                        new_voxel = (voxel[0] + dx, voxel[1] + dy, voxel[2] + dz)
                        dilated_set.add(new_voxel)

    return np.array(list(dilated_set))


all_detected_objects = []
detected_object_counter = 0
class Detected_Object:
    """An object is defined by the voxels its made up of tracked over time."""

    def __init__(self, initial_voxels):
        all_detected_objects.append(self)
        self.listed_voxels_list = [initial_voxels]
        self.last_time_updated = time.time()

        self.TIME_OF_CREATION = self.last_time_updated
        global detected_object_counter
        self.ID = detected_object_counter
        detected_object_counter += 1
        self.pygame_color = (random.randint(0, 255),
                             random.randint(0, 255),
                             random.randint(0, 255))

        #print(f"t = {(time_now - start_time):.2f}s: New Object '{self.ID}' created with {len(initial_voxels)} voxels.")


    def add_new_voxels(self, new_voxels):
        """Used to add new voxels to this object.
        PROBLEM: If there was already a group of voxels added in this timeframe they get pushed back (in the list) and
        the voxels are then regarded as noise."""

        unique_voxels = np.unique(new_voxels, axis=0)
        self.listed_voxels_list.append(unique_voxels)

        #print(f"t = {(time_now - start_time):.2f}s: Updated Object '{self.ID}' with {len(new_voxels_list)} voxels.")

        self.last_time_updated = time.time()

    def check_overlap(self, other_voxels):
        """Used to check for overlap between a given list of voxels and the previous volume of voxels of this object.
        This way, objects can be traced through time."""

        current_voxels = self.listed_voxels_list[-1]

        set1 = set(map(tuple, current_voxels))
        set2 = set(map(tuple, other_voxels))

        if len(set1 & set2) > 0:
            #print(f"t = {(time_now - start_time):.2f}s: Object '{self.ID}': Overlap detected!")
            return True
        else:
            return False

    def check_nearby(self, other_voxels):
        """Used to check if a given list of voxels is near this object.
        It uses the aforementioned dilate_voxels-function."""

        current_voxels = self.listed_voxels_list[-1]

        if len(current_voxels) == 0 or len(other_voxels) == 0:
            return False

        dilated_voxels = dilate_voxels(current_voxels, OBJECT_DISTANCE_TOLERANCE)

        set1 = set(map(tuple, dilated_voxels))
        set2 = set(map(tuple, other_voxels))

        if len(set1 & set2) > 0:
            #print(f"t = {(time_now - start_time):.2f}s: Object '{self.ID}': New Voxels detected nearby!")
            return True
        else:
            return False


    def clean(self):
        """To prevent clogging old measurements are removed here.
        If there was no update to the object after a given amount of time (OBJECT_LIFESPAN) it gets deleted."""

        time_now = time.time()
        if time_now > self.last_time_updated + OBJECT_LIFESPAN:
            all_detected_objects.remove(self)
            #print(f"t = {(time_now - start_time):.2f}s: Object '{self.ID}' deleted after {(time_now - self.TIME_OF_CREATION):.2f}s.")

    def estimate_movement_vector(self):
        """Estimates the movement vector of the given object, using the last two added voxel-volumes.
        THIS SHOULD ALSO BE IMPROVED UPON!"""

        if len(self.listed_voxels_list) < 2:
            return np.array([0, 0, 0])

        current_center = np.mean(self.listed_voxels_list[-1], axis=0)
        previous_center = np.mean(self.listed_voxels_list[-2], axis=0)


        if current_center is not None and previous_center is not None:
            return current_center - previous_center
        else:
            return np.array([0, 0, 0])

try:
    serial_connection = serial.Serial(
        port='COM6',
        baudrate=460800,
        timeout=1
    )
except Exception as e:
    print(f"Serial connection error: {e}")
    exit(1)

all_cycle_times = []

running = True
time_last_cycle = time.time()
print(f"t = {(time_last_cycle - start_time):.2f}s: Main loop starting now!")
while running:
    time_now = time.time()

    new_raw_line = serial_connection.readline().decode('utf-8').rstrip()
    read_and_input_new_data(new_raw_line)

    active_voxels = np.column_stack(np.where(voxel_matrices[-1] != 0))

    db_scan = get_DBSCAN_clustering(active_voxels)

    labels = db_scan.labels_

    cluster_labels = [label for label in np.unique(labels) if label != -1]

    for cluster_label in cluster_labels:
        voxels_of_this_cluster = active_voxels[labels == cluster_label]
        voxels_allocated = False

        for detected_object in all_detected_objects:
            # First it checks if there is overlap with an already existing object.
            if detected_object.check_overlap(voxels_of_this_cluster):

                detected_object.add_new_voxels(voxels_of_this_cluster)
                voxels_allocated = True
                break
                # Loop stops when voxels were successfully associated with an object.

        if not voxels_allocated:
            for detected_object in all_detected_objects:
                # If there was no overlap there could be a object nearby.
                if detected_object.check_nearby(voxels_of_this_cluster):

                    detected_object.add_new_voxels(voxels_of_this_cluster)
                    voxels_allocated = True
                    break
                    # Loop stops when voxels were successfully associated with an object.

        if not voxels_allocated:
            new_object = Detected_Object(voxels_of_this_cluster)
            # If the voxels could not be allocated to an existing Object a new Object is detected.

    for detected_object in all_detected_objects:
        detected_object.clean()

        mean_displacement = detected_object.estimate_movement_vector()
        if mean_displacement.any():
            velocity_vector = mean_displacement / time_last_cycle
            total_velociy = np.linalg.norm(velocity_vector)

            if total_velociy > TOTAL_VELOCITY_TOLERANCE:
                print(f"t = {(time_now - start_time):.2f}s: Movement of Object detected: v={velocity_vector}mm/s ({total_velociy}mm/s)")

    time_last_cycle = time.time() - time_now
    #print(f"Cylce complete. Time={time_last_cycle}")
    all_cycle_times.append(time_last_cycle)

    if time_now - start_time > MAX_RUNTIME:
        running = False

    visualize_objects_in_pygame()

end_time = time.time()
print(f"Runtime over. Real total time: {end_time - start_time:.2f} sec.")
print(f"Ran over {len(all_cycle_times)} cycles with an average time of {sum(all_cycle_times) / len(all_cycle_times):.4f} seconds per cycle.")