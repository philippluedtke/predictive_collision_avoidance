""" This program estimates, based on the data collected by a VL53L5CX ToF-Sensor, if any detected objects will
    collide with the sensor. The data sent from the sensor must follow this pattern:

    <sensor_id> <data list> (everything should be seperated by a space)

    The data will first be processed via cluster-analysis. For this, the library "sklearn.cluster" is used.

    Then, the cluster-analysis is used to form groups of points which can be followed over time. This way the
    motion-vector can be determined. For this the NearestNeighbors-method from the "sklearn.neighbors" library
    is used.

    By using a rudimentary collision detection the program then prints out a traffic-light-like scale:
    0: No danger detected.
    1: No direct danger detected, but there is motion in the sensor-area.
    2: Danger detected. There is an object with a motion-vector on collision-course.

    If the scale reaches case 2, the program also prints out an vector, which correlates to the shortest
    way out of the collision if the sensor moves in that direction."""



''' Dieses Programm soll aus den Daten, die es von einem VL53L5CX ToF-Sensor erhält, abschätzen, ob Objekte mit dem
    Sensor kollidieren werden. Die Eingangsdaten müssen umbedingt diesem Muster folgen:

    <sensor_id> <Liste der Messdaten> (alles jeweils getrennt durch eine Leerstelle)

    Die Messdaten werden zunächst mittels Clusteranalyse zu Clustern gefügt.
    Dazu nutze ich die DBSCAN-Methode aus der Bibliothek "sclean_cluster".

    Die Clusteranalyse wird dann genutzt um Gruppen von Punkten zu erstellen, die dann über die Zeit verfolgt werden
    können. Daraus lässt sich dann ein Bewegungsvektor des Objekts bestimmen.
    Dazu nutze ich die die NearestNeighbors-Methode, auch aus "sclean_cluster".

    Durch eine rudimentäre Kollisionserkennung wird dann die "Gefahrenstufe" für den Sensor ermittelt.
    Diese unterteile ich in 3 Stufen auf die ich später näher eingehe, die in etwa so aussehen:
    0: Keine Gefahr
    1: Keine Gefahr, aber eine Bewegung wurde erkannt. (Möglicherweise andere Aktionen einleiten,
    geringere Geschwindigkeit oder ähnliches)
    2: Kollisionsgefahr erkannt. In diesem Fall könnte man eine Route berechnen, welche der Kollision ausweicht, da
    das Programm den ungefähren Bewegungsvektor des Objekts ermittelt.

    Diese Gefahrenstufen werden visuell als Ampel dargestellt.
    In der Visuellen Darstellung werden ausserdem die Punkte und die Position der Sensoren dargestellt.
    Die Sensoren sind die weissen Punkte, die Farbe der Messpunke varriert je nach Zustand (Position/Cluster).

    Während das Programm läuft kann man die größe der Darstellung mittels +/- ändern.
    Die Sensibilität der Clusteranalyse kann mit ./, angepasst werden.
    Mit p wird die Darstellung pausiert (z.B. für Screenshots). Das Programm läuft im Hintergrund weiter.'''

import serial
import numpy as np
import time

from scipy.spatial.transform import Rotation

from sklearn.cluster import DBSCAN
from sklearn.neighbors import NearestNeighbors

'''Before the start, the sensor-positions need to be put in manually:'''
# the positions of all sensors, in mm:
sensor_positions = np.array([
    [0.0, 0.0, 0.0]
])
# the rotation of all sensors, in degrees:
sensor_angles = np.array([
    [0.0, 90.0, 0.0]
])
# just for simulation purposes you can also put in velocities:
sensor_velocities = np.array([
    [0.0, 0.0, 0.0]
])
sensor_angular_velocities = np.array([
    [0.0, 0.0, 0.0]
])

eps = 70  # maximal distance between points in the same cluster
min_samples = 4  # minimal amount of points per cluster
cluster_eps = 50  # maximal distance between points in the same grouping
movement_vector_sensibility = 11  # minimal velocity to be detected in mm/s


mess_index = 0  # Counts all measurements
sensor_data = []  # Here, the measured data of each sensor is saved.
data_table = []  # Here, the measured data of all sensors is saved.
traffic_light = 0

# Now, the needed matrices are generated automatically
sensor_rot = []

def update_sensor_rot():
    global sensor_rot
    sensor_rot = []
    for angle in sensor_angles:
        sensor_rot.append(Rotation.from_euler('xyz', angle, degrees=True))

def sensor_movement(dt):
    """Here, sensor movement is calculated. If the sensor-data should be direct updated if the future,
    this is the method which could be easily updated."""
    global sensor_positions, sensor_angles

    sensor_positions += sensor_velocities * dt
    sensor_angles += sensor_angular_velocities * dt

    update_sensor_rot()

def start_new_scan(num_sensors):
    """This method prepares "sensor_data" to recieve new data."""
    scan_data = []
    for _ in range(num_sensors):
        scan_data.append(np.empty((0, 3)))
    sensor_data.append(scan_data)
    return len(sensor_data) - 1

def parse_com(line):
    """Hier wird eine über den serial-Port empfangene "line" als neues Messergebnis eingetragen.
       Zunächst wird der Sensorindex erkannt. Darüber kann die explizite Sensorposition
       und Drehung einbezogen werden. Dies geschieht dann in der Doppelschleife.
       Zuletzt wird das Messergebnis in das Dictionary "sensor_data" übertragen."""

    """This method parses a "line" (recieved via the serial-port) as new measured data. At first,
    the sensor-index is split off. Now the sensor position and rotation can be used to calcualte the
    true coordinates of the measured point. This is done for every point in the 8x8 matrix of the sensor.
    At last the result is saved in sensor_data."""

    measured_data = []
    # local array to structure the measured data. This will be saved to sensor_data in the end.

    line_items = line.strip().split(' ')
    sensor_id = int(line_items[1])
    # Each line must begin with the sensor-id.

    distance_array = line_items[-64:]
    x, y, z = sensor_positions[sensor_id]
    rot = sensor_rot[sensor_id]
    for i in range(8):
        for j in range(8):
            h = int(distance_array[i + j * 8])
            # By i and j the 8x8-Area is defined. The Sensor really measures the height (h) of a pyramid with sensor
            # at the top and the detected point somewhere at the bottom.

            koo = ((3.5 - i) / 4.95 * h, h, (3.5 - j) / 4.95 * h)
            # By basic geometry this point can be calculated. By now, it is still in the coordinate-system of the sensor.
            koo = rot.apply(koo)
            koo = (koo[0] + x, koo[1] + y, koo[2] + z)
            # Now, the point was rotated and moved to the "real" position by the general coordinate-system.

            measured_data.append(koo)

    sensor_data[mess_index][sensor_id] = measured_data


all_groupings = []  # All currently saved groupings.
class Grouping:
    """A grouping is a collection of points which have been identified as a coherent object. Under the assumption
    that they will continue to move as one there can be a movement-vector to describe that motion."""

    def __init__(self, orgin_points):
        all_groupings.append(self)
        self.grouped_points = orgin_points
        self.last_active = mess_index

        self.points_archive = [self.grouped_points]
        self.center_archive = [np.mean(self.grouped_points, axis=0)]

    def get_distance_to_point(self, point):
        """This gives the minimal distance from some point to the grouping."""
        grouped_points_array = np.array(self.grouped_points)  # In der Form (n, 3)
        point_array = np.array(point)  # In der Form (3)

        distances = np.linalg.norm(grouped_points_array - point_array, axis=1)
        min_index = np.argmin(distances)

        min_distance = distances[min_index]
        closest_point = self.grouped_points[min_index]

        return min_distance, closest_point

    def check_point(self, new_point):
        """Determines if a point is close enough to be added to the grouping,
        or if the point is maybe a "moved" point already in the grouping."""
        min_distance, closest_point = self.get_distance_to_point(new_point)
        if min_distance > eps * 1.1:
            return False
            # Point too far away.
        elif min_distance > cluster_eps:
            self.grouped_points = np.concatenate((self.grouped_points, [new_point]))
            self.last_active = mess_index
            return True
            # Point close enough, it is added to the grouping.
        else:
            closest_idx = np.where((self.grouped_points == closest_point).all(axis=1))[0][0]
            self.grouped_points[closest_idx] = new_point
            self.last_active = mess_index
            return True
            # Point is detected as an old point in the grouping which has moved.

    def check_self(self):
        """This method checks if the grouping is still continuous."""
        self_clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(self.grouped_points)
        self.grouped_points = self.grouped_points[self_clustering.labels_ == 0]
        if len(self.grouped_points) == 0:
            all_groupings.remove(self)
        else:
            # If everything is ok, the archive is expanded to allow to follow the motion.
            self.points_archive.append(self.grouped_points)

    def get_movement_vector(self):
        """This method estimates the movement-vector via Point-set registration.
        In testing I found a average noise of 6 to 10 mm/s."""
        try:
            points_prev = self.points_archive[-10]
        except IndexError:
            points_prev = self.points_archive[0]
        points_current = self.grouped_points

        nbrs = NearestNeighbors(n_neighbors=1).fit(points_prev)
        distances, indices = nbrs.kneighbors(points_current)

        mean_displacement = np.mean(points_current - points_prev[indices.flatten()], axis=0)
        # print(f"Vector: {mean_displacement} bei Messung: {mess_index}")
        return mean_displacement


def line_intersects_sphere(P0, d, M, r, return_vector = False):
    """
    This method was created using a LLM!
    It checks, if a given sphere and line intersect.

    Parameter:
        P0 : np.array([x, y, z]) - starting point of the line.
        d  : np.array([dx, dy, dz]) - vector of the line (normed)
        M  : np.array([mx, my, mz]) - center of the sphere
        r  : float - radius of the sphere

    Returns:
        bool - True if there is an intersection
    """

    v = M - P0
    proj = np.dot(v, d)

    if return_vector:
        closest_point = P0 + proj * d
        vector_to_center = M - closest_point
        return vector_to_center

    if proj < -r:
        return False
    dist_squared = np.dot(v, v) - proj ** 2

    return dist_squared <= r ** 2


def check_for_collision(group, movement_vector):
    for sensor_id in range(num_sensors):
        M = sensor_positions[sensor_id]
        r = 50 # This method checks if there will be an object colliding with a 50mm sphere around every sensor.
        """This is the rudimentary collision detection system."""


        relative_vector = movement_vector - sensor_velocities[0]

        norm_rel_vector = relative_vector / np.linalg.norm(relative_vector)

        for point in group.grouped_points:
            # Each point is checked if there would be a collision if the grouping would continue its current motion.
            if line_intersects_sphere(point, norm_rel_vector, M, r):
                # If a collision is detected, the

                center = np.mean(group.grouped_points, axis=0)

                vector_to_center = line_intersects_sphere(center, norm_rel_vector, M, r, return_vector=True)

                print(vector_to_center)
                return 2
    return 1

ser4 = serial.Serial(
    port='COM4',
    baudrate=460800,
    timeout=1
)

''' - - Main-Loop: - - '''

last_time = time.time()
timeout = time.time() + 1  # maximal time to collect data
paused = False

num_sensors = len(sensor_positions)
while mess_index < 1000:
    dt = time.time() - last_time
    last_time = time.time()

    sensor_movement(dt)

    line = ser4.readline().decode('utf-8').rstrip()
    start_new_scan(num_sensors)
    parse_com(line)

    punktwolke = np.vstack(sensor_data[mess_index])

    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(punktwolke)

    labels = clustering.labels_
    n_clusters = len(set(labels)) - (1 if -1 in labels else 0)

    ''' Management of the groupings:
        
        Here, every cluster foudn by the cluster-analysis is checked to be sorted into a grouping or not.'''

    for i in range(n_clusters):
        cluster_i_points = punktwolke[labels == i]
        group_found = False
        for group in all_groupings:
            if group.check_point(cluster_i_points[0]):
                for i in range(len(cluster_i_points) - 1):
                    group.check_point(cluster_i_points[i + 1])
                group_found = True
                break
        if not group_found:
            new_group = Grouping(cluster_i_points)
            # If no suitable grouping is found, the cluster forms a new grouping.

    ''' Collision analysis with traffic-light:'''

    traffic_light = 0

    for group in all_groupings:
        movement_vector = group.get_movement_vector()
        # Each movement is simplified in one movement-vector.

        if np.linalg.norm(movement_vector) > movement_vector_sensibility:
            # Only if the movement_vector is long enough, meaning the movement is fast enough, the program proceeds.
            # Otherwise, it is disregarded as noise.

            traffic_light = check_for_collision(group, movement_vector)

        group.check_self()
        if mess_index - group.last_active > 20:
            all_groupings.remove(group)
            # If no action is detected for a longer time, the grouping is discarded as not to interate over too many
            # groupings each measurement.

    print(traffic_light)

    mess_index += 1
