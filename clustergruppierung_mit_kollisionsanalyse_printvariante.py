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

'''Hier müssen zu Beginn die Sensorpositionen und Drehungen eingestellt werden!'''
'''Theoretisch könnte mann später die Bewegungsdatenarrays zu Eingangsgrößen machen und so die realen Begewungen abbilden.'''
# Die Position der Sensoren, in mm:
sensor_positions = np.array([
    [0.0, 0.0, 0.0]
])
# Die Drehung der Sensoren, in Grad:
sensor_angles = np.array([
    [0.0, 90.0, 0.0]
])
# Etwaige Geschwindikeiten der Sensoren, in mm/s:
# Diese können sich auch im Verlauf des Programms ändern
sensor_geschwindigkeiten = np.array([
    [0.0, 0.0, 0.0]
])
# Etwaige Winkelgeschwindigkeiten der Sensoren, in Grad/s:
# Auch diese können sich auch im Verlauf des Programms ändern
sensor_winkelgeschwindigkeiten = np.array([
    [0.0, 0.0, 0.0]
])

# Hier werden automatisch die Rotationsmatrizen eingestellt
# Das Programm verwendet dann die Rotationsmatrizen und nicht den sensor_angles-Array
sensor_rot = []


def update_sensor_rot():
    global sensor_rot
    sensor_rot = []
    for angle in sensor_angles:
        sensor_rot.append(Rotation.from_euler('xyz', angle, degrees=True))


def sensor_movement(dt):
    '''
    Hier wird die Bewegung der Sensoren abgehandelt.
    Die länge des Zeitschritts dt wird übergeben, so werden die Geschwindigkeiten korrekt in verrechent.
    Am schluss wird auch der neue Rotationsarray berechnet.
    '''
    global sensor_positions, sensor_angles

    sensor_positions += sensor_geschwindigkeiten * dt
    sensor_angles += sensor_winkelgeschwindigkeiten * dt

    update_sensor_rot()  # Die Rotation wird angepasst


sensor_data = []  # Hier werden die aktuellen Messdaten jedes Sensors gespeichert.


def start_new_scan(num_sensors):
    ''' Diese Funktion bereitet sensor_data auf den Empfang einer neuen Messung vor. '''
    scan_data = []
    for _ in range(num_sensors):
        scan_data.append(np.empty((0, 3)))
    sensor_data.append(scan_data)
    return len(sensor_data) - 1


mess_index = 0  # Gibt die anzahl der Messungen seit beginn an.
data_table = []  # Hier werden alle Messdaten aller Sensoren gespeichert.


def parse_com(line):
    '''Hier wird eine über den serial-Port empfangene "line" als neues Messergebnis eingetragen.
       Zunächst wird der Sensorindex erkannt. Darüber kann die explizite Sensorposition
       und Drehung einbezogen werden. Dies geschieht dann in der Doppelschleife.
       Zuletzt wird das Messergebnis in das Dictionary "sensor_data" übertragen.'''

    messergebnis = []
    # Lokales Array um das Messergebnis zu erstellen. Wird am schluss in sensor_data übertragen.

    line_items = line.strip().split(' ')
    sensor_id = int(line_items[1])
    # Zu beginn jedes Datensatzes, der (vom Pico) gesendet wird, steht die Sensor-id.
    # Dies muss entsprechend vorher eingestellt worden sein.

    distance_array = line_items[-64:]
    x, y, z = sensor_positions[sensor_id]
    rot = sensor_rot[sensor_id]
    # Nun wurden die nötigen Daten gesammelt um die tatsächliche Punktwolke zu gewinnen.
    for i in range(8):
        for j in range(8):
            h = int(distance_array[i + j * 8])
            # Durch i und j wird das 8x8-Gebiet definiert. Der Sensor gibt den Abstand als Höhe (h) einer Pyramide an.

            koo = ((3.5 - i) / 4.95 * h, h, (3.5 - j) / 4.95 * h)
            # Durch die Pyramidengeometrie lassen sich die x,y,z-Koordinaten als Funktion der Höhe berechnen.
            # Dieser Punkt ist noch in einem Koordniatensystem definiert welches sich an der "Sicht" des Sensors ausrichtet.
            koo = rot.apply(koo)
            # Jetzt wurde der Punkt in das "allgemeine" Koordinatensystem gedreht.
            koo = (koo[0] + x, koo[1] + y, koo[2] + z)
            # Zuletzt wird der Punkt noch an die richtige Stelle verschoben.

            messergebnis.append(koo)
            # Die Messung wird dem Messergebnis angehängt
    sensor_data[mess_index][sensor_id] = messergebnis
    # Zuletzt wird die Messung an die ensprechende Stelle im sensor_data-Array eingefügt.


all_groupings = []  # Alle aktuell erkannten Gruppen.
class Grouping:
    '''
    Eine Gruppe ist eine Sammlung von Punkten, welche als zusammenhängedes Objekt erkannt wurde.
    Es wird davon ausgegangen, dass diese sich auch in Zunkuft als gleichartige Gruppe fortbewegen
    und man ihnen daher einen Bewegungsvektor zuordnen kann.
    '''

    def __init__(self, orgin_points):
        all_groupings.append(self)
        self.grouped_points = orgin_points
        self.last_active = mess_index

        self.points_archive = [self.grouped_points]
        self.center_archive = [np.mean(self.grouped_points, axis=0)]

    def get_distance_to_point(self, point):
        '''Gibt die minimale Distanz mit dazugehörigen Punkt zum Cluster zurück'''
        grouped_points_array = np.array(self.grouped_points)  # In der Form (n, 3)
        point_array = np.array(point)  # In der Form (3)

        distances = np.linalg.norm(grouped_points_array - point_array, axis=1)
        min_index = np.argmin(distances)

        min_distance = distances[min_index]
        closest_point = self.grouped_points[min_index]

        return min_distance, closest_point

    def check_point(self, new_point):
        '''Beurteilt, ob ein gegebener Punkt nah genug an der Gruppe ist, um neu aufgenommen zu werden bzw.
        ob der Punkt ein bereits bestehenden Punkt innerhalb der Gruppe aktualisiert'''
        min_distance, closest_point = self.get_distance_to_point(new_point)
        if min_distance > eps * 1.1:
            return False
            # Punkt zu weit entfernt.
        elif min_distance > cluster_eps:
            self.grouped_points = np.concatenate((self.grouped_points, [new_point]))
            self.last_active = mess_index
            return True
            # Punkt nah genug, er wird der Gruppe hinzugefügt.
        else:
            closest_idx = np.where((self.grouped_points == closest_point).all(axis=1))[0][0]
            self.grouped_points[closest_idx] = new_point
            self.last_active = mess_index
            return True
            # Der Punkt aktualisiert und ersetzt einen alten, bereits in der Gruppe enhaltenen Punkt.

    def check_self(self):
        '''Diese Methode überprüft ob die Gruppe noch zusammenhängend ist, und entfernt etwaige Reste.'''
        self_clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(self.grouped_points)
        self.grouped_points = self.grouped_points[self_clustering.labels_ == 0]
        if len(self.grouped_points) == 0:
            all_groupings.remove(self)
        else:
            # Wenn alles in Ordnung ist, wird das Archiv erweitert um die Bewegung nachvollziehen zu können.
            self.points_archive.append(self.grouped_points)

    def get_movement_vector(self):
        '''Hier wird die geschätze Bewegung mittels Point-set registration ermittelt.
           Ich habe eine mittleres Rausches von cirka 6 bis 10 mm/s festgestellt.'''

        try:
            points_prev = self.points_archive[-10]
        except IndexError:
            points_prev = self.points_archive[0]
        points_current = self.grouped_points

        # Finde nächste Nachbarn zwischen den Frames
        nbrs = NearestNeighbors(n_neighbors=1).fit(points_prev)
        distances, indices = nbrs.kneighbors(points_current)
        # Berechne Durchschnittsverschiebung der nächsten Nachbarn
        mean_displacement = np.mean(points_current - points_prev[indices.flatten()], axis=0)
        # print(f"Vector: {mean_displacement} bei Messung: {mess_index}")
        return mean_displacement


def strahl_schneidet_kugel(P0, d, M, r):
    """
    Mittels Deepseek erstellt!
    Prüft, ob ein Strahl eine Kugel schneidet.

    Parameter:
        P0 : np.array([x, y, z]) - Startpunkt des Strahls
        d  : np.array([dx, dy, dz]) - Richtungsvektor (normiert)
        M  : np.array([mx, my, mz]) - Kugelmittelpunkt
        r  : float - Radius der Kugel

    Returns:
        bool - True wenn Schnittpunkt existiert
    """
    # Vektor von P0 zum Kugelmittelpunkt
    v = M - P0

    # Projektion von v auf d (Skalarprodukt, da d normiert ist)
    proj = np.dot(v, d)

    if proj < -r:  # Wenn M "hinter" P0 liegt (mit Sicherheitsabstand r)
        return False

    # Minimaler Abstand zwischen Gerade und M
    dist_squared = np.dot(v, v) - proj ** 2

    return dist_squared <= r ** 2


def check_for_collision(group, movement_vector):
    for sensor_id in range(num_sensors):
        M = sensor_positions[sensor_id]
        r = 50
        # 50mm Kugel um den Sensor.
        # Für eine komplexer Kollisionerkennung würde man hier anpacken müssen.

        relative_vector = movement_vector - sensor_geschwindigkeiten[0]
        # Falls der Sensor sich selbst bewegt wird hier die enstprechende Korrektur vorgenommen

        norm_rel_vector = relative_vector / np.linalg.norm(relative_vector)
        # Der Vektor wird normiert, damit die strahl_schneidet_kugel-Funktion funktioniert.

        for point in group.grouped_points:
            if strahl_schneidet_kugel(point, norm_rel_vector, M, r):
                # Es wird für jeden Punkt in der Gruppe überprüft ob er,
                # wenn die Bewegung fortgesetzt werden würde, mit dem Sensor kollisieren würde.
                return 2
                # Wenn eine Kollision erkannt wird wird das Warnlicht auf 2 gesetzt.
    return 1
    # Wenn keine Kollision erkannt wird wird das Warnlicht auf 1 gesetzt,
    # da trotzdem eine Bewegung erkannt wurde (so wurde die Funktion aufgerufen).


# Serielle Verbindung konfigurieren
ser4 = serial.Serial(
    port='COM4',
    baudrate=460800,
    timeout=1
)

''' - - - Globale Variablen: - - - '''

scale = 1  # Skalierbare Anzeigenvergößerung.
eps = 70  # Maximaler Abstand zwischen Punkten im selben Cluster.
min_samples = 4  # Minimale Anzahl von Punkten für ein Cluster.
cluster_eps = 50  # Minimaler Abstand, um einen Punkt in einem Grouping zu ersetzen anstatt einen neuen hinzuzufügen.
movement_vector_sensibility = 11  # Ab welcher Geschwindigkeit die Bewegung als solche erkannt wird.
warnlicht = 0  # Gibt in Stufen die "gefährlichkeit" der erkannten Bewegungen an.

''' - - Datenerhebung (Main-Loop): - - '''

last_time = time.time()
timeout = time.time() + 1  # Maximale Messzeit
running = True
paused = False

num_sensors = len(sensor_positions)
while running:
    dt = time.time() - last_time
    last_time = time.time()
    # dt ist die vergangene Zeit seit dem letzten Durchlauf in Sekunden.

    # Simulation der Bewegung:
    sensor_movement(dt)

    # Lesen des serial-ports zu Übertragung der Sensordaten:
    line = ser4.readline().decode('utf-8').rstrip()
    start_new_scan(num_sensors)
    parse_com(line)

    punktwolke = np.vstack(sensor_data[mess_index])
    # Umwandlung des np-Arrays

    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(punktwolke)
    # Clusteranalyse mittels DBSCAN-Methode

    labels = clustering.labels_
    n_clusters = len(set(labels)) - (1 if -1 in labels else 0)

    ''' Verwaltung der Gruppen:

        Hier wird geprüft, ob die Punkte, welche durch die Clusteranalyse nah bei einander liegen zusammen Gruppen 
        bilden können bzw. einer bestehenden Gruppe hinzugefügt werden können.'''

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
            # Wird keine Gruppe gefunden, wird eine neue mit dem Punkt als Ursprung erstellt.
            # Im nächsten Durchlauf werden passende andere Punkte dann dieser Gruppe hinzugefügt.

    ''' Kollisionsanalyse mit Warnlicht:

        Das Warnlicht gibt in 3 Stufen an, wie "gefährlich" die Bewegung eigeschätzt wird:
        - 0: Keine Bewegung erkannt.
        - 1: Bewegung erkannt, sie wird jedoch als ungefährlich eingeschätzt.
        - 2: Bewegung erkannt und der Bewegungsvektor kollidiert mit der Sensorposition.'''

    warnlicht = 0

    for group in all_groupings:
        # Es wird über alle erkannten Gruppen interiert.

        movement_vector = group.get_movement_vector()
        # Die Bewegung der Gruppe wird als Vektor welcher in jedem Punkt der Gruppe angreift vereinfacht.

        if np.linalg.norm(movement_vector) > movement_vector_sensibility:
            # Wenn die länge des Bewegunsvektors ein gewisse Größe erreicht hat, wird eine Beweguns erkannt.
            # Ansonsten wird sie als Rauschen verworfen.

            warnlicht = check_for_collision(group, movement_vector)
            # Das Warnlicht wird über die check_for_collision-Methode eigestellt.


        group.check_self()
        if mess_index - group.last_active > 20:
            all_groupings.remove(group)
            # Wenn eine Gruppe längere Zeit inaktiv ist (d.h. keine Punkte wurden ihr zugeordet oder akutalisiert)
            # wird sie gelöscht, um die all_groupings-Liste möglichst klein zu halten, da in jedem durchlauf über jeden
            # Punkt in jeder Gruppe iteriert wird.
    
    print(warnlicht)
    #Anstelle der Darstellung über pygame wird hier nur der "wert" des Warnlichts ausgegeben um die performance zu verbessern

    mess_index += 1
    # Der Messindex wird erhöht und es folgt die nächste Messung wenn die while-Schleife nicth abgrebrochen wurde.