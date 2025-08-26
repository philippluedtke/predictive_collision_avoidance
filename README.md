# Über das Projekt



# Über die Dateien

Der "streamlined_ubertragung"-Code ist für den Pico. Falls mehrere Sensoren verwendet werden muss nur sichergestellt werden
dass das Format der Datenübertragung das selbe ist und ggf. die Sensor-id angepasst werden, da ich diese im Code festlege.

Der "clusteranalyse_mit_kollisionsanalyse"-Code ist das eigentliche Programm.
Wahrscheinlich muss man den Port anpassen (Standard ist COM4) wenn man das Programm ausprobieren will.

Die "clusteranalyse_mit_kollisionsanalyse_printvariante" ist eine andere Version des selben Programms, wobei die Visuelle Darstellung eingespart wurde.
Stattdessen wird das "Warnlicht" über die Konsole ausgegeben, was die Performance verbessert.


# Literatur-Sammlung:
  # Repo
    https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver?utm_source=chatgpt.com (Universal_Robots_ROS2_Driver)
    https://github.com/isl-org/Open3D (Open 3D)
    https://github.com/PointCloudLibrary/pcl (Point Cloud Libary)
    https://github.com/yanx27/Pointnet_Pointnet2_pytorch (PointNet ++, PointNeXT wurde rausgezogen)
    https://github.com/charlesq34/votenet-1 (Deep Hough Voting for 3D Object Detection in Point Clouds)

  # Paper
    https://arxiv.org/pdf/2208.07678 (FEC: Fast Euclidean Clustering for Point Cloud Segmentation)
    https://arxiv.org/pdf/2412.04649 (Generating Whole-Body Avoidance Motion through Localized Proximity Sensing)
    https://www.vision.rwth-aachen.de/media/papers/know-what-your-neighbors-do-3d-semantic-segmentation-of-point-clouds/W63P26.pdf (Know What Your Neighbors Do: 3D Semantic     Segmentation of Point Clouds)
    https://arxiv.org/pdf/2405.11903 (A comprehensive overview of deep learning techniques for 3D point cloud classification and semantic segmentation)
    https://arxiv.org/pdf/1812.05784 (PointPillars: Fast Encoders for Object Detection from Point Clouds)
    https://openlib.tugraz.at/download.php?id=5f6b335524db8&location=browse&utm_source=chatgpt.com (360° Monitoring for Robots Using Time-of-Flight Sensors)
    https://biorobotics.ri.cmu.edu/papers/paperUploads/Abah_Multi-modal_2019.pdf (A Multi-modal Sensor Array for Safe Human-Robot Interaction and Mapping)

  # Andere Formate
    https://medium.com/@BasicAI-Inc/3d-point-cloud-segmentation-guide-a073b4a6b5f3 (Introduction to 3D Point Cloud Segmentation)
    https://forum.universal-robots.com/t/eye-in-hand-camera-calibration/35922?utm_source=chatgpt.com (Eye-in-hand camera calibration)
