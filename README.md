# About the Project

Objective of the project is the development of a reactive collision avoidance system for a colloborative robot.

# About the Code

"streamlined_ubertragung" is used for the Pico. If more than one sensor is used the sensor-id variable should be adjusted.

"clusteranalysis_with_grouping_printvariant" is the main program. The port (in the code COM4 is used) will probably have to be adjusted.

"voxel_decay_time" is the first voxel-based program. It also uses the time the voxel was measured to determine its status, altough the "decay"-part is not really up-to-date anymore.

# Hardware 
- Microcontroller: Raspberry Pi Pico (RP2040)
- Sensor: VL53L7CX ToF (8x8 grid) (capabale of IR)
- Mounting: One ring made out of six sensors (with dead zones)
- Robotic arm: UR10
- https://ams-osram.com/products/sensor-solutions/direct-time-of-flight-sensors-dtof/ams-tmf8829-48x32-multi-zone-time-of-flight-sensor (leider nicht bestellbar)


# How the Program is structured

The program can be structured is these steps:
1. Collection of the Sensor-data. (This is not really part of this project)
2. Storage of the collected data. There are exact (point-cloud) and discrete (voxels) options. Basically, the exact options have less built-in error, but the discrete options offer far better performance which is critical when dealing with object or motion detection. Therefore, voxels seem like the best option. As we want to not only detect objects but also the movement of these objects we will have to track these objects over time. There are, again, two main options: We could store each timeframe is a different grid, for example by giving our points an extra time-coordinate or creating a new grid for every new timeframe, or we could store all points in the same grid but give the points themselves the needed information. The first option basically gives us a history of all recieved data, but the management can become difficult. The second option restricts the size of the stored data significantly, again trading performance for excactness.
3. First analysis of the stored data to find objects. This can be done by cluster-analysis. The DBSCAN-algorythm is good compromise between performance and accuracy, as its bascially the simplest algorythm which can detect overlapping objects (for example a ring with a different object inside).
4. The detected objects or groups of points must now be stored to use them for motion-detection later. Here, we seem to have two options: We can store all points assosiated with the object, or simplify the object to one point, for example the center. Obviously, the latter would be far simpler, but the first one has critical advantages: Firstly, the Object can be tracked more easily. Secondly, the dimensions of the object are preserved. This is particulary important, as some objects could otherwise become impossible to distinguish (the ring for example would have the same center as the object inside). We must therefore store all associated points.
5. Tracking of the detected objects. This is arguabely the most difficult part, as we now have to use the data from different timeframes. There are many different options, but so far we`ve explored two: By checking the overlap of the different detected objects, we can track the object through the different time frames. This works reasonably well, but is quite resource-intensive and does not work well with highter velocities. The other option takes advantage of the way information can the stored when using voxels. Usually, when using voxels, a (volumetric-)pixel is associated with a boolean true/false state. By instead storing, for example, the time at which the point was last detected, the pixel automatically becomes active troughout time. A moving object creates a tail-like structure behind itsself by which we can also deduce the movement-vector, velocity and so on. The main problem with this second option is that it becomes increasingly more difficult to track a given object the faster the object moves.
6. After we have worked out the movement-vector we can use it to predict the next movement. Here there are, again, many possible options to choose from. By only regarding the last movement, we can just continue this path and check for collision on that path. We could, however, also track the whole history of detected movement vectors. Now it becomes possible to use more complex programs to analyse the movement, but we have not yet done so, also because we can not gauge how these affect the overall performance. But just by regarding the last few movements we could predict a given movement far better, for example by not only reiterating the last vector but also the turns between the vectors.
7. Lastly we can give our output, if there is a collision detected or not.


# Sources, links and literature:
  # Repo
    https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver?utm_source=chatgpt.com (Universal_Robots_ROS2_Driver)
    https://github.com/isl-org/Open3D (Open 3D)
    https://github.com/PointCloudLibrary/pcl (Point Cloud Libary)
    https://github.com/yanx27/Pointnet_Pointnet2_pytorch (PointNet ++, PointNeXT wurde rausgezogen)
    https://github.com/facebookresearch/votenet (Deep Hough Voting for 3D Object Detection in Point Clouds)

  # Data Sets
    https://github.com/ScanNet/ScanNet (darauf wurde VoteNet trainiert)

  # Papers
    https://arxiv.org/pdf/2208.07678 (FEC: Fast Euclidean Clustering for Point Cloud Segmentation)
    https://arxiv.org/pdf/2412.04649 (Generating Whole-Body Avoidance Motion through Localized Proximity Sensing)
    https://www.vision.rwth-aachen.de/media/papers/know-what-your-neighbors-do-3d-semantic-segmentation-of-point-clouds/W63P26.pdf (Know What Your Neighbors Do: 3D Semantic     Segmentation of Point Clouds)
    https://arxiv.org/pdf/2405.11903 (A comprehensive overview of deep learning techniques for 3D point cloud classification and semantic segmentation)
    https://arxiv.org/pdf/1812.05784 (PointPillars: Fast Encoders for Object Detection from Point Clouds)
    https://openlib.tugraz.at/download.php?id=5f6b335524db8&location=browse&utm_source=chatgpt.com (360° Monitoring for Robots Using Time-of-Flight Sensors)
    https://biorobotics.ri.cmu.edu/papers/paperUploads/Abah_Multi-modal_2019.pdf (A Multi-modal Sensor Array for Safe Human-Robot Interaction and Mapping)
    https://arxiv.org/pdf/1904.09664 (Deep Hough Voting for 3D Object Detection in Point Clouds)
    https://arxiv.org/pdf/2308.11166 (Hierarchical Point-based Active Learning for Semi-supervised Point Cloud Semantic Segmentation)

  # Other Formats
    https://medium.com/@BasicAI-Inc/3d-point-cloud-segmentation-guide-a073b4a6b5f3 (Introduction to 3D Point Cloud Segmentation)
    https://forum.universal-robots.com/t/eye-in-hand-camera-calibration/35922?utm_source=chatgpt.com (Eye-in-hand camera calibration)
