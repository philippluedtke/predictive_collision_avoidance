# About the Project

Objective of the project is the development of a reactive collision avoidance system for a colloborative robot.

# Introduction

# Relevance in Industry and Academia

Industry 5.0 signifies a pivotal realignment of industrial priorities, emphasising human-centric collaboration, sustainability, and resilience rather than the pursuit of automation for its own value [1] Building on the digital and cyber-physical foundations of previous industrial advancements, Industry 5.0 employs technologies such as the industrial Internet of Things (IoT), artificial intelligence (AI), digital twins, industrial robots and additive manufacturing to facilitate smart factories, while redefining technological progress as a means to empower human workers and reduce environmental impact [2]. The focus of Industry 4.0 was predominantly on efficiency and connectivity, which gave rise to concerns regarding job security, rising unemployment due to automation, and environmental issues such as excessive energy consumption and electronic waste. Industry 5.0 aims to address these social and ecological gaps by augmenting human capabilities through collaborative machine systems, as opposed to replacing them. Collaborative robots (CoBot) are designed to undertake repetitive or hazardous tasks, thereby allowing human operators to focus on more complex activities such as oversight, problem solving and value-adding operations [1]. This shift presents a range of practical opportunities, including mass customisation, greater production flexibility, optimised resource use, and the inclusion of disabled people in the workforce. These opportunities can help to meet growing consumer demand for personalised products while lowering material and energy footprints [2]. Simultaneously, the increased proximity of humans and machines gives rise to new and significant safety challenges. These challenges require robust technical solutions and regulatory guidance to ensure worker protection under all conditions [4]. In practice, common safety systems such as torque sensors provide reliable collision detection and force control. However, they only signal after an impact has occurred. This means that complementary approaches, such as proximity sensing, offer promising opportunities for achieving anticipatory protection and facilitating smoother and safer human-robot interaction [4]. From an occupational health and safety perspective, human-centred collaboration has been shown to reduce musculoskeletal risk factors, decrease physical effort, improve coordination and efficiency, and lower exposure to hazards. Some experimental studies also report fewer errors per unit of time and maintained trust in cobotic systems, even when total error counts remain similar across conditions [4]. The societal relevance of human-centred manufacturing is reinforced by legal frameworks such as the § 154 of the German Social Code Book IX that promote inclusion and reasonable accommodations for severely disabled employees. This ensures that industrial practice aligns with broader goals of social participation and equity [5]. ...


[1] From automation to collaboration: exploring the impact of industry 5.0 on sustainable manufacturing | Discover Sustainability

[2] Industry 5.0 - Publications Office of the EU and Industry 5.0 - Research and innovation - European Commission

[3] (PDF) Adaptive Obstacle Avoidance for a Class of Collaborative Robots

[4] Human-Cobot collaboration's impact on success, time completion, errors, workload, gestures and acceptability during an assembly task

[5] § 154 SGB IX - Einzelnorm
_________________________________________________________________________________________

# The Code

"30_7_pico_serial_connection.ino" can be used for the pico. All programs with the "json"-token however use a different method to the used with the sensor ring.

"voxel_decay_time" is the voxel-based program.

# The Hardware 
- Microcontroller: Raspberry Pi Pico (RP2040)
- Sensor: VL53L7CX ToF (8x8 grid) (capabale of IR)
- Mounting: One ring made out of six sensors (with dead zones)
- Robotic arm: UR10
- https://ams-osram.com/products/sensor-solutions/direct-time-of-flight-sensors-dtof/ams-tmf8829-48x32-multi-zone-time-of-flight-sensor (sadly not yet open for orders)


# How the Program is structured

The program can be structured by these steps:
1. Collection of the Sensor-data (which is not a part of this project).
2. Storage of the collected data. There are exact (point-cloud) and discrete (voxels) options. The exact options have less built-in error, but the discrete options offer far better performance which is critical when dealing with object or motion detection. Therefore, voxels are the best option. As we want to not only detect objects but also the movement of these objects we will have to track these objects over time. There are, again, two main options: We could store each timeframe as a different grid or we could store all points in the same grid but give the points themselves the needed information. The first option gives us a history of all received data, but the management can become difficult and runs the risk of Memory Errors. The second option restricts the size of the stored data significantly, again trading performance for exactness.
3. First analysis of the stored data to find objects. This can be done by cluster-analysis. The DBSCAN-algorithm is a good compromise between performance and accuracy, as it uses the simplest algorithm which can detect overlapping objects (for example a ring with a different object inside) with reasonable accuracy.
4. The detected objects or groups of points must now be stored to use them for motion-detection later. Here, we have two options: Storing all points associated with the object, or simplify the object to one point, for example the center. Obviously, the latter would be far simpler, but the first one has critical advantages: Firstly, the Object can be tracked more easily as more information about the object is memorized. Secondly, the dimensions of the object are preserved. This is particularly important, as some objects could otherwise become impossible to distinguish (the ring for example would have the same center as the object inside). We must therefore store all associated points.
5. Tracking of the detected objects. This is arguably the most difficult part, as we now have to use the data from different timeframes. Our Program tracks different objects using two different methods. First, it checks for overlap between the objects last positions and the positions of the new, not yet associated clusters detected by DBSCAN. This method works reasonably well and has a very low error-rate, but has some difficulties and problems discussed later on. Notably, if no object can be associated by overlap the reason can be that the object moved away too fast. We then associate the object with the nearest yet unassociated object in a given radius. Although this is more prone to false associations it is a necessary enhancement of our basic method. Obtaining the motion vector for objects which have been tracked over time is very simple.
6. After we have worked out the movement vectors we use them to predict the next movement. For this we only use a very basic algorithm. By only regarding the last movement, we can continue this path and check for collisions with our known robot geometry on that path. We could also track the whole history of detected movement vectors, making it possible to use more complex programs to analyse the movement, but this would exceed the scope of this project.
7. Lastly we give our output, if there is a collision detected or not. The determined movement-vector can also be shown, including a simple recommended path to avoid a detected collision.

<img width="320" height="451" alt="image" src="https://github.com/user-attachments/assets/22564d95-09b0-4a0e-aebf-7146b180f9cc" />

Diagram showing the parts of the program. The Orange boxes represent the different storages.

# About the final Program:

Our main program (see: "voxel_boolean_json" and the other voxel-based-programs) fulfills all of these requirements.

The initial problem of latency was largely solved. The Raspberry Pi Pico we use to read the data outputs at 8.33 Hz. The main Program need approximately 60 to 70 ms to complete one cycle (which includes data-gathering and all following steps) which was also tested with larger sample sizes from up to 7 sensors.

A different problem is the fact that there is a highest velocity at which a given object is correctly identified. The problem originates from the way the main program tries to trail the objects. To discretize the problem we simply group the data recieved by the sensors together by time. It seems best to use the time needed for one program cycle for this "clock frequency", that means 0.15 seconds or 6.67 Hz in our program. Fundamentally, there are now two differing possible methods:
By checking for overlap between each time-frame we can track each object over time. This is very reliable for low velocities and efficient, because it can be expressed as simple repeated matrix-multiplications. As such it is our main tracking method. As long as the object velocity is smaller than the exposed lenght (meaning the length of the object perpendicular to the detecting sensor) of the object divided by the clock frequency the program will detect overlapping voxels, but above this velocity the object will likely have  already moved out of the way. The Program will then regard the object as two seperate objects, with a velocity of zero (see example image below). 

<img width="795" height="203" alt="image" src="https://github.com/user-attachments/assets/bf5b9d51-4a5c-4192-beeb-ac4cd531b771" />

The upper example shows a case with a velocity lower than the maximum detecable, the lower with a velocity higher. 

To complement this, one could also search in proximity of the detected object, but this is disproportionately inefficient as now the program would have to actually compute the distances. One way we could solve these performance disadvantages is by using a method called morphological dilation. Instead of comparing the distance between points, one objects volume is virtually expanded in size by the radius we would otherwise check the distance for. In the next step, we simply have to once again search for an overlap. This way is not only more efficient, but also far more consistent for different object sizes. This way however does not enable to detect velocities larger than the searched radius divided by cycle time, but cooperates nicely with our previously described main tracking method. A different approach could be to limit the searched volume, for example by using previously determined movement-vectors to restrict the direction. The problem arising from this is the fact that one would need a first movement-direction to begin with, meaning it would have to have already been observed and then accelerated, or have just entered the workspace with an unkown velocity, but likely with a direction into the workspace. This last case (fast object entering the workspace) is arguabely the most dangorous, but also the most difficult to detect and track.

Another problem stems from the inherent uncertainty of the voxel-solution. The further away an object is from the sensor, the more voxels are availabe to represent each data-input. This creates two critical regions: At a certain distance  gaps begin to form between the voxels, creating unclear geometries. This can be fixed by increasing the amount of activated voxels with increasing distance to the sensor. The other critical region is the region directly in front of the sensor. Objects in close proximity can never be fully represented in the same voxel size and will activate most voxels directly in front of the sensor which results in erratic movement detection. Two possible solutions for this problem are to either increase the voxel-density in the proximity around the sensors (for example by using Adaptive Mesh Refinement) or by using data from a different sensor whose view of the critial region is unobstructed, both of which exceed the scope of this project.

Datasheet - VL53L7CX - Time-of-Flight 8x8 multizone ranging sensor with 90° FoV: https://www.st.com/resource/en/datasheet/vl53l7cx.pdf

# Results from testing with the real sensor-ring:

Real-life testing revealed many coding and design-flaws of our approach. Although the program handled the increased data given by 6 additional sensor quite good with cycle-times of 85 to 65ms, the main concern were significant problems with the detection and especially the distinction of the different objects. We tried to address these problems by adjusting the parameters, particularly the epsilon of the DBSCAN (meaning the distance at which two voxels are merged). A major cause for errors were the static surroundings, which often grouped together with nearby moving objects or merged and then unmerged with other static objects, both resulting in motion errors. We could only partially adress this by adjusting the DBSCAN parameters, so we decided to introduce a significant adjustment by scanning for static objects before starting the main program as part of a preparation function. We then assume that these static voxels are not relevant for the object-detection. This way, we were able to reduce the merging errors considerably, but the process we use to identify the static objects is still not certain or fully developed. 

Another problem was the cycle-clock-speed. We tried classic boolean-based voxels as well as the time-based approach. The main advantages of the boolean method are a theoretically better performance, although we could not demonstrate this, and the possibility of better movement reconstruction, meaning the ability to trace back the movement further back in time as it would be possible using the time based method as each timeframe is separately recorded. The advantage of the time based method are a more consistent tracking, more adjustability in form of the TIME_TOLERANCE variable and the enhanced expandability for possible other sensor rings as there are no time-frames but a more fluid memorization of the received sensor data.

After adressing these problems we were able to gather results for motion vectors which were relatively close to the real movement, although there was still significant noise sometimes resulting in erratic, but fundamentally true findings (real direction, but wrong velocity). We found another problem when trying to detect two movements, as the errors we previously had with the static surroundings now also applied to the two objects when they came closer together. Nonetheless, we were able to get quite satisfactory results.

https://github.com/user-attachments/assets/aab121cd-14c5-4a12-bac2-3a51d68af8f2

This video shows our test with two moving objects, birds eye view. The beginning (0-2 seconds) shows one person approaching the sensors relatively fast, resulting in erratic behavior. Until 5 seconds the movement can then be identified quite well. At 6 seconds, the second person apporaches the sensors. While both persons are then near the sensors, the detected objects merge and result in no (or very small) detected motion. Around 11 seconds, the first person exits the sensor-range and both objects seperate again. The second persons movement can then be observed well again unit he exits the sensor range as well.

The parameters used in the program were adjusted empirically. For the DBSCAN-parameters (eps and min_samples) we used the established "k-distance-plot" and "grid-search with slihouette score" algorithms to find the optimal values for our given problem.

<img width="820" height="422" alt="image" src="https://github.com/user-attachments/assets/e311e7a7-2d18-4d4c-a943-2c5a3fc78bee" />

Here, the results of the value-finding algorithm can be seen.

# Current limitations of the Program:
Fundamentally there are two forms of limitatiosn for our final program: Theoretical limitations which stem from the way the program works and real life limitations which mainly come from uncertanties of the sensors.

The most critical limitation is the size of objects which can be detected, which is variable and dependent on distance from the sensors, movement speed and material (reflective or translucent meterials cannot be detected) of the object. Testing revealed that rods with a diameter of around 2cm can only be detected directly in front of the sensor ring while objects with a diameter larger than 5cm (a human arm for example) can be deteced at a distance greater than 100cm. Objects which are moving can generally be detected better, as they can often be seen by more sensors, although it's difficult to get precise thresholds for this phenomenon, as it again also depends on the distance to the sensors.

The theoretical limitations were already addressed previously, but the main problems are the difficulties regarding DBSCAN, the detection of movement speeds with variable maximal and minimal speeds, the necessary detection of stationary objects and the sometimes erratic detected motion vectors.

# Current Problem wit DBSCAN in ToF pointcloud setting 
DBSCAN causes spatially distant points to be grouped together when used in environments with a limited number of voxels. This effect stems from the algorithm relying on local neighborhood density and k nearest neighbors. Figure 1 clearly shows that the scanner initially detected a pool noodle as a separate object. However, when it came close to the wall, the scanner merged both into a single cluster. Merging reduces our ability to separate objects by distance and to detect novel items reliably and semantically. Additionally, initial tests revealed that our sensor ring produces false points, which introduce noise in a already sparse voxel representations. It is vital to handle static objects that interfere with the clustering. The solution is straightforward: save them beforehand so that those detected voxels are not taken into consideration for the DBSCAN. This also allows for more empirical parameter optimisation of the DBSCAN parameter.

<img width="411" height="296" alt="image" src="https://github.com/user-attachments/assets/b45a1278-5f81-46cc-9527-4ffe1cd3aa28" />
<img width="411" height="296" alt="image" src="https://github.com/user-attachments/assets/03be48fd-0709-4abd-b1e1-a4671e010b15" />

# Limitations
- Objectsize depedent on sensor hardware (density of voxels)
- paramater optimization empirically done
- max and min velocities -> from object detection and
- two groups limitation:
- sensor fluctuation -> problems

# Solve DBSCAN Algorithms by tuning its hyper parameters
<img width="820" height="422" alt="image" src="https://github.com/user-attachments/assets/e311e7a7-2d18-4d4c-a943-2c5a3fc78bee" />
<img width="727" height="388" alt="image" src="https://github.com/user-attachments/assets/33adcf63-9ce8-4e99-9190-8fb2fd0dd61e" />

The K distance plot is a widely used diagnostic tool for selecting the epsilon parameter for density-based clustering algorithms such as DBSCAN. For voxel-based point clouds the plot conveys not only a scale for neighborhood density but also signatures of the underlying grid structure.
The elbow point is the location of maximal curvature in the K distance curve. In practice it is the point where the plotted distances change from a relatively flat trend to a markedly increasing slope. Geometrically this point separates points that reside in dense local neighborhoods from points that are isolated or belong to sparse clusters. The vertical coordinate at the elbow is commonly chosen as the epsilon value for DBSCAN. Intuitively, points that appear before the elbow have small distance to their k nearest neighbor and therefore belong to dense cluster interiors. Points that appear after the elbow have substantially larger k distance and are likely to be noise or members of very sparse clusters. Setting epsilon to the elbow value implements the decision rule: every point whose k distance is smaller than epsilon is considered part of a cluster while points with larger k distance are treated as noise. Points inside clusters tend to have many nearby neighbors so their k distance stays small and the curve is flat. When the curve reaches the elbow the population of points transitions from cluster interior to boundary or to background. This transition produces the characteristic knee shape that guides epsilon selection.
Voxelized point clouds are defined on a discrete integer grid. Distances between voxel centers are computed with the Euclidean norm d=√(Δx^2+Δy^2+Δz^2) where each delta is an integer difference between voxel coordinates. Because the coordinate differences are integers the set of possible distance values is discrete and limited. When the k distance values for all points are sorted, many points frequently share identical or nearly identical distance values. This results in extended horizontal segments in the sorted curve. Each flat segment corresponds to a bin of identical distance values produced by the grid geometry. When the next larger discrete distance appears the curve jumps to the next step. The more regular the grid and the coarser the voxel resolution the stronger the staircase effect. The staircase appearance is not a flaw but an artifact of discretization. It implies that small changes to epsilon within a flat segment will not change cluster assignments. Conversely, choosing epsilon values at jump points will change the number of neighbors for many points at once and may cause abrupt changes in the clustering outcome. In practice it is therefore advisable to choose epsilon near the top of a stable flat segment immediately before a jump, or to use algorithms that estimate the elbow by curvature rather than by manual inspection.




# Sources, links and literature:
  # Repo
    https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver?utm_source=chatgpt.com (Universal_Robots_ROS2_Driver)
    https://github.com/isl-org/Open3D (Open 3D)
    https://github.com/PointCloudLibrary/pcl (Point Cloud Libary)
    https://github.com/yanx27/Pointnet_Pointnet2_pytorch (PointNet ++, PointNeXT was extreacted)
    https://github.com/facebookresearch/votenet (Deep Hough Voting for 3D Object Detection in Point Clouds)
    https://github.com/seung-lab/connected-components-3d (connected-components-3d)

  # Data Sets
    https://github.com/ScanNet/ScanNet (VoteNet was trained on this data)

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
