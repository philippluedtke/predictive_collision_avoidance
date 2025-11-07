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
1. Collection of the Sensor-data. (This is not really part of this project)
2. Storage of the collected data. There are exact (point-cloud) and discrete (voxels) options. Basically, the exact options have less built-in error, but the discrete options offer far better performance which is critical when dealing with object or motion detection. Therefore, voxels seem like the best option. As we want to not only detect objects but also the movement of these objects we will have to track these objects over time. There are, again, two main options: We could store each timeframe is a different grid, for example by giving our points an extra time-coordinate or creating a new grid for every new timeframe, or we could store all points in the same grid but give the points themselves the needed information. The first option basically gives us a history of all recieved data, but the management can become difficult. The second option restricts the size of the stored data significantly, again trading performance for excactness.
3. First analysis of the stored data to find objects. This can be done by cluster-analysis. The DBSCAN-algorythm is good compromise between performance and accuracy, as its bascially the simplest algorythm which can detect overlapping objects (for example a ring with a different object inside).
4. The detected objects or groups of points must now be stored to use them for motion-detection later. Here, we seem to have two options: We can store all points assosiated with the object, or simplify the object to one point, for example the center. Obviously, the latter would be far simpler, but the first one has critical advantages: Firstly, the Object can be tracked more easily. Secondly, the dimensions of the object are preserved. This is particulary important, as some objects could otherwise become impossible to distinguish (the ring for example would have the same center as the object inside). We must therefore store all associated points.
5. Tracking of the detected objects. This is arguabely the most difficult part, as we now have to use the data from different timeframes. There are many different options, but so far we`ve explored two: By checking the overlap of the different detected objects, we can track the object through the different time frames. This works reasonably well, but is quite resource-intensive and does not work well with highter velocities. The other option takes advantage of the way information can the stored when using voxels. Usually, when using voxels, a (volumetric-)pixel is associated with a boolean true/false state. By instead storing, for example, the time at which the point was last detected, the pixel automatically becomes active troughout time. A moving object creates a tail-like structure behind itsself by which we can also deduce the movement-vector, velocity and so on. The main problem with this second option is that it becomes increasingly more difficult to track a given object the faster the object moves.
6. After we have worked out the movement-vector we can use it to predict the next movement. Here there are, again, many possible options to choose from. By only regarding the last movement, we can just continue this path and check for collision on that path. We could, however, also track the whole history of detected movement vectors. Now it becomes possible to use more complex programs to analyse the movement, but we have not yet done so, also because we can not gauge how these affect the overall performance. But just by regarding the last few movements we could predict a given movement far better, for example by not only reiterating the last vector but also the turns between the vectors.
7. Lastly we can give our output, if there is a collision detected or not. The determined movement-vector can also be shown.

<img width="320" height="451" alt="image" src="https://github.com/user-attachments/assets/22564d95-09b0-4a0e-aebf-7146b180f9cc" />

Diagram showing the parts of the program. The Orange boxes represent the different storages.

# About the final Program:

Our main program (see: "voxel_boolean_json" and the other voxel-based-programs) fulfills all of these requirements, but of course there are some limitations, which I now want to elaborate.

The main problem is latency. The datasheet of the VL53L7CX-sensor gives a maximum frequency of 1000 Hz, which should theoretically enable smooth, real-time data to be gathered. However, the Raspberry Pi Pico we use to read the data only outputs at 8.33 Hz. The main Program need approximately 0.03 seconds to complete one cycle (which includes data-gathering and all following steps), although this needs to be more thoroughly tested with larger sample sizes. This gets the total time for one cycle to about 0.15 seconds, or a frequency of 6.67 Hz. One way to reduce latency would be to use parallelization to split the gathering- and refinementprocess and the analysation process.

A different problem is the fact that there is a highest velocity at which a given object is correctly identified. The problem originates from the way the main program tries to trail the objects. To discretize the problem we simply group the data recieved by the sensors together by time. It seems best to use the time needed for one program cycle for this "clock frequency", that means 0.15 seconds or 6.67 Hz in our program. Fundamentally, there are now two differing possible methods:
By checking for overlap between each time-frame we can track each object over time. This is very reliable for low velocities and efficient, because it can be expressed as simple repeated matrix-multiplications. As such it is our main tracking method. As long as the object velocity is smaller than the exposed lenght (meaning the length of the object perpendicular to the detecting sensor) of the object divided by the clock frequency the program will detect overlapping voxels, but above this velocity the object will likely have  already moved out of the way. The Program will then regard the object as two seperate objects, with a velocity of zero (see example image below). 

<img width="795" height="203" alt="image" src="https://github.com/user-attachments/assets/bf5b9d51-4a5c-4192-beeb-ac4cd531b771" />

The upper example shows a case with a velocity lower than the maximum detecable, the lower with a velocity higher. 

To complement this, one could also search in proximity of the detected object, but this is disproportionately inefficient as now the program would have to actually compute the distances. The way we solve these performance issues is by using a method called morphological dilation. Instead of comparing the distance between points, one objects volume is virtually expanded in size by the radius we would otherwise check the distance for. In the next step, we simply have to once again search for an overlap. This way is not only more efficient, but also far more consistent for different object sizes. This way however does not enable to detect velocities larger than the searched radius divided by cycle time, but cooperates nicely with our previously described main tracking method. A different approach could be to limit the searched volume, for example by using previously determined movement-vectors to restrict the direction. The problem arising from this is the fact that one would need a first movement-direction to begin with, meaning it would have to have already been observed and then accelerated, or have just entered the workspace with an unkown velocity, but likely with a direction into the workspace. This last case (fast object entering the workspace) is arguabely the most dangorous, but also the most difficult to detect and track.

Another problem stems from the inherent uncertainty of the voxel-solution. The further away an object is from the sensor, the more voxels are availabe to represent each data-input. This creates two critical regions: At a certain distance  gaps begin to form between the voxels, creating unclear geometries. This can be fixed by increasing the amount of activated voxels with increasing distance to the sensor. The other critical region is the region directly in front of the sensor. Objects in close proximity can never be fully represented in the same voxel size and will activate most voxels directly in front of the sensor which results in erratic movement detection. Two possible solutions for this problem are to either increase the voxel-density in the proximity around the sensors (for example by using Adaptive Mesh Refinement) or by using data from a different sensor whose view of the critial region is unobstructed, both of which exceed the scope of this project.

Datasheet - VL53L7CX - Time-of-Flight 8x8 multizone ranging sensor with 90° FoV: https://www.st.com/resource/en/datasheet/vl53l7cx.pdf

# Results from testing with the real sensor-ring:

In late october/early november 2025 we tested our code.

# Current Problem wit DBScan in ToF pointcloud setting 
DBSCAN causes spatially distant points to be grouped together when used in environments with a limited number of voxels. This effect stems from the algorithm relying on local neighborhood density and k nearest neighbors. Figure 1 clearly shows that the scanner initially detected a pool noodle as a separate object. However, when it came close to the wall, the scanner merged both into a single cluster. Merging reduces our ability to separate objects by distance and to detect novel items reliably and semantically. Additionally, initial tests revealed that our sensor ring produces false points, which introduce noise in a already sparse voxel representations. It is vital to handle static objects that interfere with the clustering. The solution is straightforward: save them beforehand so that those detected voxels are not taken into consideration for the DBScan. This also allows for more empirical parameter optimisation of the DBScan parameter.
<img width="617" height="445" alt="image" src="https://github.com/user-attachments/assets/b45a1278-5f81-46cc-9527-4ffe1cd3aa28" />
<img width="617" height="445" alt="image" src="https://github.com/user-attachments/assets/03be48fd-0709-4abd-b1e1-a4671e010b15" />



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
