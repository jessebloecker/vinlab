# VINLAB
## Generate and visualize 3D motion, simulate sensor data, and support the development visual-inertial navigation systems.

Specify a visual-inertial scene containing a trajectory, sensor platform, and features.


<details>
<summary><b>example_config.yaml</b>: bspline trajectory, 2 cameras, 1 imu, point features</summary>

```yaml
trajectory_group:                   #group of related trajectories - i.e. ground_truth, estimate1, estimate2, etc.
    reference: ground_truth         #which trajectory to use as the reference for the other trajectories
    trajectories:                   #description of each trajectory including translation and rotation components, or read from file
      - id: ground_truth            #id of the trajectory
        translation:                #describe translation component of the trajectory
          type: bspline             #type of trajectory (bspline, circle, line, random, or from_file)
          bspline:                  #specify bspline
            res: 50                 #number of points per span
            order: 5                #order of the bspline
            span_time: 1            #time to traverse each span
            geometric_only: false   #(not implemented) if true bspline will define the position only, velocity magnitude is independent
            control_pts:            #ordered list of control points that define the bspline, repeated points reduce the velocity around that point
              - [ 10.   , 0.   , 0.]
              - [ 10.   , 0.   , 0.]
              - [ 10.   , 0.   , 0.]
              - [ 10.   , 0.   , 0.]
              - [  9.01 , 2.339, 0.]
              - [  6.235, 7.818, 2.]
              - [  2.225, 2.749, 0.]
              - [ 2.225, 9.749, 0.]
              - [ -6.235, 7.818, 0.]
              - [ -9.01 , 4.339, 4.]
              - [-10.   , 0.   , 3.]
              - [ -10.01 ,0.339, -2.]
              - [ -8.235,-7.818, 0.]
              - [ -2.225,-9.749, 6.]
              - [  2.225,-9.749, 3.]
              - [  6.235, 1.818, 0.]
              - [  9.01 ,-4.339, 0.]
              - [ 10.   ,-0.   , 0.]
              - [ 10.   ,-0.   , 0.]
              - [ 10.   ,-0.   , 0.]
              - [ 10.   ,-0.   , 0.]
              - [ 10.   ,-0.   , 0.]
        rotation:                     #rotation component of the trajectory
          type: align_axis            #type of rotation trajectory (align_axis, constant, from_file, random)
          align_axis:                 #align one axis of the base frame with a vector at each time step (base frame can be any frame on the platform)
            axis: x                   #axis of the base frame to align
            with: vel                 #vector to align axis with (vel, acc, centroid, or a specified list of vectors)
            grounded_axis: z          #axis of the base frame that will remain parallel to the ground (global xy-plane) at all times
            flip: true                #two solutions exist, select between them with this boolean
      - id: estimate
        file: '/path/to/data/trajectory.txt' #load trajectory from file
        jpl: true                       #if true, quaternions in the file are assumed JPL convention, else hamilton convention
        # align: first                     
        #   type: first                 #first, best_fit,
        #   rescale: false              #if true, rescaling is allowed in computing best fit alignment, otherwise only translation and rotation are allowed


platform:                               #specify the sensor platform
  id: 'stereo'                          #identifier for the whole sensor platform
  base_frame: 'cam0'                    #the frame that will follow the reference trajectory. There must be a sensor or body frame with this id
  sensors:                              #specify all camera and imu sensors
    - id: 'imu0'                        #id of sensor
      type: 'imu'                       #type of sensor (imu, camera)
      enable_measurements: true         #if true, simulated imu measurements will be generated based on the trajectory and noise parameters
      rate: 100                         #sensor rate in Hz (cannot be higher than the trajectory rate)
      transform:                        #transform will be ignored and set to unity for which ever frame is specified as the base frame
        rotation: [0,0,0]               #x-y-z fixed-axis euler angles in degrees from base frame to this sensor frame
        translation: [0,0,0]            #translation from base frame to this sensor frame
      imu:                              #imu noise parameters
        gyro_noise: 0.001               #noise density (sigma/sqrt(Hz))
        gyro_bias: 0.0001               #bias instability (sigma/sqrt(Hz))
        accel_noise: 0.01               #noise density (sigma/sqrt(Hz))
        accel_bias: 0.001               #bias instability (sigma/sqrt(Hz))
      time_offset: 0.0                  #add this value to all timestamps generated by this sensor

    - id: 'cam0'                        
      type: 'camera'                    #type of sensor (imu, camera)
      enable_measurements: true         #if true, simulated camera measurements of the features will be generated based on the trajectory and noise parameters
      rate: 25                          #sensor rate in Hz (cannot be higher than the trajectory rate)
      transform:                      
        translation: [-3.6,-10.,0.]     #translation from base frame to sensor frame
        rotation: [180,0,0]             #x-y-z fixed-axis (z-y-x moving-axis) euler angles in degrees from base to this sensor frame
      camera:                           #camera intrinsics and resolution
        height: 480                     #height of the image in pixels
        width: 640                      #width of the image in pixels
        intrinsics: [300,300,325,242]   #fx,fy,cx,cy
        distortion: [-0.01, 0.01, 0.00019359, 1.76187114e-05] #radial-tangential k1,k2,p1,p2
      time_offset: 0.0                  #add this value to all timestamps generated by this sensor
                                        
    - id: 'cam1'                        
      type: 'camera'                    
      enable_measurements: false 
      rate: 4                          
      transform: 
        parent: 'cam0'                   #frame relative to which this transform is defined, if not provided, this platform's 'base frame' is assumed                    
        translation: [0,0.5,0]            
        rotation: [0,0,45]              
      camera:                           
        height: 480                    
        width: 640                     
        intrinsics: [500,500,320,240]   
        # distortion: 
      time_offset: -0.0012               #add this value to all timestamps generated by this sensor

  body_frames:                           #list of other frames attached to the platform that are not sensors
    - id: test_frame    
      transform:                        
        rotation: [30,15,0]              #x-y-z fixed-axis (z-y-x moving-axis) euler angles in degrees from base to this frame
        translation: [1,2,0]             #translation from base frame to this frame               
                                        
features:                               #list of features to be added to the scene 
  - id: feats0                          #id of the feature (or set of features)
    type: point_set                     #type of feature (point_set, random_point_set, planar_point_set)
    color: red                          #color of the feature
    points:                             #list of points that define the feature
      - [6,0,0.5]                         
      - [-7,-2,2]                        
      - [-4,1,0] 
    center: [1,-1,1]                    #optional value to be added to all points in the list                        
                                        
  - id: feats1                          
    type: random_point_set              #type of feature (point_set, random_point_set, planar_point_set)
    num: 100                            #number of random points to generate
    center: [-2,0,2]                    #center of the point set
    radius: 1                           #max distance from the center
                                        
  - id: feats2                          
    type: planar_point_set              
    num: 50                             #number of random points to generate (ignored if grid_spacing is specified)
    color: yellow                        
    center: [2,2,0]                     #center point of plane
    normal: [0, 0, -1]                  #normal vector of the plane
    radius: 3                           #max distance from the center
    grid_spacing: 0.25                  #if specified, points will be generated in a grid with this spacing, else they will be randomly spaced on the plane
 
  - id: feats3                          
    type: planar_point_set              
    num: 50                             #number of random points to generate (ignored if grid_spacing is specified)
    color: cyan                        
    center: [0,8,2]                     #center point of plane
    normal: [0, -1, -1]                 #normal vector of the plane
    radius: 4                           #max distance from the center
    grid_spacing: 0.5                   #if specified, points will be generated in a grid with this spacing, else they will be randomly spaced on the plane
  
  - id: feats4                          
    type: planar_point_set              
    num: 50                             #number of random points to generate (ignored if grid_spacing is specified)
    color: pink                       
    center: [0,-5,2]                    #center point of plane
    normal: [3, -1, 1]                  #normal vector of the plane
    radius: 2                           #max distance from the center                

```
</details>

https://github.com/jessebloecker/vinlab/assets/29802265/815486bf-6d41-4584-a26d-bc99d16f060b


Vinlab contains Python and C++ and uses [Scipy](https://scipy.org/), [OpenCV](https://opencv.org/), [ROS2](https://docs.ros.org/en/humble/index.html), and [Eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html)


## Install
todo: put instructions here

## Run Demo
todo: put instructions here

run the scene viewer
~~~
ros2 run vinlab scene_viewer.py --ros-args --params-file /path/to/ros2_ws/src/vinlab/config/scene_viewer.yaml
~~~

run the camera simulator
~~~
ros2 run vinlab cam_simulator_node
~~~

run rviz:
~~~
rviz2 -d ~/path/to/config.rviz'
~~~




