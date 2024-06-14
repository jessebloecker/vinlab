### **accel_noise_density** ([imu](#imu-sensor))
required: no  

 **description**: accelerometer noise density - units: (m/s^2) * (1/sqrt(Hz)) 
<hr style="border:1px solid gray"> 

### **accel_random_walk** ([imu](#imu-sensor))
required: no  

 **description**: accelerometer rate random walk - units: (m/s^3) * (1/sqrt(Hz)) 
<hr style="border:1px solid gray"> 

### **align_axis** ([rotation_trajectory](#rotation_trajectory-trajectory))
required: no  
**co-keys**: [bspline_so3](#bspline_so3-rotation_trajectory), [file](#file-rotation_trajectory), [constant](#constant-rotation_trajectory)  
**sub-keys**: [grounded_axis](#grounded_axis-align_axis), [vec](#vec-align_axis), [flip](#flip-align_axis), [axis](#axis-align_axis) 

 **description**: arguments for the [rotation_align_axis function](/vinlab/src/geometry_utils.py) 
<hr style="border:1px solid gray"> 

### **align_axis** ([rotation](#rotation-rotate_about_point))
required: no  
**sub-keys**: [grounded_axis](#grounded_axis-align_axis), [vec](#vec-align_axis), [flip](#flip-align_axis), [axis](#axis-align_axis) 

 **description**: arguments for the [rotation_align_axis function](/vinlab/src/geometry_utils.py) 
<hr style="border:1px solid gray"> 

### **axis** ([align_axis](#align_axis-rotation))
required: no  

 **description**: string, 'x','y', or 'z' axis to align with 'vec' 
<hr style="border:1px solid gray"> 

### **base_frame** ([platform](#platform-scene))
required: no  

 **description**: name of the base frame of the sensor platform. This is the frame that will follow the described trajectory, and the frame that the sensor and body frames are defined relative to, by default 
<hr style="border:1px solid gray"> 

### **body_frame** ([body_frames](#body_frames-platform))
required: no  
**sub-keys**: [transform](#transform-body_frame), [id](#id-body_frame) 

 **description**: fixed frame attached to the sensor platform 
<hr style="border:1px solid gray"> 

### **body_frames** ([platform](#platform-scene))
required: no  

 **description**: list of body frames attached to the sensor platform 
<hr style="border:1px solid gray"> 

### **bspline** ([translation_trajectory](#translation_trajectory-transform))
required: no  
**co-keys**: [file](#file-translation_trajectory), [constant](#constant-translation_trajectory)  
**sub-keys**: [control_points](#control_points-bspline), [span_time](#span_time-bspline), [degree](#degree-bspline), [res](#res-bspline) 

 **description**: uniform bspline trajectory - generates spline of position values from set of control points 
<hr style="border:1px solid gray"> 

### **bspline_so3** ([rotation_trajectory](#rotation_trajectory-trajectory))
required: no  
**co-keys**: [file](#file-rotation_trajectory), [constant](#constant-rotation_trajectory), [align_axis](#align_axis-rotation_trajectory)  

 **description**: NOT IMPLEMENTED: bspline rotation trajectory - generate spline of rotation values from set of control rotations 
<hr style="border:1px solid gray"> 

### **camera** ([sensor](#sensor-sensors))
required: no  
**co-keys**: [imu](#imu-sensor)  
**sub-keys**: [transform](#transform-camera), [height](#height-camera), [intrinsics](#intrinsics-camera), [distortion](#distortion-camera), [width](#width-camera) 

 **description**: description of camera sensor 
<hr style="border:1px solid gray"> 

### **center** ([planar_points](#planar_points-feature))
required: no  

 **description**: position of the center of the plane 
<hr style="border:1px solid gray"> 

### **center** ([random_points](#random_points-feature))
required: no  

 **description**: position center around which to generate the random points 
<hr style="border:1px solid gray"> 

### **color** ([feature](#feature-features))
required: no  

 **description**: color of all points in the set, given as string e.g. 'red', 'blue', 'green'. If not given, a repeating sequence of colors will be assigned 
<hr style="border:1px solid gray"> 

### **constant** ([rotation_trajectory](#rotation_trajectory-trajectory))
required: no  

 **description**: NOT IMPLEMENTED: constant rotation value to apply to the trajectory 
<hr style="border:1px solid gray"> 

### **control_points** ([bspline](#bspline-translation_trajectory))
required: yes  
**sub-keys**: [file](#file-control_points), [points](#points-control_points) 

 **description**: The set of 3D position points to use as bspline control points. For degree k, there must be at least k+1 control points 
<hr style="border:1px solid gray"> 

### **control_point_rate** ([subsample_interpolate](#subsample_interpolate-trajectory))
required: no  

 **description**: rate at which to sample control points from the original trajectory 
<hr style="border:1px solid gray"> 

### **copy** ([trajectory](#trajectory-trajectory_group))
required: no  

 **description**: NOT IMPLEMENTED: copy an existing trajectory 
<hr style="border:1px solid gray"> 

### **current_trajectory** ([vec](#vec-align_axis))
required: no  

 **description**: Use the current trajectory's position, velocity, acceleration, or the bearing vectors from each position to its centroid as the source of vectors to align with 'axis'. Allowed values: 'pos', 'vel', 'acc', 'centroid' 
<hr style="border:1px solid gray"> 

### **degree** ([subsample_interpolate](#subsample_interpolate-trajectory))
required: no  

 **description**: degree of the bspline polynomial - equal to the bspline order minus 1 
<hr style="border:1px solid gray"> 

### **distortion** ([camera](#camera-sensor))
required: no  

 **description**: radtan lens distortion parameters to generate images format: [k1, k2, p1, p2] 
<hr style="border:1px solid gray"> 

### **enable_measurements** ([sensor](#sensor-sensors))
required: no  

 **description**: boolean: if true, compute simulated measurements from this sensor 
<hr style="border:1px solid gray"> 

### **feature** ([features](#features-scene))
required: no  
**sub-keys**: [id](#id-feature), [color](#color-feature), [planar_points](#planar_points-feature), [points](#points-feature), [random_points](#random_points-feature) 

 **description**: set of points and corresponding colors 
<hr style="border:1px solid gray"> 

### **features** ([scene](#scene))
required: no  

 **description**: list of features 
<hr style="border:1px solid gray"> 

### **file** ([trajectory](#trajectory-trajectory_group))
required: no  
**co-keys**: [translation_trajectory](#translation_trajectory-trajectory), [rotation_trajectory](#rotation_trajectory-trajectory), [subsample_interpolate](#subsample_interpolate-trajectory), [copy](#copy-trajectory)  
**sub-keys**: [format](#format-file), [path](#path-file), [jpl](#jpl-file), [time_unit](#time_unit-file) 

 **description**: file information for the trajectory data 
<hr style="border:1px solid gray"> 

### **file** ([control_points](#control_points-bspline))
required: no  
**co-keys**: [points](#points-control_points)  
**sub-keys**: [format](#format-file), [path](#path-file) 

 **description**: NOT IMPLEMENTED: file containing list of 3D position points to use as bspline control points 
<hr style="border:1px solid gray"> 

### **flip** ([align_axis](#align_axis-rotation))
required: no  

 **description**: boolean: if true, the 'grounded' axis will be negated, giving the other available solution 
<hr style="border:1px solid gray"> 

### **format** ([file](#file-control_points))
required: no  

 **description**: format of the file containing position and/or rotation data example: 't x y z qx qy qz qw' separated by spaces or comma according to file, use dashes to indicate columns to ignore, e.g. 't,x,y,z,-,-,-,qx,qy,qz,qw'  
<hr style="border:1px solid gray"> 

### **from** ([transform](#transform-body_frames))
required: no  

 **description**: name of the parent frame, relative to which to define the transform 
<hr style="border:1px solid gray"> 

### **grounded_axis** ([align_axis](#align_axis-rotation))
required: no  

 **description**: string, 'x','y', or 'z' axis to remain parallel to the original xy-plane. cannot be the same as 'axis' param. if not provided, it will be the next sequential axis after 'axis' (x->y, y->z, z->x). 
<hr style="border:1px solid gray"> 

### **gyro_noise_density** ([imu](#imu-sensor))
required: no  

 **description**: gyro noise density - units: (rad/s^2) * (1/sqrt(Hz)) 
<hr style="border:1px solid gray"> 

### **gyro_random_walk** ([imu](#imu-sensor))
required: no  

 **description**: gyro rate random walk - units: (rad/s^2) * (1/sqrt(Hz)) 
<hr style="border:1px solid gray"> 

### **height** ([camera](#camera-sensor))
required: no  

 **description**: height in pixels of the camera images to generate 
<hr style="border:1px solid gray"> 

### **id** ([trajectory](#trajectory-trajectory_group))
required: no  

 **description**: name of the trajectory 
<hr style="border:1px solid gray"> 

### **id** ([platform](#platform-scene))
required: no  

 **description**: name of the platform 
<hr style="border:1px solid gray"> 

### **id** ([sensor](#sensor-sensors))
required: no  

 **description**: name of the sensor 
<hr style="border:1px solid gray"> 

### **id** ([feature](#feature-features))
required: no  

 **description**: name of the feature 
<hr style="border:1px solid gray"> 

### **id** ([body_frame](#body_frame-body_frames))
required: no  

 **description**: name of the body frame 
<hr style="border:1px solid gray"> 

### **imu** ([sensor](#sensor-sensors))
required: no  
**co-keys**: [camera](#camera-sensor)  
**sub-keys**: [gyro_noise_density](#gyro_noise_density-imu), [accel_noise_density](#accel_noise_density-imu), [accel_random_walk](#accel_random_walk-imu), [gyro_random_walk](#gyro_random_walk-imu) 

 **description**: description of imu sensor 
<hr style="border:1px solid gray"> 

### **intrinsics** ([camera](#camera-sensor))
required: no  

 **description**: camera intrinsic parameter values to generate images format: [fx, fy, cx, cy] 
<hr style="border:1px solid gray"> 

### **jpl** ([file](#file-control_points))
required: no  

 **description**: boolean: if true, assume the file contains jpl quaternions, otherwise hamilton quaternions 
<hr style="border:1px solid gray"> 

### **main** ([trajectory_group](#trajectory_group-scene))
required: no  

 **description**: name of the main trajectory in the group 
<hr style="border:1px solid gray"> 

### **modify** ([trajectory](#trajectory-trajectory_group))
required: no  
**sub-keys**: [scale](#scale-modify), [rotate_about_point](#rotate_about_point-modify), [shift](#shift-modify) 

 **description**: NOT IMPLEMENTED modify a trajectory by scaling, shifting, and/or rotating about a point 
<hr style="border:1px solid gray"> 

### **negate** ([vec](#vec-align_axis))
required: no  

 **description**: boolean: if true, negate the vector(s) to align the specified axis 
<hr style="border:1px solid gray"> 

### **normal** ([planar_points](#planar_points-feature))
required: no  

 **description**: normal vector of the plane 
<hr style="border:1px solid gray"> 

### **num** ([planar_points](#planar_points-feature))
required: no  

 **description**: number of coplanar points to generate 
<hr style="border:1px solid gray"> 

### **num** ([random_points](#random_points-feature))
required: no  

 **description**: number of random points to generate 
<hr style="border:1px solid gray"> 

### **path** ([file](#file-control_points))
required: no  

 **description**: path to the file containing position and/or rotation data 
<hr style="border:1px solid gray"> 

### **planar_points** ([feature](#feature-features))
required: no  
**co-keys**: [points](#points-feature), [random_points](#random_points-feature)  
**sub-keys**: [radius](#radius-planar_points), [num](#num-planar_points), [normal](#normal-planar_points), [grid_spacing](#grid_spacing-planar_points), [center](#center-planar_points) 

 **description**: None 
<hr style="border:1px solid gray"> 

### **platform** ([scene](#scene))
required: yes  
**sub-keys**: [sensors](#sensors-platform), [body_frames](#body_frames-platform), [base_frame](#base_frame-platform), [id](#id-platform) 

 **description**: sensor platform containing sensors and/or body frames 
<hr style="border:1px solid gray"> 

### **point** ([rotate_about_point](#rotate_about_point-modify))
required: no  

 **description**: global position point about which to rotate the trajectory 
<hr style="border:1px solid gray"> 

### **points** ([feature](#feature-features))
required: no  

 **description**: list of 3D points 
<hr style="border:1px solid gray"> 

### **points** ([control_points](#control_points-bspline))
required: no  

 **description**: list of 3D position points to use as bspline control points 
<hr style="border:1px solid gray"> 

### **radius** ([planar_points](#planar_points-feature))
required: no  

 **description**: max distance from the center of the plane 
<hr style="border:1px solid gray"> 

### **radius** ([random_points](#random_points-feature))
required: no  

 **description**: max distance from the center of the point set 
<hr style="border:1px solid gray"> 

### **random_points** ([feature](#feature-features))
required: no  
**co-keys**: [points](#points-feature), [planar_points](#planar_points-feature)  
**sub-keys**: [radius](#radius-random_points), [num](#num-random_points), [center](#center-random_points) 

 **description**: arguments for random_point_set function 
<hr style="border:1px solid gray"> 

### **reference** ([trajectory_group](#trajectory_group-scene))
required: no  

 **description**: name of the reference trajectory in the group 
<hr style="border:1px solid gray"> 

### **res** ([bspline](#bspline-translation_trajectory))
required: no  

 **description**: resolution of the bspline curve 
<hr style="border:1px solid gray"> 

### **rotate_about_point** ([modify](#modify-trajectory))
required: no  
**sub-keys**: [point](#point-rotate_about_point), [rotation](#rotation-rotate_about_point) 

 **description**: rotate entire trajectory about a point in space - transforms both position and rotation values along the trajectory 
<hr style="border:1px solid gray"> 

### **rotation** ([transform](#transform-body_frames))
required: no  

 **description**: x-y-z fixed-axis euler angles in degrees from base frame to this sensor frame 
<hr style="border:1px solid gray"> 

### **rotation** ([rotate_about_point](#rotate_about_point-modify))
required: no  

 **description**:  NOT IMPLEMENTED: x-y-z fixed-axis euler angles in degrees to apply to the trajectory about a fixed point 
<hr style="border:1px solid gray"> 

### **rotation_only** ([subsample_interpolate](#subsample_interpolate-trajectory))
required: no  

 **description**: translation value in the trajectory will be zero at all times 
<hr style="border:1px solid gray"> 

### **rotation_trajectory** ([trajectory](#trajectory-trajectory_group))
required: no  
**co-keys**: [file](#file-trajectory), [translation_trajectory](#translation_trajectory-trajectory), [subsample_interpolate](#subsample_interpolate-trajectory), [copy](#copy-trajectory)  
**sub-keys**: [bspline_so3](#bspline_so3-rotation_trajectory), [constant](#constant-rotation_trajectory), [align_axis](#align_axis-rotation_trajectory), [file](#file-rotation_trajectory) 

 **description**: rotation component of the trajectory 
<hr style="border:1px solid gray"> 

### **scale** ([modify](#modify-trajectory))
required: no  

 **description**: NOT IMPLEMENTED: scale all position values in the trajectory by this value 
<hr style="border:1px solid gray"> 

### **scene**
required: no  
**sub-keys**: [trajectory_group](#trajectory_group-scene), [platform](#platform-scene), [features](#features-scene) 

 **description**: Specify a visual-inertial scene containing trajectory group, sensor platform, and features. 
<hr style="border:1px solid gray"> 

### **sensor** ([sensors](#sensors-platform))
required: no  
**sub-keys**: [transform](#transform-sensor), [rate](#rate-sensor), [time_offset](#time_offset-sensor), [id](#id-sensor), [imu](#imu-sensor), [camera](#camera-sensor), [enable_measurements](#enable_measurements-sensor) 

 **description**: camera or imu sensor rigidly attached to the sensor platform 
<hr style="border:1px solid gray"> 

### **shift** ([modify](#modify-trajectory))
required: no  

 **description**: NOT IMPLEMENTED: shift all position values in the trajectory by this position vector 
<hr style="border:1px solid gray"> 

### **span_time** ([bspline](#bspline-translation_trajectory))
required: no  

 **description**: time in seconds to traverse a single span of the bspline curve 
<hr style="border:1px solid gray"> 

### **subsample_interpolate** ([trajectory](#trajectory-trajectory_group))
required: no  
**co-keys**: [file](#file-trajectory), [translation_trajectory](#translation_trajectory-trajectory), [rotation_trajectory](#rotation_trajectory-trajectory), [copy](#copy-trajectory)  
**sub-keys**: [super_id](#super_id-subsample_interpolate), [control_point_rate](#control_point_rate-subsample_interpolate), [translation_only](#translation_only-subsample_interpolate), [degree](#degree-subsample_interpolate), [rotation_only](#rotation_only-subsample_interpolate) 

 **description**: subsample an existing trajectory and use the resulting positions as control points for a new trajectory 
<hr style="border:1px solid gray"> 

### **super_id** ([subsample_interpolate](#subsample_interpolate-trajectory))
required: no  

 **description**: existing id of the trajectory to subsample and use as control points 
<hr style="border:1px solid gray"> 

### **sensors** ([platform](#platform-scene))
required: no  

 **description**: list of sensors attached to the sensor platform 
<hr style="border:1px solid gray"> 

### **time_offset** ([sensor](#sensor-sensors))
required: no  

 **description**: NOT IMPLEMENTED: time offset in seconds to apply to the sensor measurements 
<hr style="border:1px solid gray"> 

### **time_unit** ([file](#file-control_points))
required: no  

 **description**: The unit of time used in the file. Allowed values: s, ms, us, ns 
<hr style="border:1px solid gray"> 

### **trajectories** ([trajectory_group](#trajectory_group-scene))
required: no  

 **description**: list of trajectories to compare 
<hr style="border:1px solid gray"> 

### **trajectory** ([trajectory_group](#trajectory_group-scene))
required: no  
**sub-keys**: [file](#file-trajectory), [id](#id-trajectory), [subsample_interpolate](#subsample_interpolate-trajectory), [copy](#copy-trajectory), [rotation_trajectory](#rotation_trajectory-trajectory), [translation_trajectory](#translation_trajectory-trajectory) 

 **description**: trajectory containing a sequence of position rotation values 
<hr style="border:1px solid gray"> 

### **trajectory_group** ([scene](#scene))
required: yes  
**sub-keys**: [reference](#reference-trajectory_group), [main](#main-trajectory_group), [trajectories](#trajectories-trajectory_group) 

 **description**: set of representations of a single trajectory to be compared - i.e. the estimate and the ground truth  
<hr style="border:1px solid gray"> 

### **transform** ([sensor](#sensor-sensors))
required: no  
**sub-keys**: [from](#from-transform), [translation](#translation-transform), [rotation](#rotation-transform) 

 **description**: Transformation to this sensor frame. By default, relative to the 'base frame' of this platform unless, 'from' is specified 
<hr style="border:1px solid gray"> 

### **transform** ([body_frames](#body_frames-platform))
required: no  
**sub-keys**: [from](#from-transform), [translation](#translation-transform), [rotation](#rotation-transform) 

 **description**: Transformation to this body frame. By default, relative to the 'base frame' of this platform, unless 'from' is specified 
<hr style="border:1px solid gray"> 

### **translation_trajectory** ([trajectory](#trajectory-trajectory_group))
required: no  
**co-keys**: [file](#file-trajectory), [rotation_trajectory](#rotation_trajectory-trajectory), [subsample_interpolate](#subsample_interpolate-trajectory), [copy](#copy-trajectory)  
**sub-keys**: [constant](#constant-translation_trajectory), [file](#file-translation_trajectory), [bspline](#bspline-translation_trajectory) 

 **description**: translation component of the trajectory 
<hr style="border:1px solid gray"> 

### **translation_trajectory** ([transform](#transform-body_frames))
required: no  

 **description**: translation from base frame to this sensor frame 
<hr style="border:1px solid gray"> 

### **translation_only** ([subsample_interpolate](#subsample_interpolate-trajectory))
required: no  

 **description**: rotation value in the trajectory will be identity at all times 
<hr style="border:1px solid gray"> 

### **vec** ([align_axis](#align_axis-rotation))
required: yes  
**sub-keys**: [negate](#negate-vec), [current_trajectory](#current_trajectory-vec) 

 **description**: vector(s) to align with the specified axis 
<hr style="border:1px solid gray"> 

### **width** ([camera](#camera-sensor))
required: no  

 **description**: width in pixels of the camera images to generate 
<hr style="border:1px solid gray"> 

