### **accel_noise_density** (imu)
required: no  
<pre> 
accelerometer noise density - units: (m/s^2) * (1/sqrt(Hz))
 </pre> 
<hr style="border:1px solid gray"> 

### **accel_random_walk** (imu)
required: no  
<pre> 
accelerometer rate random walk - units: (m/s^3) * (1/sqrt(Hz))
 </pre> 
<hr style="border:1px solid gray"> 

### **align_axis** (rotation)
required: no  
alternatives: [constant](#constant-rotation), [bspline_so3](#bspline_so3-rotation)  
**subkeys**: [flip](#flip-align_axis), [axis](#axis-align_axis), [grounded_axis](#grounded_axis-align_axis), [vec](#vec-align_axis) 
<pre> 
arguments for rotation_align_axis function
 </pre> 
<hr style="border:1px solid gray"> 

### **axis** (align_axis)
required: no  
<pre> 
string, 'x','y', or 'z' axis to align with 'vec'
 </pre> 
<hr style="border:1px solid gray"> 

### **base_frame** (platform)
required: no  
<pre> 
name of the base frame of the sensor platform. This is the frame that will follow the described trajectory, and the frame that the sensor and body frames are defined relative to, by default
 </pre> 
<hr style="border:1px solid gray"> 

### **body_frame** (body_frames)
required: no  
**subkeys**: [id](#id-body_frame), [transform](#transform-body_frame) 
<pre> 
fixed frame attached to the sensor platform
 </pre> 
<hr style="border:1px solid gray"> 

### **body_frames** (platform)
required: no  
<pre> 
list of body frames attached to the sensor platform
 </pre> 
<hr style="border:1px solid gray"> 

### **bspline** (translation)
required: no  
alternatives: [file](#file-translation), [constant](#constant-translation)  
**subkeys**: [span_time](#span_time-bspline), [res](#res-bspline), [control_points](#control_points-bspline), [degree](#degree-bspline) 
<pre> 
uniform bspline trajectory - generates spline of position values from set of control points
 </pre> 
<hr style="border:1px solid gray"> 

### **bspline_so3** (rotation)
required: no  
alternatives: [align_axis](#align_axis-rotation), [constant](#constant-rotation)  
<pre> 
NOT IMPLEMENTED: bspline rotation trajectory - generate spline of rotation values from set of control rotations
 </pre> 
<hr style="border:1px solid gray"> 

### **camera** (sensor)
required: no  
alternatives: [imu](#imu-sensor)  
**subkeys**: [width](#width-camera), [transform](#transform-camera), [intrinsics](#intrinsics-camera), [distortion](#distortion-camera), [height](#height-camera) 
<pre> 
description of camera sensor
 </pre> 
<hr style="border:1px solid gray"> 

### **center** (random_points)
required: no  
<pre> 
position center around which to generate the random points
 </pre> 
<hr style="border:1px solid gray"> 

### **color** (feature)
required: no  
<pre> 
color of all points in the set, given as string e.g. 'red', 'blue', 'green'. If not given, a repeating sequence of colors will be assigned
 </pre> 
<hr style="border:1px solid gray"> 

### **constant** (rotation)
required: no  
<pre> 
NOT IMPLEMENTED: constant rotation value to apply to the trajectory
 </pre> 
<hr style="border:1px solid gray"> 

### **control_points** (bspline)
required: yes  
**subkeys**: [points](#points-control_points), [file](#file-control_points) 
<pre> 
The set of 3D position points to use as bspline control points. For degree k, there must be at least k+1 control points
 </pre> 
<hr style="border:1px solid gray"> 

### **control_point_rate** (subsample_interpolate)
required: no  
<pre> 
rate at which to sample control points from the original trajectory
 </pre> 
<hr style="border:1px solid gray"> 

### **copy** (trajectory)
required: no  
<pre> 
NOT IMPLEMENTED: copy an existing trajectory
 </pre> 
<hr style="border:1px solid gray"> 

### **current_trajectory** (vec)
required: no  
<pre> 
Use the current trajectory's position, velocity, acceleration, or the bearing vectors from each position to its centroid as the source of vectors to align with 'axis'. Allowed values: 'pos', 'vel', 'acc', 'centroid'
 </pre> 
<hr style="border:1px solid gray"> 

### **degree** (subsample_interpolate)
required: no  
<pre> 
degree of the bspline polynomial - equal to the bspline order minus 1
 </pre> 
<hr style="border:1px solid gray"> 

### **distortion** (camera)
required: no  
<pre> 
radtan lens distortion parameters to generate images format: [k1, k2, p1, p2]
 </pre> 
<hr style="border:1px solid gray"> 

### **enable_measurements** (sensor)
required: no  
<pre> 
boolean: if true, compute simulated measurements from this sensor
 </pre> 
<hr style="border:1px solid gray"> 

### **feature** (features)
required: no  
**subkeys**: [id](#id-feature), [random_points](#random_points-feature), [points](#points-feature), [planar_points](#planar_points-feature), [color](#color-feature) 
<pre> 
set of points and corresponding colors
 </pre> 
<hr style="border:1px solid gray"> 

### **features** (scene)
required: no  
<pre> 
list of features
 </pre> 
<hr style="border:1px solid gray"> 

### **file** (trajectory)
required: no  
alternatives: [translation](#translation-trajectory), [rotation](#rotation-trajectory), [subsample_interpolate](#subsample_interpolate-trajectory), [copy](#copy-trajectory)  
**subkeys**: [format](#format-file), [path](#path-file), [jpl](#jpl-file), [time_unit](#time_unit-file) 
<pre> 
file information for the trajectory data
 </pre> 
<hr style="border:1px solid gray"> 

### **file** (control_points)
required: no  
alternatives: [points](#points-control_points)  
**subkeys**: [format](#format-file), [path](#path-file) 
<pre> 
NOT IMPLEMENTED: file containing list of 3D position points to use as bspline control points
 </pre> 
<hr style="border:1px solid gray"> 

### **flip** (align_axis)
required: no  
<pre> 
boolean: if true, the 'grounded' axis will be negated, giving the other available solution
 </pre> 
<hr style="border:1px solid gray"> 

### **format** (file)
required: no  
<pre> 
format of the file containing position and/or rotation data example: 't x y z qx qy qz qw' separated by spaces or comma according to file, use dashes to indicate columns to ignore, e.g. 't,x,y,z,-,-,-,qx,qy,qz,qw' 
 </pre> 
<hr style="border:1px solid gray"> 

### **from** (transform)
required: no  
<pre> 
name of the parent frame, relative to which to define the transform
 </pre> 
<hr style="border:1px solid gray"> 

### **grounded_axis** (align_axis)
required: no  
<pre> 
string, 'x','y', or 'z' axis to remain parallel to the original xy-plane. cannot be the same as 'axis' param. if not provided, it will be the next sequential axis after 'axis' (x->y, y->z, z->x).
 </pre> 
<hr style="border:1px solid gray"> 

### **gyro_noise_density** (imu)
required: no  
<pre> 
gyro noise density - units: (rad/s^2) * (1/sqrt(Hz))
 </pre> 
<hr style="border:1px solid gray"> 

### **gyro_random_walk** (imu)
required: no  
<pre> 
gyro rate random walk - units: (rad/s^2) * (1/sqrt(Hz))
 </pre> 
<hr style="border:1px solid gray"> 

### **height** (camera)
required: no  
<pre> 
height in pixels of the camera images to generate
 </pre> 
<hr style="border:1px solid gray"> 

### **id** (trajectory)
required: no  
<pre> 
name of the trajectory
 </pre> 
<hr style="border:1px solid gray"> 

### **id** (platform)
required: no  
<pre> 
name of the platform
 </pre> 
<hr style="border:1px solid gray"> 

### **id** (sensor)
required: no  
<pre> 
name of the sensor
 </pre> 
<hr style="border:1px solid gray"> 

### **id** (feature)
required: no  
<pre> 
name of the feature
 </pre> 
<hr style="border:1px solid gray"> 

### **id** (body_frame)
required: no  
<pre> 
name of the body frame
 </pre> 
<hr style="border:1px solid gray"> 

### **imu** (sensor)
required: no  
alternatives: [camera](#camera-sensor)  
**subkeys**: [gyro_noise_density](#gyro_noise_density-imu), [accel_random_walk](#accel_random_walk-imu), [gyro_random_walk](#gyro_random_walk-imu), [accel_noise_density](#accel_noise_density-imu) 
<pre> 
description of imu sensor
 </pre> 
<hr style="border:1px solid gray"> 

### **intrinsics** (camera)
required: no  
<pre> 
camera intrinsic parameter values to generate images format: [fx, fy, cx, cy]
 </pre> 
<hr style="border:1px solid gray"> 

### **jpl** (file)
required: no  
<pre> 
boolean: if true, assume the file contains jpl quaternions, otherwise hamilton quaternions
 </pre> 
<hr style="border:1px solid gray"> 

### **main** (trajectory_group)
required: no  
<pre> 
name of the main trajectory in the group
 </pre> 
<hr style="border:1px solid gray"> 

### **modify** (trajectory)
required: no  
**subkeys**: [scale](#scale-modify), [shift](#shift-modify), [rotate_about_point](#rotate_about_point-modify) 
<pre> 
NOT IMPLEMENTED modify a trajectory by scaling, shifting, and/or rotating about a point
 </pre> 
<hr style="border:1px solid gray"> 

### **negate** (vec)
required: no  
<pre> 
boolean: if true, negate the vector(s) to align the specified axis
 </pre> 
<hr style="border:1px solid gray"> 

### **normal** (planar_points)
required: no  
<pre> 
normal vector of the plane
 </pre> 
<hr style="border:1px solid gray"> 

### **num** (planar_points)
required: no  
<pre> 
number of coplanar points to generate
 </pre> 
<hr style="border:1px solid gray"> 

### **num** (random_points)
required: no  
<pre> 
number of random points to generate
 </pre> 
<hr style="border:1px solid gray"> 

### **path** (file)
required: no  
<pre> 
path to the file containing position and/or rotation data
 </pre> 
<hr style="border:1px solid gray"> 

### **planar_points** (feature)
required: no  
alternatives: [points](#points-feature), [random_points](#random_points-feature)  
**subkeys**: [center](#center-planar_points), [normal](#normal-planar_points), [num](#num-planar_points), [radius](#radius-planar_points), [grid_spacing](#grid_spacing-planar_points) 
<pre> 
None
 </pre> 
<hr style="border:1px solid gray"> 

### **platform** (scene)
required: yes  
**subkeys**: [sensors](#sensors-platform), [id](#id-platform), [base_frame](#base_frame-platform), [body_frames](#body_frames-platform) 
<pre> 
sensor platform containing sensors and/or body frames
 </pre> 
<hr style="border:1px solid gray"> 

### **points** (feature)
required: no  
<pre> 
list of 3D points
 </pre> 
<hr style="border:1px solid gray"> 

### **points** (control_points)
required: no  
<pre> 
list of 3D position points to use as bspline control points
 </pre> 
<hr style="border:1px solid gray"> 

### **radius** (planar_points)
required: no  
<pre> 
max distance from the center of the plane
 </pre> 
<hr style="border:1px solid gray"> 

### **radius** (random_points)
required: no  
<pre> 
max distance from the center of the point set
 </pre> 
<hr style="border:1px solid gray"> 

### **random_points** (feature)
required: no  
alternatives: [points](#points-feature), [planar_points](#planar_points-feature)  
**subkeys**: [center](#center-random_points), [radius](#radius-random_points), [num](#num-random_points) 
<pre> 
arguments for random_point_set function
 </pre> 
<hr style="border:1px solid gray"> 

### **reference** (trajectory_group)
required: no  
<pre> 
name of the reference trajectory in the group
 </pre> 
<hr style="border:1px solid gray"> 

### **res** (bspline)
required: no  
<pre> 
resolution of the bspline curve
 </pre> 
<hr style="border:1px solid gray"> 

### **rotate_about_point** (modify)
required: no  
**subkeys**: [rotation](#rotation-rotate_about_point) 
<pre> 
rotate entire trajectory about a point in space - transforms both position and rotation values along the trajectory
 </pre> 
<hr style="border:1px solid gray"> 

### **rotation** (trajectory)
required: no  
alternatives: [file](#file-trajectory), [translation](#translation-trajectory), [subsample_interpolate](#subsample_interpolate-trajectory), [copy](#copy-trajectory)  
**subkeys**: [constant](#constant-rotation), [bspline_so3](#bspline_so3-rotation), [align_axis](#align_axis-rotation) 
<pre> 
rotation trajectory - generate a sequence of rotation values over time
 </pre> 
<hr style="border:1px solid gray"> 

### **rotation** (transform)
required: no  
<pre> 
x-y-z fixed-axis euler angles in degrees from base frame to this sensor frame
 </pre> 
<hr style="border:1px solid gray"> 

### **rotation** (rotate_about_point)
required: no  
<pre> 
 NOT IMPLEMENTED: x-y-z fixed-axis euler angles in degrees to apply to the trajectory about a fixed point
 </pre> 
<hr style="border:1px solid gray"> 

### **rotation_only** (subsample_interpolate)
required: no  
<pre> 
translation value in the trajectory will be zero at all times
 </pre> 
<hr style="border:1px solid gray"> 

### **scale** (modify)
required: no  
<pre> 
NOT IMPLEMENTED: scale all position values in the trajectory by this value
 </pre> 
<hr style="border:1px solid gray"> 

### **scene** ()
required: no  
**subkeys**: [features](#features-scene), [platform](#platform-scene), [trajectory_group](#trajectory_group-scene) 
<pre> 
Specify a visual-inertial scene containing trajectory group, sensor platform, and features.
 </pre> 
<hr style="border:1px solid gray"> 

### **sensor** (sensors)
required: no  
**subkeys**: [imu](#imu-sensor), [id](#id-sensor), [transform](#transform-sensor), [time_offset](#time_offset-sensor), [camera](#camera-sensor), [rate](#rate-sensor), [enable_measurements](#enable_measurements-sensor) 
<pre> 
camera or imu sensor rigidly attached to the sensor platform
 </pre> 
<hr style="border:1px solid gray"> 

### **shift** (modify)
required: no  
<pre> 
NOT IMPLEMENTED: shift all position values in the trajectory by this position vector
 </pre> 
<hr style="border:1px solid gray"> 

### **span_time** (bspline)
required: no  
<pre> 
time in seconds to traverse a single span of the bspline curve
 </pre> 
<hr style="border:1px solid gray"> 

### **subsample_interpolate** (trajectory)
required: no  
alternatives: [file](#file-trajectory), [translation](#translation-trajectory), [rotation](#rotation-trajectory), [copy](#copy-trajectory)  
**subkeys**: [translation_only](#translation_only-subsample_interpolate), [control_point_rate](#control_point_rate-subsample_interpolate), [rotation_only](#rotation_only-subsample_interpolate), [super_id](#super_id-subsample_interpolate), [degree](#degree-subsample_interpolate) 
<pre> 
subsample an existing trajectory and use the resulting positions as control points for a new trajectory
 </pre> 
<hr style="border:1px solid gray"> 

### **super_id** (subsample_interpolate)
required: no  
<pre> 
existing id of the trajectory to subsample and use as control points
 </pre> 
<hr style="border:1px solid gray"> 

### **sensors** (platform)
required: no  
<pre> 
list of sensors attached to the sensor platform
 </pre> 
<hr style="border:1px solid gray"> 

### **time_offset** (sensor)
required: no  
<pre> 
NOT IMPLEMENTED: time offset in seconds to apply to the sensor measurements
 </pre> 
<hr style="border:1px solid gray"> 

### **time_unit** (file)
required: no  
<pre> 
The unit of time used in the file. Allowed values: s, ms, us, ns
 </pre> 
<hr style="border:1px solid gray"> 

### **trajectory** (trajectory_group)
required: no  
**subkeys**: [id](#id-trajectory), [file](#file-trajectory), [copy](#copy-trajectory), [subsample_interpolate](#subsample_interpolate-trajectory) 
<pre> 
trajectory containing a sequence of position rotation values
 </pre> 
<hr style="border:1px solid gray"> 

### **trajectory_group** (scene)
required: yes  
**subkeys**: [trajectories](#trajectories-trajectory_group), [reference](#reference-trajectory_group), [main](#main-trajectory_group) 
<pre> 
set of representations of a single trajectory to be compared - i.e. the estimate and the ground truth 
 </pre> 
<hr style="border:1px solid gray"> 

### **transform** (sensor)
required: no  
**subkeys**: [rotation](#rotation-transform), [translation](#translation-transform), [from](#from-transform) 
<pre> 
Transformation to this sensor frame. By default, relative to the 'base frame' of this platform unless, 'from' is specified
 </pre> 
<hr style="border:1px solid gray"> 

### **transform** (body_frame)
required: yes  
**subkeys**: [rotation](#rotation-transform), [from](#from-transform), [translation](#translation-transform), [jpl](#jpl-transform) 
<pre> 
Transformation to this body frame. By default, relative to the 'base frame' of this platform, unless 'from' is specified
 </pre> 
<hr style="border:1px solid gray"> 

### **translation** (trajectory)
required: no  
alternatives: [file](#file-trajectory), [rotation](#rotation-trajectory), [subsample_interpolate](#subsample_interpolate-trajectory), [copy](#copy-trajectory)  
**subkeys**: [constant](#constant-translation), [bspline](#bspline-translation), [file](#file-translation) 
<pre> 
translation component of the trajectory
 </pre> 
<hr style="border:1px solid gray"> 

### **translation** (transform)
required: no  
<pre> 
translation from base frame to this sensor frame
 </pre> 
<hr style="border:1px solid gray"> 

### **translation_only** (subsample_interpolate)
required: no  
<pre> 
rotation value in the trajectory will be identity at all times
 </pre> 
<hr style="border:1px solid gray"> 

### **vec** (align_axis)
required: yes  
**subkeys**: [negate](#negate-vec), [current_trajectory](#current_trajectory-vec) 
<pre> 
vector(s) to align with the specified axis
 </pre> 
<hr style="border:1px solid gray"> 

### **width** (camera)
required: no  
<pre> 
width in pixels of the camera images to generate
 </pre> 
<hr style="border:1px solid gray"> 

