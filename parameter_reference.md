# Parameter Reference 
This document is automatically generated based on the contents of [valid_keys.yaml](config/valid_keys.yaml). 
It describes the allowed parameters in a VINLAB configuration file, for example: [example_config_01](config/example_config_01.yaml). 
Each parameter name (key) is listed here alphabetically, along with it's context (parent key), in the form: **name**(context).
Some parameters may have more than one context, and may function differently for each context, in which case there is an entry here for each. 
For each param, the corresponding **co-params** are the other params that may be present in the same context (same indentation level in the yaml file). 
Similarly, the corresponding **sub-params**  are the parameters expected under the param, at the next indentation level. 
Not all of the parameters are required, and some may be mutually exclusive. 
The the sub-keys of [**scene**](#scene) should be the top-level parameters in the file. 
If the yaml file contains any keys that are unknown or in the wrong context, a ConfigurationError will be raised.
<hr style="border:1px solid gray"> 

### **accel_noise_density** ([imu](#imu-sensor))

 **description**: accelerometer noise density - units: (m/s^2) * (1/sqrt(Hz)) 
<hr style="border:1px solid gray"> 

### **accel_random_walk** ([imu](#imu-sensor))

 **description**: accelerometer rate random walk - units: (m/s^3) * (1/sqrt(Hz)) 
<hr style="border:1px solid gray"> 

### **align_axis** ([rotation_trajectory](#rotation_trajectory-trajectory))
**co-params**: [bspline_so3](#bspline_so3-rotation_trajectory), [file](#file-rotation_trajectory), [constant](#constant-rotation_trajectory)  
**sub-params**: [grounded_axis](#grounded_axis-align_axis), [vec](#vec-align_axis), [flip](#flip-align_axis), [axis](#axis-align_axis) 

 **description**: arguments for the [rotation_align_axis function](/vinlab/src/geometry_utils.py). Compute the rotation(s) that aligns given axis with the given vector(s), and keeps one other axis parallel to the original xy-plane (ground plane) 
<hr style="border:1px solid gray"> 

### **align_axis** ([rotation](#rotation-rotate_about_point))
**sub-params**: [grounded_axis](#grounded_axis-align_axis), [vec](#vec-align_axis), [flip](#flip-align_axis), [axis](#axis-align_axis) 

 **description**: arguments for the [rotation_align_axis function](/vinlab/src/geometry_utils.py) 
<hr style="border:1px solid gray"> 

### **axis** ([align_axis](#align_axis-rotation))

 **description**: string, 'x','y', or 'z' axis to align with 'vec' 
<hr style="border:1px solid gray"> 

### **base_frame** ([platform](#platform-scene))

 **description**: name of the base frame of the sensor platform. This is the frame that will follow the described trajectory, and the frame that the sensor and body frames are defined relative to, by default 
<hr style="border:1px solid gray"> 

### **body_frame** ([body_frames](#body_frames-platform))
**sub-params**: [id](#id-body_frame), [transform](#transform-body_frame) 

 **description**: fixed frame attached to the sensor platform 
<hr style="border:1px solid gray"> 

### **body_frames** ([platform](#platform-scene))

 **description**: list of body frames attached to the sensor platform 
<hr style="border:1px solid gray"> 

### **bspline** ([translation_trajectory](#translation_trajectory-trajectory))
**co-params**: [file](#file-translation_trajectory), [constant](#constant-translation_trajectory)  
**sub-params**: [span_time](#span_time-bspline), [control_points](#control_points-bspline), [degree](#degree-bspline), [res](#res-bspline) 

 **description**: uniform bspline trajectory - generates spline of position values from set of control points 
<hr style="border:1px solid gray"> 

### **bspline_so3** ([rotation_trajectory](#rotation_trajectory-trajectory))
**co-params**: [file](#file-rotation_trajectory), [constant](#constant-rotation_trajectory), [align_axis](#align_axis-rotation_trajectory)  

 **description**: NOT IMPLEMENTED: bspline rotation trajectory - generate spline of rotation values from set of control rotations 
<hr style="border:1px solid gray"> 

### **camera** ([sensor](#sensor-sensors))
**co-params**: [imu](#imu-sensor)  
**sub-params**: [height](#height-camera), [intrinsics](#intrinsics-camera), [width](#width-camera), [distortion](#distortion-camera) 

 **description**: description of camera sensor 
<hr style="border:1px solid gray"> 

### **center** ([planar_points](#planar_points-feature))

 **description**: position of the center of the plane 
<hr style="border:1px solid gray"> 

### **center** ([random_points](#random_points-feature))

 **description**: position center around which to generate the random points 
<hr style="border:1px solid gray"> 

### **color** ([feature](#feature-features))

 **description**: color of all points in the set, given as string e.g. 'red', 'blue', 'green'. If not given, a repeating sequence of colors will be assigned 
<hr style="border:1px solid gray"> 

### **constant** ([rotation_trajectory](#rotation_trajectory-trajectory))

 **description**: NOT IMPLEMENTED: constant rotation value to apply to the trajectory 
<hr style="border:1px solid gray"> 

### **control_points** ([bspline](#bspline-translation_trajectory))
**sub-params**: [points](#points-control_points), [file](#file-control_points) 

 **description**: The set of 3D position points to use as bspline control points. For degree k, there must be at least k+1 control points 
<hr style="border:1px solid gray"> 

### **control_point_rate** ([subsample_interpolate](#subsample_interpolate-trajectory))

 **description**: rate at which to sample control points from the original trajectory 
<hr style="border:1px solid gray"> 

### **copy** ([trajectory](#trajectory-trajectory_group))

 **description**: NOT IMPLEMENTED: copy an existing trajectory 
<hr style="border:1px solid gray"> 

### **current_trajectory** ([vec](#vec-align_axis))

 **description**: Use the current trajectory's position, velocity, acceleration, or the bearing vectors from each position to its centroid as the source of vectors to align with 'axis'. Allowed values: 'pos', 'vel', 'acc', 'centroid' 
<hr style="border:1px solid gray"> 

### **degree** ([subsample_interpolate](#subsample_interpolate-trajectory))

 **description**: degree of the bspline polynomial - equal to the bspline order minus 1 
<hr style="border:1px solid gray"> 

### **distortion** ([camera](#camera-sensor))

 **description**: radtan lens distortion parameters to generate images format: [k1, k2, p1, p2] 
<hr style="border:1px solid gray"> 

### **enable_measurements** ([sensor](#sensor-sensors))

 **description**: boolean: if true, compute simulated measurements from this sensor 
<hr style="border:1px solid gray"> 

### **feature** ([features](#features-scene))
**sub-params**: [planar_points](#planar_points-feature), [points](#points-feature), [id](#id-feature), [color](#color-feature), [random_points](#random_points-feature) 

 **description**: set of points and corresponding colors 
<hr style="border:1px solid gray"> 

### **features** ([scene](#scene))

 **description**: list of features 
<hr style="border:1px solid gray"> 

### **file** ([trajectory](#trajectory-trajectory_group))
**co-params**: [translation_trajectory](#translation_trajectory-trajectory), [rotation_trajectory](#rotation_trajectory-trajectory), [subsample_interpolate](#subsample_interpolate-trajectory), [copy](#copy-trajectory)  
**sub-params**: [format](#format-file), [time_unit](#time_unit-file), [path](#path-file), [jpl](#jpl-file) 

 **description**: file information for the trajectory data 
<hr style="border:1px solid gray"> 

### **file** ([control_points](#control_points-bspline))
**co-params**: [points](#points-control_points)  
**sub-params**: [format](#format-file), [path](#path-file) 

 **description**: NOT IMPLEMENTED: file containing list of 3D position points to use as bspline control points 
<hr style="border:1px solid gray"> 

### **flip** ([align_axis](#align_axis-rotation))

 **description**: boolean: if true, the 'grounded' axis will be negated, giving the other available solution 
<hr style="border:1px solid gray"> 

### **format** ([file](#file-control_points))

 **description**: format of the file containing position and/or rotation data example: 't x y z qx qy qz qw' separated by spaces or comma according to file, use dashes to indicate columns to ignore, e.g. 't,x,y,z,-,-,-,qx,qy,qz,qw'  
<hr style="border:1px solid gray"> 

### **from** ([transform](#transform-body_frame))

 **description**: name of the parent frame, relative to which to define the transform 
<hr style="border:1px solid gray"> 

### **grid_spacing** ([planar_points](#planar_points-feature))

 **description**: uniform spacing used to construct grid of points on the plane 
<hr style="border:1px solid gray"> 

### **grounded_axis** ([align_axis](#align_axis-rotation))

 **description**: string, 'x','y', or 'z' axis to remain parallel to the original xy-plane. cannot be the same as 'axis' param. if not provided, it will be the next sequential axis after 'axis' (x->y, y->z, z->x). 
<hr style="border:1px solid gray"> 

### **gyro_noise_density** ([imu](#imu-sensor))

 **description**: gyro noise density - units: (rad/s^2) * (1/sqrt(Hz)) 
<hr style="border:1px solid gray"> 

### **gyro_random_walk** ([imu](#imu-sensor))

 **description**: gyro rate random walk - units: (rad/s^2) * (1/sqrt(Hz)) 
<hr style="border:1px solid gray"> 

### **height** ([camera](#camera-sensor))

 **description**: height in pixels of the camera images to generate 
<hr style="border:1px solid gray"> 

### **id** ([trajectory](#trajectory-trajectory_group))

 **description**: name of the trajectory 
<hr style="border:1px solid gray"> 

### **id** ([platform](#platform-scene))

 **description**: name of the platform 
<hr style="border:1px solid gray"> 

### **id** ([sensor](#sensor-sensors))

 **description**: name of the sensor 
<hr style="border:1px solid gray"> 

### **id** ([feature](#feature-features))

 **description**: name of the feature 
<hr style="border:1px solid gray"> 

### **id** ([body_frame](#body_frame-body_frames))

 **description**: name of the body frame 
<hr style="border:1px solid gray"> 

### **imu** ([sensor](#sensor-sensors))
**co-params**: [camera](#camera-sensor)  
**sub-params**: [accel_random_walk](#accel_random_walk-imu), [accel_noise_density](#accel_noise_density-imu), [gyro_random_walk](#gyro_random_walk-imu), [gyro_noise_density](#gyro_noise_density-imu) 

 **description**: description of imu sensor 
<hr style="border:1px solid gray"> 

### **intrinsics** ([camera](#camera-sensor))

 **description**: camera intrinsic parameter values to generate images format: [fx, fy, cx, cy] 
<hr style="border:1px solid gray"> 

### **jpl** ([file](#file-control_points))

 **description**: boolean: if true, assume the file contains jpl quaternions, otherwise hamilton quaternions 
<hr style="border:1px solid gray"> 

### **main** ([trajectory_group](#trajectory_group-scene))

 **description**: name of the main trajectory in the group 
<hr style="border:1px solid gray"> 

### **modify** ([trajectory](#trajectory-trajectory_group))
**sub-params**: [shift](#shift-modify), [rotate_about_point](#rotate_about_point-modify), [scale](#scale-modify) 

 **description**: NOT IMPLEMENTED modify a trajectory by scaling, shifting, and/or rotating about a point 
<hr style="border:1px solid gray"> 

### **negate** ([vec](#vec-align_axis))

 **description**: boolean: if true, negate the vector(s) to align the specified axis 
<hr style="border:1px solid gray"> 

### **normal** ([planar_points](#planar_points-feature))

 **description**: normal vector of the plane 
<hr style="border:1px solid gray"> 

### **num** ([planar_points](#planar_points-feature))

 **description**: number of coplanar points to generate 
<hr style="border:1px solid gray"> 

### **num** ([random_points](#random_points-feature))

 **description**: number of random points to generate 
<hr style="border:1px solid gray"> 

### **path** ([file](#file-control_points))

 **description**: path to the file containing position and/or rotation data 
<hr style="border:1px solid gray"> 

### **planar_points** ([feature](#feature-features))
**co-params**: [points](#points-feature), [random_points](#random_points-feature)  
**sub-params**: [num](#num-planar_points), [radius](#radius-planar_points), [center](#center-planar_points), [grid_spacing](#grid_spacing-planar_points), [normal](#normal-planar_points) 

 **description**: None 
<hr style="border:1px solid gray"> 

### **platform** ([scene](#scene))
**sub-params**: [base_frame](#base_frame-platform), [id](#id-platform), [body_frames](#body_frames-platform), [sensors](#sensors-platform) 

 **description**: sensor platform containing sensors and/or body frames 
<hr style="border:1px solid gray"> 

### **point** ([rotate_about_point](#rotate_about_point-modify))

 **description**: global position point about which to rotate the trajectory 
<hr style="border:1px solid gray"> 

### **points** ([feature](#feature-features))

 **description**: list of 3D points 
<hr style="border:1px solid gray"> 

### **points** ([control_points](#control_points-bspline))

 **description**: list of 3D position points to use as bspline control points 
<hr style="border:1px solid gray"> 

### **radius** ([planar_points](#planar_points-feature))

 **description**: max distance from the center of the plane 
<hr style="border:1px solid gray"> 

### **radius** ([random_points](#random_points-feature))

 **description**: max distance from the center of the point set 
<hr style="border:1px solid gray"> 

### **random_points** ([feature](#feature-features))
**co-params**: [points](#points-feature), [planar_points](#planar_points-feature)  
**sub-params**: [radius](#radius-random_points), [center](#center-random_points), [num](#num-random_points) 

 **description**: arguments for random_point_set function 
<hr style="border:1px solid gray"> 

### **reference** ([trajectory_group](#trajectory_group-scene))

 **description**: name of the reference trajectory in the group 
<hr style="border:1px solid gray"> 

### **res** ([bspline](#bspline-translation_trajectory))

 **description**: resolution of the bspline curve 
<hr style="border:1px solid gray"> 

### **rotate_about_point** ([modify](#modify-trajectory))
**sub-params**: [point](#point-rotate_about_point), [rotation](#rotation-rotate_about_point) 

 **description**: rotate entire trajectory about a point in space - transforms both position and rotation values along the trajectory 
<hr style="border:1px solid gray"> 

### **rotation** ([transform](#transform-body_frame))

 **description**: x-y-z fixed-axis euler angles in degrees from base frame to this sensor frame 
<hr style="border:1px solid gray"> 

### **rotation** ([rotate_about_point](#rotate_about_point-modify))

 **description**:  NOT IMPLEMENTED: x-y-z fixed-axis euler angles in degrees to apply to the trajectory about a fixed point 
<hr style="border:1px solid gray"> 

### **rotation_only** ([subsample_interpolate](#subsample_interpolate-trajectory))

 **description**: translation value in the trajectory will be zero at all times 
<hr style="border:1px solid gray"> 

### **rotation_trajectory** ([trajectory](#trajectory-trajectory_group))
**co-params**: [file](#file-trajectory), [translation_trajectory](#translation_trajectory-trajectory), [subsample_interpolate](#subsample_interpolate-trajectory), [copy](#copy-trajectory)  
**sub-params**: [constant](#constant-rotation_trajectory), [align_axis](#align_axis-rotation_trajectory), [file](#file-rotation_trajectory), [bspline_so3](#bspline_so3-rotation_trajectory) 

 **description**: rotation component of the trajectory 
<hr style="border:1px solid gray"> 

### **scale** ([modify](#modify-trajectory))

 **description**: NOT IMPLEMENTED: scale all position values in the trajectory by this value 
<hr style="border:1px solid gray"> 

### **scene**
**sub-params**: [platform](#platform-scene), [trajectory_group](#trajectory_group-scene), [features](#features-scene) 

 **description**: Specify a visual-inertial scene containing trajectory group, sensor platform, and features. 
<hr style="border:1px solid gray"> 

### **sensor** ([sensors](#sensors-platform))
**sub-params**: [id](#id-sensor), [camera](#camera-sensor), [enable_measurements](#enable_measurements-sensor), [rate](#rate-sensor), [imu](#imu-sensor), [transform](#transform-sensor), [time_offset](#time_offset-sensor) 

 **description**: camera or imu sensor rigidly attached to the sensor platform 
<hr style="border:1px solid gray"> 

### **sensors** ([platform](#platform-scene))

 **description**: list of sensors attached to the sensor platform 
<hr style="border:1px solid gray"> 

### **shift** ([modify](#modify-trajectory))

 **description**: NOT IMPLEMENTED: shift all position values in the trajectory by this position vector 
<hr style="border:1px solid gray"> 

### **span_time** ([bspline](#bspline-translation_trajectory))

 **description**: time in seconds to traverse a single span of the bspline curve 
<hr style="border:1px solid gray"> 

### **subsample_interpolate** ([trajectory](#trajectory-trajectory_group))
**co-params**: [file](#file-trajectory), [translation_trajectory](#translation_trajectory-trajectory), [rotation_trajectory](#rotation_trajectory-trajectory), [copy](#copy-trajectory)  
**sub-params**: [control_point_rate](#control_point_rate-subsample_interpolate), [super_id](#super_id-subsample_interpolate), [translation_only](#translation_only-subsample_interpolate), [rotation_only](#rotation_only-subsample_interpolate), [degree](#degree-subsample_interpolate) 

 **description**: subsample an existing trajectory and use the resulting positions as control points for a new trajectory 
<hr style="border:1px solid gray"> 

### **super_id** ([subsample_interpolate](#subsample_interpolate-trajectory))

 **description**: existing id of the trajectory to subsample and use as control points 
<hr style="border:1px solid gray"> 

### **time_offset** ([sensor](#sensor-sensors))

 **description**: NOT IMPLEMENTED: time offset in seconds to apply to the sensor measurements 
<hr style="border:1px solid gray"> 

### **time_unit** ([file](#file-control_points))

 **description**: The unit of time used in the file. Allowed values: s, ms, us, ns 
<hr style="border:1px solid gray"> 

### **trajectories** ([trajectory_group](#trajectory_group-scene))

 **description**: list of trajectories to compare 
<hr style="border:1px solid gray"> 

### **trajectory** ([trajectory_group](#trajectory_group-scene))
**sub-params**: [id](#id-trajectory), [copy](#copy-trajectory), [file](#file-trajectory), [subsample_interpolate](#subsample_interpolate-trajectory), [rotation_trajectory](#rotation_trajectory-trajectory), [translation_trajectory](#translation_trajectory-trajectory) 

 **description**: trajectory containing a sequence of position rotation values 
<hr style="border:1px solid gray"> 

### **trajectory_group** ([scene](#scene))
**sub-params**: [main](#main-trajectory_group), [trajectories](#trajectories-trajectory_group), [reference](#reference-trajectory_group) 

 **description**: set of representations of a single trajectory to be compared - i.e. the estimate and the ground truth  
<hr style="border:1px solid gray"> 

### **transform** ([sensor](#sensor-sensors))
**sub-params**: [from](#from-transform), [rotation](#rotation-transform), [translation](#translation-transform) 

 **description**: Transformation to this sensor frame. By default, relative to the 'base frame' of this platform unless, 'from' is specified 
<hr style="border:1px solid gray"> 

### **transform** ([body_frame](#body_frame-body_frames))
**sub-params**: [from](#from-transform), [rotation](#rotation-transform), [translation](#translation-transform) 

 **description**: Transformation to this body frame. By default, relative to the 'base frame' of this platform, unless 'from' is specified 
<hr style="border:1px solid gray"> 

### **translation** ([transform](#transform-body_frame))

 **description**: translation from base frame to this sensor frame 
<hr style="border:1px solid gray"> 

### **translation_trajectory** ([trajectory](#trajectory-trajectory_group))
**co-params**: [file](#file-trajectory), [rotation_trajectory](#rotation_trajectory-trajectory), [subsample_interpolate](#subsample_interpolate-trajectory), [copy](#copy-trajectory)  
**sub-params**: [constant](#constant-translation_trajectory), [bspline](#bspline-translation_trajectory), [file](#file-translation_trajectory) 

 **description**: translation component of the trajectory 
<hr style="border:1px solid gray"> 

### **translation_only** ([subsample_interpolate](#subsample_interpolate-trajectory))

 **description**: rotation value in the trajectory will be identity at all times 
<hr style="border:1px solid gray"> 

### **vec** ([align_axis](#align_axis-rotation))
**sub-params**: [negate](#negate-vec), [current_trajectory](#current_trajectory-vec) 

 **description**: vector(s) to align with the specified axis 
<hr style="border:1px solid gray"> 

### **width** ([camera](#camera-sensor))

 **description**: width in pixels of the camera images to generate 
<hr style="border:1px solid gray"> 

[[back to top]](#parameter-reference) 
