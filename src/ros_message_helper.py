#!/usr/bin/env python
from std_msgs.msg import Float32
from builtin_interfaces.msg import Time
from std_msgs.msg           import *
from nav_msgs.msg           import *
from sensor_msgs.msg        import *
from geometry_msgs.msg      import *
from visualization_msgs.msg import *
import numpy as np
from scipy.spatial.transform import Rotation
import motion_utils as utils


"""
Convenience functions to help fill ROS2 messages fields
"""

def as_color_msg(color, scale=1.0):
    # print('rmh: color: {}'.format(color))
    c = ColorRGBA()
    color = np.array(color).astype(float)/scale
    if len(color) == 3:
        color = np.append(color,1.0)
    c.r, c.g, c.b, c.a = color
    return c

def as_header(stamp,frame_id):
    h = Header()
    h.stamp = as_stamp(stamp)
    h.frame_id = frame_id
    return h


def as_int16multiarray_msg(arr):
    i = Int16MultiArray()
    i.data = arr
    return i

def as_marker_msg(stamp=0.0,frame_id='',pos=None,*,marker_id=0, rot=None, marker_type=2,
                   scale=3*[0.25], color=[1.,0.,0.,0.9], points=None):
    m = Marker()
    m.header.stamp = as_stamp(stamp)
    m.id = marker_id
    m.header.frame_id = frame_id
    m.scale.x, m.scale.y, m.scale.z = scale
    m.color.r, m.color.g, m.color.b, m.color.a = color
    m.type = marker_type
    if pos is not None:
        m.pose.position = as_point_msg(pos)
    if rot is None:
        m.pose.orientation.w = 1.0
    else:
        m.pose.orientation = as_quaternion_msg(rot)
    
    if points is not None:
        if marker_type is not None:
            m.type = marker_type
        else:
            m.type = Marker.LINE_STRIP
            m.points = [as_point_msg(p) for p in points]
    return m

def as_markerarray_msg(stamp=0.0,frame_id='',pos=None, *,n=None, marker_type=2, scale=3*[0.25]):
    ma = MarkerArray()
    if pos is not None:
        for i in range(len(pos)):
            m = as_marker_msg(stamp,frame_id,pos[i],marker_id=i, marker_type=marker_type)
            ma.markers.append(m)
    elif n is not None:
        for i in range(n):
            m = as_marker_msg(stamp,frame_id,marker_id=i, marker_type=marker_type, scale=scale)
            ma.markers.append(m)
    return ma

def as_path_msg(stamps=None,frame_id='',pos=None,rot=None):
    p = Path()
    if stamps is not None:
        p.header.stamp = as_stamp(stamps[0])
    p.header.frame_id = frame_id
    if rot is None:
        # rot = np.tile([0.,0.,0.,1.],(len(pos),1))
        rot = [0.,0.,0.,1.]
    if pos is not None:
        p.poses = [as_posestamped_msg(stamps[i],frame_id,pos[i],rot) for i in range(len(pos))]
    return p

def as_point_msg(pos):
    p = Point()
    p.x, p.y, p.z = np.array(pos).astype(float)
    return p

def as_point32_array(arr):
    n = len(arr)
    arr.astype(np.float32)
    p = Point32()
    p_array = []
    for row in range(n):
        p.x,p.y,p.z = arr[row,:] 
        p_array.append(p)
    return p_array
        
def as_pointcloud_msg(stamp, frame_id, points):
    p = PointCloud()
    p.header.stamp=stamp
    p.header.frame_id=frame_id
    # p.points=as_point32_array(points)
    p.points=points
    return p

def as_posestamped_msg(stamp=0.0,frame_id='',pos=None,*,rot=None):
    p = PoseStamped()
    p.header.stamp = as_stamp(stamp)
    p.header.frame_id = frame_id
    p.pose.position = as_point_msg(pos)
    if rot is None:
        p.pose.orientation.w = 1.0
    else:
        p.pose.orientation = as_quaternion_msg(rot)
    return p

def as_quaternion_msg(rot):
    _q = rot if rot.shape==(4,) else utils.as_scipy_rotation(rot).as_quat() 
    # _q = utils.as_scipy_rotation(rot).as_quat() #probably no performance difference
    q = Quaternion()
    q.x, q.y, q.z, q.w = _q
    return q

def as_stamp(t):
    s = Time()
    s.sec = int(t)
    s.nanosec = int((t-int(t))*1e9)
    return s

def as_transformstamped_msg(stamp,frame_id,child_frame_id,pos,rot):
    t = TransformStamped()
    t.header.stamp = as_stamp(stamp)
    t.header.frame_id = frame_id
    t.child_frame_id = child_frame_id
    t.transform.translation = as_vector3_msg(pos)
    t.transform.rotation = as_quaternion_msg(rot)
    return t

def as_vector3_msg(vec):
    v = Vector3()
    v.x, v.y, v.z = vec.astype(float)
    return v


def as_ndarray(msg):
    """
    convert a ROS message to a numpy array
    """
    pass


def paths_from_trajectory(traj):
    """
    convert a Trajectory object to 3 Path messages: pos, vel, acc
    only the position message contains rotations
    """
    t = traj.t
    frame_id = traj.global_frame
    pos = traj.translation.pos
    vel = traj.translation.vel
    acc = traj.translation.acc
    q = traj.rotation.q

    path_pos = as_path_msg(t,frame_id,pos,q)
    path_vel = as_path_msg(t,frame_id,vel)
    path_acc = as_path_msg(t,frame_id,acc)

    return path_pos, path_vel, path_acc



