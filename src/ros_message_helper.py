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


"""
Convenience functions to help fill ROS2 messages fields

For all rotations, if given: 4x1 vector, assume quaternion [w,x,y,z]
                             3x3 matrix, assume rotation matrix
                             3x1 vector, assume roll-pitch-yaw = (x,y,z) 
                                                with convention x --> y --> z in fixed frame
                                                 (equivalent to z --> y --> x in body frame)
                             1x1 scalar, assume yaw in radians
"""


def as_header(stamp,frame_id='global'):
    h = Header()
    h.stamp = stamp
    h.frame_id = frame_id
    return h

def as_marker_msg(stamp,frame_id,pos,rot):
    m = Marker()
    m.header.stamp= stamp
    m.id = 0
    m.header.frame_id= frame_id
    m.scale.x =0.25
    m.scale.y =0.25
    m.scale.z =0.25
    m.type = m.SPHERE
    m.pose.position = as_point_msg(pos)
    m.color.r, m.color.g, m.color.b = np.array([1.0,1.0,0.0])
    m.color.a=0.9
    m.pose.orientation = as_quaternion_msg(np.array(rot))
    return m

def as_path_msg(stamp,frame_id,poses):
    p = Path()
    p.header.stamp=stamp
    p.header.frame_id = frame_id
    p.poses=poses
    return p

def as_point_msg(pos):
    p = Point()
    p.x = pos[0]
    p.y = pos[1]
    p.z = pos[2]
    return p
def as_point32_array(arr):
    """
    arr: nx3 array of points
    """
    n = len(arr)
    arr.astype(np.float32)
    p = Point32()
    p_array =[]
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

def as_posestamped_msg(stamp,frame_id,pos,rpy):
    p = PoseStamped()
    p.header.stamp = stamp
    p.header.frame_id = frame_id
    p.pose.position.x = pos[0]
    p.pose.position.y = pos[1]
    p.pose.position.z = pos[2]
    p.pose.orientation = as_quaternion_msg(rpy)
    return p

def as_quaternion_msg(rot):
    dim = rot.shape
    if dim == (4,) or dim == (4,1):
        _q = rot
    elif dim == (3,) or dim == (3,1):
        _q = Rotation.from_euler('xyz', rot).as_quat()
    elif dim == (3,3):

      _q = Rotation.from_matrix(rot).as_quat()
    else:
        raise ValueError('Invalid rotation input: {} (shape {})'.format(rot,dim))
    # _q format is x,y,z,w
    q = Quaternion()
    q.x = _q[0]
    q.y = _q[1]
    q.z = _q[2]
    q.w = _q[3]
    return q

def as_stamp(t):
    s = Time()
    s.sec = int(t)
    s.nanosec = int((t-int(t))*10**9)
    return s

def as_transformstamped_msg(stamp,frame_id,child_frame_id,pos,rpy):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = frame_id
    t.child_frame_id = child_frame_id
    t.transform.translation.x = pos[0]
    t.transform.translation.y = pos[1]
    t.transform.translation.z = pos[2]
    t.transform.rotation = as_quaternion_msg(rpy)
    return t


