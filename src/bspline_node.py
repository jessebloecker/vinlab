#!/usr/bin/env python 
import rclpy
from rclpy.node   import Node
import numpy as np
import tf2_ros
from std_msgs.msg import Float32
from builtin_interfaces.msg import Time
from  nav_msgs.msg          import Odometry, Path
from  geometry_msgs.msg     import PolygonStamped, Point32, PoseStamped
from std_srvs.srv           import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation
from bspline_controller import BSplineController
import ros_message_helper as rmh
import threading

"""
Publishes spline
should make this a cpp node if its gonna have to loop through points to turn them into pointstamped messages

poll the bspline controller at a constant rate, convert the arrays to path messages, and the control points to markers

TODO:
functions to publish tf frame
services to start and stop animation

"""
class BSplineNode(Node):
    def __init__(self):
        super().__init__('bspline_node')

        print('something')
        self.controller = BSplineController()
        self.controller_start()
        
    
        """
        Timer to loop through data and publish
        """
        display_rate = 10 # display 'framerate' at which to publish the visualization
        self.create_timer(1.0/display_rate, self.publish_paths)
        self.create_timer(1.0/display_rate, self.publish_markers)
        
        self.pub_pos = self.create_publisher(Path,'~/pos', 10)
        self.pub_vel = self.create_publisher(Path,'~/vel', 10)
        self.pub_acc = self.create_publisher(Path,'~/acc', 10)
        self.pub_markers = self.create_publisher(MarkerArray,'~/control_pts', 10)
        self.zero_rotation = np.array([0.0,0.0,0.0,1.0])

    def publish_markers(self):
        """
        Publish markers for each control point
        """
        markers = MarkerArray()
        for i in range(len(self.controller.control_pts)):
            m = rmh.as_marker_msg(rmh.as_stamp(0.0),'global',self.controller.control_pts[i,:],self.zero_rotation)
            if i==self.controller.selection:
                # m.color = rmh.as_color('green')
                m.color.a = 1.0
                m.color.r = 0.0
                m.color.g = 1.0
                m.color.b = 1.0
            m.id = i
            markers.markers.append(m)
        self.pub_markers.publish(markers)
        

    def publish_paths(self):

        pos = self.controller.pos_bspline
        vel = self.controller.vel_bspline
        acc = self.controller.acc_bspline
        if pos is not None:
            pos_path = self.as_path_msg(rmh.as_stamp(0.0),'global',self.controller.pos_bspline) #TODO: cython
            vel_path = self.as_path_msg(rmh.as_stamp(0.0),'global',self.controller.vel_bspline)
            acc_path = self.as_path_msg(rmh.as_stamp(0.0),'global',self.controller.acc_bspline)

            self.pub_pos.publish(pos_path)
            self.pub_vel.publish(vel_path)
            self.pub_acc.publish(acc_path)
        else:
            print('pos is none')
        
    def as_path_msg(self,stamp,frame_id,poses):
        p = Path()
        p.header.stamp=stamp
        p.header.frame_id = frame_id
        # p.poses=[as_posestamped_msg(as_stamp(0.0),'global',poses[i,:],self.zero_rotation) for i in range(len(poses))]
        p.poses=[rmh.as_posestamped_msg(rmh.as_stamp(0.0),'global',poses[i,:],self.zero_rotation) for i in range(len(poses))]
        return p
    
    def controller_start(self):
        thread = threading.Thread(target=self.controller.keyboard_start(), daemon=True)
        thread.start()
        # thread.join()
   

if __name__ == '__main__':
    rclpy.init(args=None)
    b = BSplineNode()
    rclpy.spin(b)
    
