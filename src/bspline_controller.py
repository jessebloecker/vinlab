#!/usr/bin/env python 

import numpy as np
from time import sleep
from pynput import keyboard
import time
from bspline_core import BSplineCore
import motion_utils as utils
import threading
import yaml


class BSplineController():
    """
    Control the position of any control points in the bspline and other params via key presses or UI buttons
    and compute the bspline and its derivatives

    Attributes:
        core (BSplineCore): core instance for computing the bspline
        timescale (float): time (seconds) that it takes to travel 1 span of the spline (use this change speed)
        increment (float): amount (meters) by which to move selected control point on key press
        control_pts (ndarray): n x 3 array of current control points
        selection (int): index of the currently selected control point TODO: handle repeated control points better
        directions (dict): directions to move corresponding to key presses
        shift_active (bool): flag for shift key
        min_update_time (float): minimum time required to recompute the bspline
        last_update (float): time of last update
        pos_bspline (ndarray): n x 3 array of position values
        vel_bspline (ndarray): n x 3 array of velocity values
        acc_bspline (ndarray): n x 3 array of acceleration values
    """
    def __init__(self):

        #Spline parameters - load from yaml
        config_path = '/home/jesse/ros2_ws/src/motion_tools/config/bspline_config.yaml'
        res = yaml.load(open(config_path),Loader=yaml.FullLoader)['res']
        order = yaml.load(open(config_path),Loader=yaml.FullLoader)['order']
        timescale = yaml.load(open(config_path),Loader=yaml.FullLoader)['timescale']
        increment = yaml.load(open(config_path),Loader=yaml.FullLoader)['increment']
        control_pts = yaml.load(open(config_path),Loader=yaml.FullLoader)['control_pts']
        control_pts = np.array(control_pts).astype(np.float64)

        control_pts_reduced, inds = np.unique(control_pts,axis=0, return_index=True) #use np.diff to get rid of repeated control points
        control_pts_reduced = control_pts_reduced[np.argsort(inds)]
        print('control_pts_reduced',control_pts_reduced)
        duration = timescale*(len(control_pts)-order) #total duration of the bspline
        
        self.duration_sec = duration
        self.timescale = timescale
        self.increment = increment
        self.control_pts = control_pts
        self.control_pts_orig = self.control_pts.copy()
        self.selection = 0  #index of the currently selected control point
        self.directions = {'d':np.array([1.,0.,0.]),  # +x    directions to move corresponding to key presses
                           'a':np.array([-1.,0.,0.]), # -x
                           'w':np.array([0.,1.,0.]),  # +y
                           's':np.array([0.,-1.,0.]), # -y
                           'q':np.array([0.,0.,1.]),  # +z
                           'e':np.array([0.,0.,-1.])} # -z
        self.shift_active = False #flag for shift key
        self.min_update_time = 0.5 #minimum time required to recompute the bspline
        self.update_waiting = False
        self.last_update = time.time() #time of last update

        core = BSplineCore(res,order) #core class for computing the bspline
        self.core=core
        p,v,a = self.compute_bsplines(core,control_pts) #for now, just compute the first 3 derivatives
        self.pos_bspline = p
        self.vel_bspline = v
        self.acc_bspline = a
        self.update()

    def update_control_pt(self,k):
        """
        update the  position of the selected control point
        params:
            k: key pressed
        """
        inc = self.increment
        s = self.selection
        cur_pos = self.control_pts[s]
        new_pos = cur_pos + self.directions[k]*inc
        self.control_pts[s]=new_pos

    def select(self):
        """
        select a control point
        """
        s = self.selection
        n = len(self.control_pts)
        if self.shift_active:
            self.selection = (s-1, n-1)[int(bool(s<1))] #wrap around to n-1 if s is 0
        else:
            self.selection = (s+1, 0)[int(bool(s>n-2))] #wrap around to 0 if s is n-1           
    
    def reset_control_pts(self):
        """
        reset control points to original values
        """
        if self.shift_active:
            self.control_pts = self.control_pts_orig.copy() #reset all control points
        else:
            self.control_pts[self.selection] = self.control_pts_orig[self.selection] #reset selected control point


    def on_press(self,key):
        """
        callback for key press
        params:
            key: key pressed
        """
        try:
            k=key.char.lower()
            if k in 'qweasd':
                # print('{} pressed'.format(k))
                self.update_control_pt(k)
            elif k in ('0',')'): #the ')' is <shift>+0
                self.reset_control_pts()
                
        except AttributeError:
            if key == keyboard.Key.space:
                self.select()
            if key == keyboard.Key.shift:
                self.shift_active = True
        # self.print_status()

    def on_release(self,key):
        """
        callback for key release
        params:
            key: key released
        """
        if key == keyboard.Key.shift:
            self.shift_active = False
        self.update() #recompute the spline if enough time has passed since the last updates
        self.print_status()

    def print_status(self):
        """
        print the current position of all control points and show which is selected
        """
        control_pts_str = ''
        for i in range(len(self.control_pts)):
            if i == self.selection:
                control_pts_str += '\033[1m' #make boldface
            control_pts_str += '{}: {:0.2f}, {:0.2f}, {:0.2f} \033[0m \n'.format(i,*self.control_pts[i])
        print('\n'+ 
                'shift: {}'.format(('OFF','ON')[self.shift_active])+'\n'+
                'selected control point: {}'.format(self.selection)+
                'control points: \n'+
                control_pts_str+'\r')
        

    def compute_bsplines(self,core,control_pts):
        """
        use the t and order that have already been established
        return uniform bspline and its corresponding derivatives
        this should be in the controller, call it every time something changes
        t and order are not gonna change as often
        """
        k = core.order
        M = core.get_blending_matrix(k) 
        n = core.res
        l = len(control_pts) 
        N = n*(l-k) #total number of points in (each) spline
        scale = self.timescale
        print('n: {}  l: {}  k: {}  N: {}'.format(n,l,k,N))
        

        #could just loop to the end to handle any number of derivatives. prob gonna have to do this
        all_basis = core.basis_vectors
        pos_basis = core.basis_vectors[0,:,:] #nxk
        vel_basis = core.basis_vectors[1,:,:] #nxk
        acc_basis = core.basis_vectors[2,:,:] #nxk

        pos_bspline = np.zeros((N,3)) 
        vel_bspline = np.zeros((N,3))
        acc_bspline = np.zeros((N,3))

        for i in range(l-k): #could probably make this whole thing a matrix operation somehow (make giant diagonal matrix...i've done this before))
            #also, after the first time, you only have to update sections of the bspline each time, not recompute the whole thing
            pts = control_pts[i:i+k+1,:] #sliding window of (k+1) control points
            lower = i*n
            upper = (i+1)*n
            pos_bspline[lower:upper,:] = np.linalg.multi_dot((pos_basis,M,pts)) #(n x k1)*(k1 x k1)*(k1 x 3)=(n x 3), where k1 = order+1
            vel_bspline[lower:upper,:] = np.linalg.multi_dot((vel_basis,M,pts))*(1.0/scale)
            acc_bspline[lower:upper,:] = np.linalg.multi_dot((acc_basis,M,pts))*(1.0/scale)**2

       # ======================================================================================================================
        #better implemntation
        # only loop trough each derivative
        # pts_diag = every group of points in a diagonal matrix
        #do the whole thing the first time, then only change sections (lower and upper bounds are neighborhood of the changed point)
        # bspline[i,:,:] = np.linalg.multi_dot((basis[i,:,:],M,pts))
        # ======================================================================================================================
        
        self.last_update = time.time()
        return pos_bspline, vel_bspline, acc_bspline
    

    def update(self):
        """
        recompute the bspline if enough time has passed since the last update
        """
        core = self.core
        control_pts = self.control_pts

        now = time.time()
        prev = self.last_update
        # m = self.min_update_time
        # if now - prev > m:

        p,v,a = self.compute_bsplines(core,control_pts)
        self.pos_bspline = p
        self.vel_bspline = v
        self.acc_bspline = a

        m = utils.centroid(p)
        M = m.reshape((1,3)).repeat(len(p),axis=0)
        self.rotations = utils.rotation_align_axis('z',v,grounded_axis='x',flip=False)

        # is slicing the whole array faster than re-assigning it??? test this
        #     self.pos_bspline[:,:]=p    ~vs~   self.pos_bspline=p

    def keyboard_start(self):
        """
        start the keyboard listener (UI should control this)
        """
        listener = keyboard.Listener(on_press=self.on_press,on_release=self.on_release)
        listener.start()

    def keyboard_stop(self):
        """
        stop the keyboard listener (UI should control this)
        """
        listener.stop()
        listener.join()


    def update_core():
        pass

        
if __name__ == '__main__':
    b = BSplineController()
    b.keyboard_start()
    




