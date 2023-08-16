#!/usr/bin/env python 


import numpy as np
from time import sleep
from pynput import keyboard
import time
from bspline_core import BSplineCore


class BSplineController():
    def __init__(self):
        """
        Adjust the position of any control points in the bspline and other params via key presses or UI buttons
        other features:
        change the scale of the whole curve - expand about centroid or selected point
        min max velocity and acceleration, centroid, path length, path duration
        """
        self.control_pts = np.array([[0,0,0],  #idk where this will be set by the user yet
                 
                                [3,0,0],
                                [16,7,0],
                                [12,14,0],
                                [7,1,0],
                                [0,20,0],
                                [-10,17,0],
                                [-3,8,0],
                                [-2,0,0]]).astype(np.float64)
        self.control_pts_orig = self.control_pts.copy()
        self.selection = 0 #index of the currently selected control point

        self.directions = {'d':np.array([1.,0.,0.]),  # +x    directions to move corresponding to key presses
                           'a':np.array([-1.,0.,0.]), # -x
                           'w':np.array([0.,1.,0.]),  # +y
                           's':np.array([0.,-1.,0.]), # -y
                           'q':np.array([0.,0.,1.]),  # +z
                           'e':np.array([0.,0.,-1.])} # -z
        
        #spline params. UI can change at any time
        r=100
        self.resolution = r #number of points in the bspline 
        self.order = 3 #order of the bspline 
        self.t = np.linspace(0,1,num=r,endpoint=False) #default


        self.increment=0.1 #amount to move selected control point by
        self.shift_active = False #flag for shift key
        self.min_update_time = 0.5 #minimum time between recomputing the bspline
        self.last_update = time.time() #time of last update

        #store all of the splines, recompute on key press
        self.pos_bspline = None 
        self.vel_bspline = None
        self.acc_bspline = None
        self.jerk_bspline = None     
        self.snap_bspline = None   

        r = self.resolution
        t = self.t
        order = self.order
        control_pts = self.control_pts


        core = BSplineCore(t,order) #core class for computing the bspline
        self.core=core
        #pos, vel, acc, jerk, snap, 5th deriv, 6th deriv. Some will have value of None (unless order is the max) 
        # p,v,a,j,s,d5,d6 = self.compute_bsplines(core,control_pts) 
        p,v,a = self.compute_bsplines(core,control_pts) #for now, just compute the first 3 derivatives
        self.pos_bspline = p
        self.vel_bspline = v
        self.acc_bspline = a

    

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
        self.recompute() #recompute the spline if enough time has passed since the last updates
        self.print_status()

    def print_status(self):
        """
        print the current position of all control points and show which is selected
        """
        # control_pts_str = ''
        # for i in range(len(self.control_pts)):
        #     if i == self.selection:
        #         control_pts_str += '\033[1m' #make boldface
        #     control_pts_str += '{}: {:0.2f}, {:0.2f}, {:0.2f} \033[0m \n'.format(i,*self.control_pts[i])
        print('\n'+ 
                # 'shift: {}'.format(('OFF','ON')[self.shift_active])+'\n'+
                'selected control point: {}'.format(self.selection))
                # 'control points: \n'+
                # control_pts_str+'\r')
        

    def compute_bsplines(self,core,control_pts):
        """
        use the t and order that have already been established
        return uniform bspline and its corresponding derivatives
        this should be in the controller, call it every time something changes
        t and order are not gonna change as often
        """
        k = core.order
        M = core.get_blending_matrix(k) 
        t = core.t
        n = len(t)
        l = len(control_pts) 
        N = n*(l-k) #total number of points in (each) spline
        print('n: {}  l: {}  k: {}  N: {}'.format(n,l,k,N))
        

        #could just loop to the end to handle any number of derivatives. prob gonna have to do this
        all_basis = core.basis_vectors
        pos_basis = core.basis_vectors[0,:,:] #nx4
        vel_basis = core.basis_vectors[1,:,:] #nx4
        acc_basis = core.basis_vectors[2,:,:] #nx4

        pos_bspline = np.zeros((N,3)) 
        vel_bspline = np.zeros((N,3))
        acc_bspline = np.zeros((N,3))
        
        for i in range(l-k): #could probably make this whole thing a matrix operation somehow (make giant diagonal matrix...i've done this before))
            #also, after the first time, you only have to update sections of the bspline each time, not recompute the whole thing
            pts = control_pts[i:i+k+1,:] #sliding window of (k+1) control points
            lower = i*n
            upper = (i+1)*n
            pos_bspline[lower:upper,:] = np.linalg.multi_dot((pos_basis,M,pts)) #(n x k)*(k x k)*(k x 3)=(n x 3), where k = k+1
            vel_bspline[lower:upper,:] = np.linalg.multi_dot((vel_basis,M,pts))
            acc_bspline[lower:upper,:] = np.linalg.multi_dot((acc_basis,M,pts))

       # ======================================================================================================================
        #better implemntation
        # only loop trough each derivative
        # pts_diag = every group of points in a diagonal matrix
        #do the whole thing the first time, then only change sections (lower and upper bounds are neighborhood of the changed point)
        # bspline[i,:,:] = np.linalg.multi_dot((basis[i,:,:],M,pts))
        # ======================================================================================================================


        # do evaluation stuff somewhere
        # vel_norms = np.linalg.norm(vel_bspline,axis=1)
        # max_vel = np.max(vel_norms)
        # acc_norms = np.linalg.norm(acc_bspline,axis=1)
        # max_acc = np.max(acc_norms)
        # print('max vel: {}'.format(max_vel))
        # print('max acc: {}'.format(max_acc))
        
        self.last_update = time.time()
        return pos_bspline, vel_bspline, acc_bspline
    

    def recompute(self):
        """
        recompute the bspline if enough time has passed since the last update
        """
        core = self.core
        control_pts = self.control_pts

        now = time.time()
        prev = self.last_update
        m = self.min_update_time
        if now - prev > m:
            print('\nrecomputing...')
            p,v,a = self.compute_bsplines(core,control_pts)
            self.pos_bspline = p
            self.vel_bspline = v
            self.acc_bspline = a

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




