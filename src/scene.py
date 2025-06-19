#!/usr/bin/env python
import numpy as np
import yaml
from trajectory_group import TrajectoryGroup
from sensor_platform import SensorPlatform
from feature import Feature, PointSet
from static_frame import StaticFrame
from config_utils import check_keys, set_output, ConfigurationError
from sensor_measurements import CameraMeasurements, IMUMeasurements
from geometry_utils import get_point_positions
from modify_scene_config import modify_scene_config
from array_utils import ValueMatcher
from geometry_utils import align_trajectories

np.set_printoptions(precision=4, suppress=True)

class Scene():
    def __init__(self, config_file, trajectory_group, platform=None, features=None, static_frames=None, output_config=''):
        """
        A scene contains a trajectory group, sensor platform, and/or a set of features.
        Feature positions in each camera fame at all times and their corresponding measurements are
        computed and stored as attributes of the scene.   
        """
        self.config_file = config_file
        self.trajectory_group = trajectory_group
        self.platform = platform
        self.features = features
        self.static_frames = static_frames        
        self.output_config = output_config #only pertains to sensors, maybe should not be here

        #combine all features
        if features is not None:
            all_points = np.vstack([f.points['global'] for f in features.values() if isinstance(f,PointSet)])
            all_colors = np.vstack([f.colors for f in features.values() if isinstance(f,PointSet)])
            self.features['all'] = PointSet(all_points, colors=all_colors, feature_id='all', feature_type='points')

            #for each camera, get all features expressed in the camera frame at all times
            for k,v in platform.sensors.items():
                if v.type == 'camera':
                    pos = trajectory_group.trajectories[trajectory_group.reference].translation.pos.values
                    rot = trajectory_group.trajectories[trajectory_group.reference].rotation.rot
                    points = get_point_positions(pos,rot, self.features['all'].points['global'])
                    self.features['all'].points[k] = points #don't need to store this 
                
            #compute measurements for all sensors that have measurements enabled
            measurements = {}  
            sensor_ids = [i for i in platform.sensors.keys()]
            #todo: static_sensor_ids = [i for i in static_frames.sensors.keys()]
            ref = trajectory_group.reference
            traj = trajectory_group.trajectories[ref]
            features = self.features['all']
            for i in sensor_ids:
                sensor = platform.sensors[i]
                if sensor.type=='camera' and sensor.enable_measurements:
                    m = CameraMeasurements(traj, sensor, features)
                    measurements[i] = m
                elif sensor.type == 'imu' and sensor.enable_measurements:
                    m = IMUMeasurements(traj, sensor)
                    measurements[i] = m
            self.measurements = measurements
            
    @classmethod
    def config(cls, config_file):
        """
        Initialize a Scene instance from a config file
        """
        config = check_keys(yaml.load(open(config_file),Loader=yaml.FullLoader),'scene', context=None)[0]
        # config = modify_scene_config(config)
        trajectory_group = TrajectoryGroup.config(config['trajectory_group'])
        platform = SensorPlatform.config(config['platform'])
        features = None
        static_frames = None
        output_config = ''

        if 'features' in config.keys():
            cf = config['features']
            features = {}
            for i in range(len(cf)):
                f = Feature.config(cf[i])
                features[f.id] = f
        if 'output' in config.keys():
            output_config = set_output(config['output'],config_file)

        if 'static_frames' in config.keys():
            cs = config['static_frames']
            #compute the best fit tf then add the result to the scene
            if 'align_trajectories' in config.keys():
                align_trajectories = config['align_trajectories']
                trajectory_group.align_trajectories(align_trajectories)

            static_frames = {}
            for i in range(len(cs)):
                # print('APPLYING STATIC FRAME {}'.format(cs[i]['id']))
                s = StaticFrame.config(cs[i])
                static_frames[s.id] = s

        for k,v in trajectory_group.trajectories.items():
            if v.frame != 'global':
                if 'static_frames' in config.keys():
                    static_frames={}
                    cs_list = config['static_frames']
                    # print('cs_list',cs_list)
                    static_frame_ids = [c['id'] for c in cs_list]
                    if v.frame in static_frame_ids:
                        index = static_frame_ids.index(v.frame)
                        cs, cs_mode = check_keys(cs_list[index], 'static_frame', context='scene')
                        # print('*cs {} *cs_mode: {}'.format(cs, cs_mode))
                        if cs_mode == {'transform'}:
                            s = StaticFrame.config(cs)
                            static_frames[s.id] = s 
                            R = s.rot.as_matrix()
                            p = s.pos
                            # print('Applying static frame \'{}\' to trajectory \'{}\''.format((R,p),k))
                            trajectory_group.trajectories[k] = v.transform(R,p)
                        elif cs_mode == {'align_trajectory'}:
                            centering_mode = cs['align_trajectory']['centering_mode'] 
                            rotation_mode = cs['align_trajectory']['rotation_mode']
                            selection = cs['align_trajectory']['selection']
                            # print('Trajectory: \'{}\' frame set to \'{}\' with centering mode \'{}\' and rotation mode \'{}\' and selection \'{}\''.format(k,v.frame,centering_mode,rotation_mode,selection))
                        
                            traj_ref = trajectory_group.trajectories[trajectory_group.reference]
                            traj = v

                            rot_alignment, pos_alignment = align_trajectories(traj_ref,traj,centering_mode, rotation_mode, selection)
                            
                            rot_align_euler = rot_alignment.as_euler('xyz',degrees=True)
                            # print('rotation: [{},{},{}]'.format(rot_align_euler[0], rot_align_euler[1], rot_align_euler[2]))
                            # print('translation: [{},{},{}]'.format(pos_alignment[0], pos_alignment[1], pos_alignment[2]))
                            s = StaticFrame(frame_id=v.frame, rot=rot_alignment, pos=pos_alignment)   
                            static_frames[s.id] = s
                            R = s.rot.as_matrix()
                            p = s.pos
                            trajectory_group.trajectories[k] = v.transform(R,p)

                    else:
                        print ('Static frame \'{}\' not found in static frames'.format(v.frame))
                        raise ConfigurationError('Trajectory \'{}\' frame set to \'{}\', but no such static frames is defined. Defined frames are: \'{}\''.format(
                            k,v.frame, ', '.join([i for i in cs.keys()])))
                else:
                    raise ConfigurationError('Trajectory \'{}\' frame set to \'{}\', but static frames are defined'.format(k,v.frame))


        return cls(config_file, trajectory_group, platform, features, static_frames, output_config)
    
    def radtan_distort(self, points, coefficients):
        """
        apply radial and tangential distortion to a set of points
        
        params:
            points: n x 2 array of uv points
            coefficients: 4 element array of distortion coefficients
        returns: n x 2 array of distorted points
        """
        k1,k2,p1,p2 = coefficients
        #TODO apply the distortion
        return points
        
    def print_scene_info(self,output_file=None):
        """
        print a description of the scene to the console or to a file
        """
        PURPLE = "\033[0;35m"
        BOLDYELLOW = "\033[1;33m"
        YELLOW = "\033[0;33m"
        BOLD = "\033[1m"
        END = "\033[0m"
        indent='    '
        config_file = self.config_file
        print(BOLDYELLOW+'============================ SCENE INFO ============================'+END)
        print(BOLDYELLOW+'Base Config: '+END+'{}'.format(self.config_file)+END)

        tg = self.trajectory_group
        ids = ', '.join([i for i in tg.trajectories.keys()])
        print(('{}'+BOLDYELLOW+'Trajectories: '+END+'\'{}\'').format(indent,ids))
        for t in tg.trajectories.values():
            if t.id == tg.reference:
                _id = '[REF] '+t.id
            elif t.id == tg.main:
                _id = '[MAIN] '+t.id
            else:
                _id = t.id
            print(('{}'+YELLOW+'{}'+END).format(indent*2,_id))
            print(('{}'+PURPLE+'n'+END+': {} '+PURPLE+'dur'+END+': {:0.2f} '+PURPLE+'rate'+END+': {:0.2f} '+PURPLE+'offset'+END+': {:0.3f}')
            .format(indent*2,t.n,t.dur,t.rate,t.time_offset))
            print(('{}'+PURPLE+'length'+END+': {:0.3f} '+PURPLE+'vel min'+END+': {:0.3f} '+PURPLE+'vel avg.'+END+': {:0.3f} '+PURPLE+'vel max'+END+': {:0.3f}')
            .format(indent*2,t.translation.pos.length,t.translation.vel.minval,t.translation.vel.mean,t.translation.vel.maxval))
        p = self.platform
        cameras = [v for k,v in p.sensors.items() if v.type == 'camera']
        imus = [v for k,v in p.sensors.items() if v.type == 'imu']
        body_frames = [v for k,v in p.body_frames.items()]
        print(('{}'+BOLDYELLOW+'Platform: '+END+'\'{}\' (cameras: {}  imus: {}  body_frames: {})')
              .format(indent,p.id,len(cameras),len(imus),len(body_frames)))
        for c in cameras:
            _id = c.id+' [BASE FRAME]' if c.id==p.base_frame else c.id
            print(('{}'+YELLOW+'{}'+END+' (camera) ')
                .format(indent*2,_id))
            print(('{}'+PURPLE+'rate'+END+': {:0.2f}  '+'images'+END+': {:d}  '+PURPLE+'height'+END+': {}  '+PURPLE+'width'+END+': {}')
                  .format(indent*2,c.rate,self.measurements[c.id].values.shape[1],c.height,c.width))
            print(('{}'+PURPLE+'pos'+END+': [{:8.3f} {:8.3f} {:8.3f} ]').format(indent*2,*c.pos))
            print(('{}'+PURPLE+'rpy'+END+': [{:8.3f} {:8.3f} {:8.3f} ](deg)').format(indent*2,*c.rot.as_euler('xyz',degrees=True)))
            print(('{}'+PURPLE+'calibration'+END+': [{:5.0f} {:5.0f} {:5.0f}]').format(indent*2,*c.K[0]))
            print(('{}     [{:5.0f} {:5.0f} {:5.0f}]').format(indent*4,*c.K[1]))
            print(('{}     [{:5.0f} {:5.0f} {:5.0f}]').format(indent*4,*c.K[2]))
            # print(('{}'+PURPLE+'distortion'+END+':[{:8.3f} {:8.3f} {:8.3f} {:8.3f}] (rad-tan)\n').format(indent*2,*c.distortion))
        for i in imus:
            _id = i.id+' [BASE FRAME]' if i.id==p.base_frame else i.id
            print(('{}'+YELLOW+'{}'+END+' (imu) ')
                .format(indent*2,_id))
            print(('{}'+PURPLE+'rate'+END+': {:0.2f}').format(indent*2,i.rate))
            print(('{}'+PURPLE+'pos'+END+': [{:8.3f} {:8.3f} {:8.3f} ]').format(indent*2,*i.pos))
            print(('{}'+PURPLE+'rpy'+END+': [{:8.3f} {:8.3f} {:8.3f} ](deg)').format(indent*2,*i.rot.as_euler('xyz',degrees=True)))
            print(('{}'+PURPLE+'gyro_noise_density'+END+': {:8.5f}  '+PURPLE+'gyro_random_walk'+END+': {:8.5f}').format(indent*2, i.gyro_noise_density,i.gyro_random_walk))
            print(('{}'+PURPLE+'accel_noise_density'+END+': {:8.5f}  '+PURPLE+'accel_random_walk'+END+': {:8.5f}\n').format(indent*2, i.accel_noise_density,i.accel_random_walk))
        for b in body_frames:
            _id = b.id+' [BASE FRAME]' if b.id==p.base_frame else b.id
            print(('{}'+YELLOW+'{}'+END+' (body frame) ')
                .format(indent*2,_id))
            print(('{}'+PURPLE+'pos'+END+': [{:8.3f} {:8.3f} {:8.3f} ]').format(indent*2,*b.pos))
            print(('{}'+PURPLE+'rpy'+END+': [{:8.3f} {:8.3f} {:8.3f} ](deg)\n').format(indent*2,*b.rot.as_euler('xyz',degrees=True)))


        f = self.features
        if f is not None:
            print(('{}'+BOLDYELLOW+'Features: '+END+'{} total points').format(indent,len(f['all'].points['global'])))
            for k,v in f.items():
                if isinstance(v,PointSet) and not k == 'all':
                    print(('{}'+YELLOW+'{:<10s}'+END+': {:<18s} '+PURPLE+'n'+END+': {}').format(indent*2,k, v.type,v.n))

        s = self.static_frames
        if s is not None:
            print(('{}'+BOLDYELLOW+'Static frames: '+END).format(indent))
            for k,v in s.items():
                print(('{}'+YELLOW+'{:<10s}'+END+': {:<18s} '+PURPLE+'pos'+END+': [{:8.3f} {:8.3f} {:8.3f}] '+PURPLE+'rpy'+END+': [{:8.3f} {:8.3f} {:8.3f}] (deg)')
                      .format(indent*2,k,v.id,*v.pos,*v.rot.as_euler('xyz',degrees=True)))

            print(BOLDYELLOW+'===================================================================='+END)
            