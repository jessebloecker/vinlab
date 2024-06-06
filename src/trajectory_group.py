
from trajectory import Trajectory
from config_utils import check_keys, ConfigurationError

class TrajectoryGroup():
    """
    group of related Trajectory objects - e.g. the ground truth and the estimate
    """
    def __init__(self, trajectories, reference, main):
        self.reference = reference
        self.main = main
        self.trajectories = trajectories
        self.n = len(trajectories)
        self.error = {}
        #time match every trajectory to the reference trajectory
        # self.get_error()


    @classmethod
    def config(cls, config):
        config = check_keys(config, 'trajectory_group', context='scene')[0]
        reference = config['reference']
        trajectories = {}
        ct = config['trajectories']
        sub_trajectory_configs = [] #list of 'sub-trajectory' (trajectory dependent on another trajectory) configs
        for c in ct:
            t = Trajectory.config(c)
            if t == c: #the trajectory config was returned because it is a sub-trajectory (requires another trajectory to be generated first)
                sub_trajectory_configs.append(c)
            else:
                trajectories[t.id] = t

        for s in sub_trajectory_configs: #generate each sub-trajectory from it's corresponding (super) trajectory
            _id = s['id'] 
            if 'subsample_interpolate' in s.keys():        
                sub_traj_config = s['subsample_interpolate']
                super_id = sub_traj_config.pop('super_id')
                try:
                    super_traj = trajectories[super_id]
                except KeyError:
                    raise ConfigurationError('super_id: \'{}\' does not match any trajectory id in the trajectory group'.format(super_id))

                sub_traj = super_traj.subsample_interpolate(**sub_traj_config)
                super_traj.sub_ids.append(_id)
                sub_traj.super_id = super_id 

                trajectories[_id] = sub_traj #finally add the sub-trajectory to the trajectory group
        
        if not reference in trajectories.keys():
            raise ConfigurationError('reference trajectory id: \'{}\' does not match any trajectory id in the trajectory group'.format(reference))

        if 'main' in config.keys():
            main = config['main']
            if not main in trajectories.keys():
                raise ConfigurationError('main trajectory id: \'{}\' does not match any trajectory id in the trajectory group'.format(main))
        else: #automatically set the main trajectory
            if len(trajectories) > 2:
                raise ConfigurationError('must specify a \'main\' trajectory in addition to \'reference\' since there are more than 2 trajectories in the trajectory group')
            elif len(trajectories) == 1:
                main = reference #no trajectories are being compared
            else: #theres only one other trajectory
                main = next(iter([k for k in trajectories.keys() if k != reference]))

        return cls(trajectories,reference, main)