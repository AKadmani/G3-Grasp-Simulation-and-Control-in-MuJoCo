import numpy as np
from enum import Enum

class GraspType(Enum):
    """Grasp types based on the referenced paper"""
    POWER_GRASP = 1
    PRECISION_GRASP = 2
    LATERAL_GRASP = 3
    SPHERICAL_GRASP = 4
    CYLINDRICAL_GRASP = 5

class GraspPlanner:
    """Plans grasp configurations and trajectories"""
    
    def __init__(self, model, data):
        self.model = model
        self.data = data
        
        # Allegro hand joint limits (approximate)
        self.joint_limits = {
            'lower': np.array([-0.47, -0.196, -0.174, -0.227] * 3 + [0.263, -0.105, -0.189, -0.162]),
            'upper': np.array([0.47, 1.61, 1.709, 1.618] * 3 + [1.396, 1.163, 1.644, 1.719])
        }
        
        # Pre-grasp configurations for different grasp types
        self.pre_grasp_configs = {
            GraspType.POWER_GRASP: self._power_grasp_config(),
            GraspType.PRECISION_GRASP: self._precision_grasp_config(),
            GraspType.LATERAL_GRASP: self._lateral_grasp_config(),
            GraspType.SPHERICAL_GRASP: self._spherical_grasp_config(),
            GraspType.CYLINDRICAL_GRASP: self._cylindrical_grasp_config()
        }
        
    def _power_grasp_config(self):
        """Power grasp: all fingers wrap around object"""
        # Open hand configuration
        config = np.zeros(16)
        # Slightly bend all fingers
        for i in range(4):  # 4 fingers
            config[i*4:(i+1)*4] = [0.1, 0.2, 0.2, 0.1]
        return config
        
    def _precision_grasp_config(self):
        """Precision grasp: thumb and index finger"""
        config = np.zeros(16)
        # Index finger
        config[0:4] = [0.2, 0.4, 0.3, 0.2]
        # Thumb
        config[12:16] = [0.8, 0.3, 0.3, 0.2]
        # Other fingers slightly bent back
        config[4:8] = [-0.1, 0.1, 0.1, 0.1]
        config[8:12] = [-0.1, 0.1, 0.1, 0.1]
        return config
        
    def _lateral_grasp_config(self):
        """Lateral grasp: thumb against side of index"""
        config = np.zeros(16)
        # Index finger straight
        config[0:4] = [0.3, 0.1, 0.1, 0.1]
        # Thumb to the side
        config[12:16] = [1.0, 0.5, 0.2, 0.1]
        # Other fingers curled
        config[4:8] = [0.1, 0.8, 0.8, 0.5]
        config[8:12] = [0.1, 0.8, 0.8, 0.5]
        return config
        
    def _spherical_grasp_config(self):
        """Spherical grasp: all fingers evenly distributed"""
        config = np.zeros(16)
        # All fingers moderately bent
        for i in range(4):
            config[i*4:(i+1)*4] = [0.3, 0.5, 0.4, 0.3]
        return config
        
    def _cylindrical_grasp_config(self):
        """Cylindrical grasp: fingers wrap, thumb opposes"""
        config = np.zeros(16)
        # Three fingers wrap
        for i in range(3):
            config[i*4:(i+1)*4] = [0.2, 0.6, 0.5, 0.4]
        # Thumb opposes
        config[12:16] = [0.6, 0.4, 0.3, 0.2]
        return config
        
    def plan_grasp(self, object_type, grasp_type):
        """
        Plan a grasp for given object and grasp type
        
        Args:
            object_type: 'cylinder', 'box', or 'sphere'
            grasp_type: GraspType enum
            
        Returns:
            dict: Grasp plan with trajectories
        """
        # Get pre-grasp configuration
        pre_grasp = self.pre_grasp_configs[grasp_type]
        
        # Get object position
        if object_type == 'cylinder':
            obj_name = 'cylinder_object'
        elif object_type == 'box':
            obj_name = 'box_object'
        elif object_type == 'sphere':
            obj_name = 'sphere_object'
        else:
            raise ValueError(f"Unknown object type: {object_type}")
            
        obj_id = self.model.body(obj_name).id
        obj_pos = self.data.xpos[obj_id].copy()
        
        # Adjust grasp based on object
        grasp_config = self._adjust_grasp_for_object(pre_grasp, object_type, grasp_type)
        
        # Generate approach trajectory
        trajectory = self._generate_trajectory(pre_grasp, grasp_config)
        
        return {
            'object_type': object_type,
            'grasp_type': grasp_type,
            'pre_grasp': pre_grasp,
            'grasp': grasp_config,
            'trajectory': trajectory,
            'object_position': obj_pos
        }
        
    def _adjust_grasp_for_object(self, pre_grasp, object_type, grasp_type):
        """Adjust grasp configuration based on object geometry"""
        grasp = pre_grasp.copy()
        
        if object_type == 'cylinder' and grasp_type == GraspType.CYLINDRICAL_GRASP:
            # Increase finger curvature for cylinder
            for i in range(3):
                grasp[i*4+1:i*4+4] *= 1.2
        elif object_type == 'sphere' and grasp_type == GraspType.SPHERICAL_GRASP:
            # Uniform curvature for sphere
            for i in range(4):
                grasp[i*4+1:i*4+3] *= 1.1
        elif object_type == 'box' and grasp_type == GraspType.PRECISION_GRASP:
            # Less curvature for flat surfaces
            grasp[1:3] *= 0.8  # Index finger
            grasp[13:15] *= 0.8  # Thumb
            
        # Ensure within joint limits
        grasp = np.clip(grasp, self.joint_limits['lower'], self.joint_limits['upper'])
        
        return grasp
        
    def _generate_trajectory(self, start_config, end_config, duration=2.0, dt=0.01):
        """Generate smooth trajectory between configurations"""
        n_steps = int(duration / dt)
        trajectory = []
        
        for i in range(n_steps):
            t = i / (n_steps - 1)
            # Use smooth interpolation (minimum jerk trajectory)
            s = t**2 * (3 - 2*t)  # Smooth step function
            config = start_config + s * (end_config - start_config)
            trajectory.append(config)
            
        return np.array(trajectory)
        
    def compute_approach_direction(self, object_type, grasp_type):
        """Compute approach direction for hand based on grasp type"""
        if grasp_type == GraspType.POWER_GRASP:
            # Approach from side
            return np.array([1, 0, 0])
        elif grasp_type == GraspType.PRECISION_GRASP:
            # Approach from above
            return np.array([0, 0, -1])
        elif grasp_type == GraspType.LATERAL_GRASP:
            # Approach at angle
            return np.array([0.7, 0, -0.7])
        elif grasp_type == GraspType.SPHERICAL_GRASP:
            # Approach from above-side
            return np.array([0.5, 0, -0.866])
        else:  # CYLINDRICAL_GRASP
            # Approach perpendicular to cylinder axis
            return np.array([1, 0, 0])
            
    def evaluate_grasp_feasibility(self, grasp_plan):
        """Evaluate if a grasp is feasible"""
        grasp_config = grasp_plan['grasp']
        
        # Check joint limits
        within_limits = np.all(grasp_config >= self.joint_limits['lower']) and \
                       np.all(grasp_config <= self.joint_limits['upper'])
        
        # Check for self-collision (simplified)
        # In practice, this would use MuJoCo's collision detection
        no_self_collision = True  # Placeholder
        
        # Check reachability (simplified)
        # Would check if object is within workspace
        reachable = True  # Placeholder
        
        return {
            'feasible': within_limits and no_self_collision and reachable,
            'within_limits': within_limits,
            'no_self_collision': no_self_collision,
            'reachable': reachable
        }