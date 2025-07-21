import numpy as np
from abc import ABC, abstractmethod

class GraspController(ABC):
    """Abstract base class for grasp controllers"""
    
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.n_joints = 16  # Allegro hand has 16 joints
        
    @abstractmethod
    def compute_control(self, target_positions, target_forces=None):
        """Compute control signals"""
        pass
    
    def get_joint_positions(self):
        """Get current joint positions"""
        return self.data.qpos[:self.n_joints].copy()
    
    def get_joint_velocities(self):
        """Get current joint velocities"""
        return self.data.qvel[:self.n_joints].copy()
    
    def set_control(self, control_signals):
        """Apply control signals to actuators"""
        self.data.ctrl[:self.n_joints] = control_signals

class PIDController(GraspController):
    """PID position controller"""
    
    def __init__(self, model, data, kp=5.0, ki=0.1, kd=0.5):
        super().__init__(model, data)
        self.kp = np.ones(self.n_joints) * kp
        self.ki = np.ones(self.n_joints) * ki
        self.kd = np.ones(self.n_joints) * kd
        self.integral_error = np.zeros(self.n_joints)
        self.last_error = np.zeros(self.n_joints)
        self.dt = model.opt.timestep
        
    def compute_control(self, target_positions, target_forces=None):
        """Compute PID control signals"""
        current_pos = self.get_joint_positions()
        current_vel = self.get_joint_velocities()
        
        # Position error
        error = target_positions - current_pos
        
        # Integral error
        self.integral_error += error * self.dt
        
        # Derivative error (use actual velocity)
        deriv_error = -current_vel
        
        # PID control law
        control = (self.kp * error + 
                  self.ki * self.integral_error + 
                  self.kd * deriv_error)
        
        self.last_error = error
        
        return control

class ImpedanceController(GraspController):
    """Impedance controller for compliant grasping"""
    
    def __init__(self, model, data, k_p=50.0, k_d=5.0, k_f=0.1):
        super().__init__(model, data)
        self.k_p = np.ones(self.n_joints) * k_p  # Position stiffness
        self.k_d = np.ones(self.n_joints) * k_d  # Damping
        self.k_f = k_f  # Force feedback gain
        self.desired_impedance = np.diag(np.ones(self.n_joints) * 0.1)
        
    def compute_control(self, target_positions, target_forces=None):
        """
        Compute impedance control signals
        tau = K_p * (q_d - q) - K_d * q_dot + J^T * F_d
        """
        current_pos = self.get_joint_positions()
        current_vel = self.get_joint_velocities()
        
        # Position control term
        pos_error = target_positions - current_pos
        tau_pos = self.k_p * pos_error
        
        # Damping term
        tau_damp = -self.k_d * current_vel
        
        # Force control term (if forces provided)
        tau_force = np.zeros(self.n_joints)
        if target_forces is not None:
            # This would require the Jacobian computation
            # For now, simplified force feedback
            tau_force = self.k_f * target_forces[:self.n_joints]
        
        # Total control
        control = tau_pos + tau_damp + tau_force
        
        return control

class HybridController(GraspController):
    """Hybrid position/force controller"""
    
    def __init__(self, model, data, k_p=5.0, k_f=0.5):
        super().__init__(model, data)
        self.k_p = k_p
        self.k_f = k_f
        self.selection_matrix = np.eye(self.n_joints)  # Position control by default
        self.force_controlled_joints = []
        
    def set_force_controlled_joints(self, joint_indices):
        """Set which joints should be force controlled"""
        self.force_controlled_joints = joint_indices
        self.selection_matrix = np.eye(self.n_joints)
        for idx in joint_indices:
            self.selection_matrix[idx, idx] = 0
            
    def compute_control(self, target_positions, target_forces=None):
        """Compute hybrid control signals"""
        current_pos = self.get_joint_positions()
        
        # Position control for selected joints
        pos_error = target_positions - current_pos
        tau_pos = self.k_p * self.selection_matrix @ pos_error
        
        # Force control for other joints
        tau_force = np.zeros(self.n_joints)
        if target_forces is not None:
            force_selection = np.eye(self.n_joints) - self.selection_matrix
            tau_force = self.k_f * force_selection @ target_forces[:self.n_joints]
        
        control = tau_pos + tau_force
        
        return control

class AdaptiveGraspController(ImpedanceController):
    """Adaptive impedance controller that adjusts based on contact feedback"""
    
    def __init__(self, model, data, k_p=50.0, k_d=5.0, k_f=0.1):
        super().__init__(model, data, k_p, k_d, k_f)
        self.contact_threshold = 0.1
        self.stiffness_adaptation_rate = 0.1
        self.min_stiffness = 10.0
        self.max_stiffness = 100.0
        
    def adapt_stiffness(self, contact_forces):
        """Adapt stiffness based on contact forces"""
        # Reduce stiffness when high forces detected
        force_magnitude = np.linalg.norm(contact_forces)
        
        if force_magnitude > self.contact_threshold:
            # Reduce stiffness
            self.k_p *= (1 - self.stiffness_adaptation_rate)
            self.k_p = np.clip(self.k_p, self.min_stiffness, self.max_stiffness)
        else:
            # Increase stiffness
            self.k_p *= (1 + self.stiffness_adaptation_rate * 0.1)
            self.k_p = np.clip(self.k_p, self.min_stiffness, self.max_stiffness)
            
    def compute_control(self, target_positions, target_forces=None):
        """Compute adaptive control with contact force feedback"""
        # Get contact forces from sensors
        contact_forces = self.get_contact_forces()
        
        # Adapt stiffness
        self.adapt_stiffness(contact_forces)
        
        # Compute control using parent class method
        return super().compute_control(target_positions, target_forces)
    
    def get_contact_forces(self):
        """Extract contact forces from sensor data"""
        # This would read from force sensors
        # For now, return estimated forces based on joint torques
        return np.zeros(self.n_joints)  # Placeholder