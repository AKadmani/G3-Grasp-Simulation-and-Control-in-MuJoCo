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
        return self.data.qpos[7:self.n_joints+7].copy()
    
    def get_joint_velocities(self):
        """Get current joint velocities"""
        return self.data.qvel[6:self.n_joints+6].copy()
    
    def set_control(self, control_signals):
        """Apply control signals to actuators, enforcing joint torque limits"""
        # Allegro hand torque limits (Nm)
        torque_min = -0.7
        torque_max = 0.7
        clamped = np.clip(control_signals, torque_min, torque_max)
        self.data.ctrl[:self.n_joints] = clamped
        # Store the actually applied torques for analysis (optional, for analyzer)
        self.last_applied_torque = clamped.copy()

class PIDController(GraspController):
    """PID position controller"""
    
    def __init__(self, model, data, kp=15.0, ki=0.3, kd=2.0):
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
    
    def __init__(self, model, data, k_p=5, k_d=0.11, k_f=0.1):
        super().__init__(model, data)
        self.k_p = np.ones(self.n_joints) * k_p  # Position stiffness (default)
        self.k_d = np.ones(self.n_joints) * k_d  # Damping
        self.k_f = k_f  # Force feedback gain
        self.desired_impedance = np.diag(np.ones(self.n_joints) * 0.1)
        
    def compute_control(self, target_positions, target_forces=None, jacobian=None):
        """
        Compute impedance control signals
        tau = K_p * (q_d - q) - K_d * q_dot + J^T * F_d (if J available)
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
    
    def __init__(self, model, data, k_p=5, k_d=0.07, k_f=0.1):
        super().__init__(model, data, k_p, k_d, k_f)
        self.contact_threshold = 0.1
        self.stiffness_adaptation_rate = 0.1
        self.min_stiffness = 1
        self.max_stiffness = 10
        
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
    
    def modulate_grip_force(self, contact_data):
        """Modulate grip force based on estimated contact forces using J and G matrices."""
        # Import calculation functions
        import calculationFunctions as calc
        # Estimate G and J using available contact data
        try:
            G_t, J = calc.grasp_matrix_transposed_and_jacobian(
                contact_data['positions'],
                contact_data['orientations'],
                contact_data['joint_positions'],
                contact_data['joint_directions'],
                contact_data['object_position'],
                'SF'  # or use args.contact if available
            )
            # Estimate joint torques (tau) from current control
            tau = self.data.ctrl[:self.n_joints]
            # Estimate contact forces: F = (J^T)^+ tau (pseudo-inverse)
            if J.shape[0] > 0 and J.shape[1] > 0:
                JT = J.transpose()
                JT_pinv = np.linalg.pinv(JT)
                F_est = JT_pinv @ tau
                # Simple rule: if norm of F_est is low, increase k_p; if high, decrease
                force_magnitude = np.linalg.norm(F_est)
                if hasattr(self, 'k_p'):
                    if force_magnitude < 0.1:
                        self.k_p *= 1.005  # Increase grip
                    elif force_magnitude > 1.0:
                        self.k_p *= 0.995  # Decrease grip
                    # Clamp to reasonable range
                    self.k_p = np.clip(self.k_p, 1.0, 100.0)
        except Exception as e:
            # If calculation fails, do nothing
            pass