#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
import mujoco
import mujoco.viewer
import numpy as np
import time
import threading

# Import our modules
import calculationFunctions as calc
from grasp_controllers import PIDController, ImpedanceController, HybridController, AdaptiveGraspController
from grasp_planner import GraspPlanner, GraspType

class GraspSimulationNode(Node):
    """ROS2 node for grasp simulation and control"""
    
    def __init__(self):
        super().__init__('grasp_simulation_node')
        
        # Declare parameters
        self.declare_parameter('scene_file', 'scene_grasp.xml')
        self.declare_parameter('controller_type', 'impedance')
        self.declare_parameter('object_type', 'cylinder')
        self.declare_parameter('grasp_type', 'cylindrical')
        self.declare_parameter('contact_model', 'SF')
        
        # Get parameters
        scene_file = self.get_parameter('scene_file').value
        self.controller_type = self.get_parameter('controller_type').value
        self.object_type = self.get_parameter('object_type').value
        grasp_type_str = self.get_parameter('grasp_type').value
        self.contact_model = self.get_parameter('contact_model').value
        
        # Map grasp type string to enum
        grasp_type_map = {
            'power': GraspType.POWER_GRASP,
            'precision': GraspType.PRECISION_GRASP,
            'lateral': GraspType.LATERAL_GRASP,
            'spherical': GraspType.SPHERICAL_GRASP,
            'cylindrical': GraspType.CYLINDRICAL_GRASP
        }
        self.grasp_type = grasp_type_map.get(grasp_type_str, GraspType.CYLINDRICAL_GRASP)
        
        # Load MuJoCo model
        self.model = mujoco.MjModel.from_xml_path(scene_file)
        self.data = mujoco.MjData(self.model)
        
        # Initialize controller
        if self.controller_type == 'pid':
            self.controller = PIDController(self.model, self.data)
        elif self.controller_type == 'impedance':
            self.controller = ImpedanceController(self.model, self.data)
        elif self.controller_type == 'hybrid':
            self.controller = HybridController(self.model, self.data)
        elif self.controller_type == 'adaptive':
            self.controller = AdaptiveGraspController(self.model, self.data)
        else:
            self.get_logger().error(f"Unknown controller type: {self.controller_type}")
            self.controller = ImpedanceController(self.model, self.data)
            
        # Initialize grasp planner
        self.planner = GraspPlanner(self.model, self.data)
        
        # Plan grasp
        self.grasp_plan = self.planner.plan_grasp(self.object_type, self.grasp_type)
        self.trajectory = self.grasp_plan['trajectory']
        self.trajectory_index = 0
        
        # State variables
        self.grasp_phase = 'approach'  # 'approach', 'grasp', 'lift', 'hold'
        self.phase_timer = 0
        self.grasp_matrix = None
        self.jacobian = None
        self.contact_forces = None
        
        # ROS publishers
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.grasp_state_pub = self.create_publisher(String, 'grasp_state', 10)
        self.grasp_quality_pub = self.create_publisher(Float64MultiArray, 'grasp_quality', 10)
        self.object_wrench_pub = self.create_publisher(WrenchStamped, 'object_wrench', 10)
        
        # ROS subscribers
        self.create_subscription(String, 'grasp_command', self.grasp_command_callback, 10)
        
        # Create timer for control loop
        self.control_timer = self.create_timer(0.01, self.control_callback)  # 100 Hz
        
        # Start visualization in separate thread
        self.viz_thread = threading.Thread(target=self.run_visualization)
        self.viz_thread.daemon = True
        self.viz_thread.start()
        
        self.get_logger().info(f"Grasp simulation node started with {self.controller_type} controller")
        self.get_logger().info(f"Object: {self.object_type}, Grasp type: {grasp_type_str}")
        
    def control_callback(self):
        """Main control loop callback"""
        # Get current state
        current_pos = self.controller.get_joint_positions()
        
        # Determine control action based on phase
        if self.grasp_phase == 'approach':
            # Follow pre-grasp trajectory
            if self.trajectory_index < len(self.trajectory):
                target_pos = self.trajectory[self.trajectory_index]
                self.trajectory_index += 1
            else:
                # Transition to grasp phase
                self.grasp_phase = 'grasp'
                self.phase_timer = 0
                self.get_logger().info("Transitioning to grasp phase")
                target_pos = self.grasp_plan['grasp']
                
        elif self.grasp_phase == 'grasp':
            # Close fingers on object
            target_pos = self.grasp_plan['grasp']
            self.phase_timer += 1
            
            # Check for stable contact
            if self.phase_timer > 100 and self.check_grasp_stability():
                self.grasp_phase = 'lift'
                self.phase_timer = 0
                self.get_logger().info("Grasp stable, transitioning to lift phase")
                
        elif self.grasp_phase == 'lift':
            # Maintain grasp while lifting
            target_pos = self.grasp_plan['grasp']
            
            # Apply upward force to hand base
            if self.phase_timer < 200:
                # Gradually increase upward force
                lift_force = min(self.phase_timer * 0.01, 2.0)
                base_id = self.model.body('base_link').id
                self.data.xfrc_applied[base_id, 2] = lift_force
                
            self.phase_timer += 1
            
            if self.phase_timer > 300:
                self.grasp_phase = 'hold'
                self.phase_timer = 0
                self.get_logger().info("Lift complete, holding object")
                
        else:  # hold
            # Maintain grasp
            target_pos = self.grasp_plan['grasp']
            
        # Compute control
        control_signal = self.controller.compute_control(target_pos)
        self.controller.set_control(control_signal)
        
        # Update grasp analysis
        self.update_grasp_analysis()
        
        # Publish states
        self.publish_joint_states()
        self.publish_grasp_state()
        self.publish_grasp_quality()
        self.publish_object_wrench()
        
    def update_grasp_analysis(self):
        """Update grasp matrix and quality metrics"""
        # Get contact information
        contact_data = self.get_contact_data()
        
        if contact_data['num_contacts'] > 0:
            # Calculate grasp matrix and Jacobian
            self.grasp_matrix, self.jacobian = calc.grasp_matrix_transposed_and_jacobian(
                contact_data['positions'],
                contact_data['orientations'],
                contact_data['joint_positions'],
                contact_data['joint_directions'],
                contact_data['object_position'],
                self.contact_model
            )
            
            # Compute grasp quality
            self.grasp_quality = calc.compute_grasp_quality(self.grasp_matrix, self.contact_model)
            
            # Estimate required contact forces for current object weight
            object_weight = np.array([0, 0, -2.0, 0, 0, 0])  # 2N downward
            self.contact_forces = calc.compute_contact_forces(self.grasp_matrix, object_weight)
            
    def get_contact_data(self):
        """Extract contact data from simulation"""
        num_contacts = self.data.ncon
        contact_positions = []
        contact_orientations = []
        joint_positions_all = []
        joint_directions_all = []
        
        # Get object position
        obj_name = f"{self.object_type}_object"
        try:
            obj_id = self.model.body(obj_name).id
            object_position = self.data.xpos[obj_id].copy()
        except:
            object_position = np.zeros(3)
            
        # Process each contact
        for i in range(num_contacts):
            contact = self.data.contact[i]
            
            # Check if contact involves the object
            geom1_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            geom2_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            
            if self.object_type in str(geom1_name) or self.object_type in str(geom2_name):
                # Contact position
                contact_positions.append(contact.pos.copy())
                
                # Contact orientation (simplified - using body orientation)
                if self.object_type in str(geom1_name):
                    body_id = self.model.geom_bodyid[contact.geom2]
                else:
                    body_id = self.model.geom_bodyid[contact.geom1]
                    
                rot_mat = self.data.xmat[body_id].reshape(3, 3)
                contact_orientations.append(rot_mat)
                
                # Joint data (simplified - assuming 4 joints per contact)
                joint_pos = np.zeros((3, 4))
                joint_dir = np.zeros((3, 4))
                
                # Find finger joints
                finger_joints = []
                for j in range(self.model.njnt):
                    if self.model.jnt_bodyid[j] == body_id:
                        finger_joints.append(j)
                        
                for idx, joint_id in enumerate(finger_joints[:4]):
                    joint_pos[:, idx] = self.data.xanchor[joint_id]
                    joint_dir[:, idx] = self.data.xaxis[joint_id]
                    
                joint_positions_all.append(joint_pos)
                joint_directions_all.append(joint_dir)
                
        return {
            'num_contacts': len(contact_positions),
            'positions': np.array(contact_positions) if contact_positions else np.zeros((0, 3)),
            'orientations': np.array(contact_orientations) if contact_orientations else np.zeros((0, 3, 3)),
            'joint_positions': np.array(joint_positions_all) if joint_positions_all else np.zeros((0, 3, 4)),
            'joint_directions': np.array(joint_directions_all) if joint_directions_all else np.zeros((0, 3, 4)),
            'object_position': object_position
        }
        
    def check_grasp_stability(self):
        """Check if grasp is stable"""
        contact_data = self.get_contact_data()
        
        # Need at least 3 contacts for stable grasp
        if contact_data['num_contacts'] < 3:
            return False
            
        # Check grasp quality metrics
        if self.grasp_quality is not None:
            return (self.grasp_quality['force_closure'] and 
                    self.grasp_quality['min_singular_value'] > 0.01)
                    
        return False
        
    def grasp_command_callback(self, msg):
        """Handle grasp commands"""
        command = msg.data
        if command == 'reset':
            self.reset_simulation()
        elif command == 'stop':
            self.grasp_phase = 'hold'
            
    def reset_simulation(self):
        """Reset simulation to initial state"""
        mujoco.mj_resetData(self.model, self.data)
        self.trajectory_index = 0
        self.grasp_phase = 'approach'
        self.phase_timer = 0
        self.get_logger().info("Simulation reset")
        
    def publish_joint_states(self):
        """Publish joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'joint_{i}' for i in range(16)]
        msg.position = self.controller.get_joint_positions().tolist()
        msg.velocity = self.controller.get_joint_velocities().tolist()
        msg.effort = self.data.ctrl[:16].tolist()
        self.joint_state_pub.publish(msg)
        
    def publish_grasp_state(self):
        """Publish current grasp phase"""
        msg = String()
        msg.data = f"{self.grasp_phase} (timer: {self.phase_timer})"
        self.grasp_state_pub.publish(msg)
        
    def publish_grasp_quality(self):
        """Publish grasp quality metrics"""
        if self.grasp_quality is not None:
            msg = Float64MultiArray()
            msg.data = [
                float(self.grasp_quality['force_closure']),
                self.grasp_quality['min_singular_value'],
                self.grasp_quality['condition_number'],
                self.grasp_quality['wrench_space_volume']
            ]
            self.grasp_quality_pub.publish(msg)
            
    def publish_object_wrench(self):
        """Publish estimated wrench on object"""
        if self.grasp_matrix is not None and self.contact_forces is not None:
            # Compute object wrench from contact forces
            wrench = self.grasp_matrix @ self.contact_forces
            
            msg = WrenchStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
            msg.wrench.force.x = wrench[0]
            msg.wrench.force.y = wrench[1]
            msg.wrench.force.z = wrench[2]
            msg.wrench.torque.x = wrench[3]
            msg.wrench.torque.y = wrench[4]
            msg.wrench.torque.z = wrench[5]
            self.object_wrench_pub.publish(msg)
            
    def run_visualization(self):
        """Run MuJoCo visualization"""
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                mujoco.mj_step(self.model, self.data)
                viewer.sync()
                time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = GraspSimulationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()