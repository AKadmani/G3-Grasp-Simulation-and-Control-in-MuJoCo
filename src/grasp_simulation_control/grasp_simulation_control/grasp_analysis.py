#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, FancyBboxPatch
from mpl_toolkits.mplot3d import Axes3D
import mujoco
import time
import os
from datetime import datetime

class GraspAnalyzer:
    """Analyze and visualize grasp simulation results"""
    
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.results = {
            'timestamps': [],
            'joint_positions': [],
            'joint_velocities': [],
            'joint_torques': [],
            'contact_forces': [],
            'grasp_quality': [],
            'object_pose': [],
            'controller_errors': []
        }
        
    def record_state(self, controller, grasp_quality=None, joint_error=None, planned_position=None):
        """Record current state for analysis, now with joint error and planned position"""
        self.results['timestamps'].append(time.time())
        self.results['joint_positions'].append(controller.get_joint_positions().copy())
        self.results['joint_velocities'].append(controller.get_joint_velocities().copy())
        self.results['joint_torques'].append(self.data.ctrl[:16].copy())
        
        # Record contact forces
        contact_forces = []
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            # Simplified force calculation
            force = np.linalg.norm(contact.frame[:3])
            contact_forces.append(force)
        self.results['contact_forces'].append(contact_forces)
        
        # Record grasp quality
        if grasp_quality is not None:
            self.results['grasp_quality'].append(grasp_quality)
            
        # Record object pose
        try:
            obj_id = self.model.body('cylinder_object').id
            obj_pos = self.data.xpos[obj_id].copy()
            obj_quat = self.data.xquat[obj_id].copy()
            self.results['object_pose'].append(np.concatenate([obj_pos, obj_quat]))
        except:
            self.results['object_pose'].append(np.zeros(7))
            
        # Record joint error
        if joint_error is not None:
            if 'joint_errors' not in self.results:
                self.results['joint_errors'] = []
            self.results['joint_errors'].append(joint_error.copy())
            
        # Record planned (target) joint position
        if planned_position is not None:
            if 'planned_positions' not in self.results:
                self.results['planned_positions'] = []
            self.results['planned_positions'].append(planned_position.copy())

    def generate_report(self, output_dir='reports'):
        """Generate comprehensive analysis report"""
        os.makedirs(output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_dir = os.path.join(output_dir, f"grasp_report_{timestamp}")
        os.makedirs(report_dir, exist_ok=True)
        
        # Generate plots
        self.plot_joint_trajectories(report_dir)
        self.plot_contact_forces(report_dir)
        self.plot_grasp_quality_metrics(report_dir)
        self.plot_object_trajectory(report_dir)
        self.plot_controller_performance(report_dir)
        
        # Generate summary statistics
        self.generate_summary_stats(report_dir)
        
        # Create presentation slides
        self.create_presentation_slides(report_dir)
        
        return report_dir
        
    def plot_joint_trajectories(self, output_dir):
        """Plot joint position trajectories, joint errors, and planned positions"""
        if not self.results['joint_positions']:
            return
            
        joint_positions = np.array(self.results['joint_positions'])
        timestamps = np.array(self.results['timestamps'])
        timestamps = timestamps - timestamps[0]  # Relative time
        
        fig, axes = plt.subplots(4, 4, figsize=(16, 12))
        axes = axes.flatten()
        
        # Plot joint positions
        for i in range(16):
            ax = axes[i]
            ax.plot(timestamps, joint_positions[:, i], label='Position')
            ax.set_title(f'Joint {i}')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Position (rad)')
            ax.grid(True)
            # Plot joint error if available
            if 'joint_errors' in self.results and self.results['joint_errors']:
                joint_errors = np.array(self.results['joint_errors'])
                ax.plot(timestamps[:len(joint_errors)], joint_errors[:, i], label='Error')
            # Plot planned position if available
            if 'planned_positions' in self.results and self.results['planned_positions']:
                planned_positions = np.array(self.results['planned_positions'])
                ax.plot(timestamps[:len(planned_positions)], planned_positions[:, i], label='Planned')
            ax.legend()
            
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, 'joint_trajectories.png'))
        plt.close()
        
    def plot_contact_forces(self, output_dir):
        """Plot contact force evolution"""
        if not self.results['contact_forces']:
            return
            
        fig, ax = plt.subplots(figsize=(10, 6))
        
        # Plot total contact force over time
        timestamps = np.array(self.results['timestamps'])
        timestamps = timestamps - timestamps[0]
        
        total_forces = []
        for forces in self.results['contact_forces']:
            total_forces.append(sum(forces) if forces else 0)
            
        ax.plot(timestamps, total_forces, linewidth=2)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Total Contact Force (N)')
        ax.set_title('Contact Force Evolution During Grasp')
        ax.grid(True)
        
        # Mark grasp phases
        ax.axvline(x=2.0, color='r', linestyle='--', label='Grasp Start')
        ax.axvline(x=3.0, color='g', linestyle='--', label='Lift Start')
        ax.legend()
        
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, 'contact_forces.png'))
        plt.close()
        
    def plot_grasp_quality_metrics(self, output_dir):
        """Plot grasp quality metrics over time"""
        if not self.results['grasp_quality']:
            return
            
        timestamps = np.array(self.results['timestamps'])
        timestamps = timestamps - timestamps[0]
        
        # Extract quality metrics
        force_closure = []
        min_singular = []
        condition_num = []
        
        for quality in self.results['grasp_quality']:
            if quality:
                force_closure.append(float(quality['force_closure']))
                min_singular.append(quality['min_singular_value'])
                condition_num.append(min(quality['condition_number'], 100))  # Cap for visualization
                
        if not force_closure:
            return
            
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10))
        
        # Force closure
        ax1.plot(timestamps[:len(force_closure)], force_closure, 'b-', linewidth=2)
        ax1.set_ylabel('Force Closure')
        ax1.set_ylim(-0.1, 1.1)
        ax1.grid(True)
        ax1.set_title('Grasp Quality Metrics')
        
        # Minimum singular value
        ax2.plot(timestamps[:len(min_singular)], min_singular, 'g-', linewidth=2)
        ax2.set_ylabel('Min Singular Value')
        ax2.grid(True)
        
        # Condition number
        ax3.plot(timestamps[:len(condition_num)], condition_num, 'r-', linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Condition Number')
        ax3.grid(True)
        
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, 'grasp_quality.png'))
        plt.close()
        
    def plot_object_trajectory(self, output_dir):
        """Plot 3D trajectory of grasped object"""
        if not self.results['object_pose']:
            return
            
        object_poses = np.array(self.results['object_pose'])
        
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot trajectory
        ax.plot(object_poses[:, 0], object_poses[:, 1], object_poses[:, 2], 'b-', linewidth=2)
        
        # Mark start and end
        ax.scatter(*object_poses[0, :3], color='g', s=100, label='Start')
        ax.scatter(*object_poses[-1, :3], color='r', s=100, label='End')
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('Object Trajectory During Grasp and Lift')
        ax.legend()
        
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, 'object_trajectory.png'))
        plt.close()
        
    def plot_controller_performance(self, output_dir):
        """Plot controller performance metrics"""
        if not self.results['joint_torques']:
            return
            
        joint_torques = np.array(self.results['joint_torques'])
        timestamps = np.array(self.results['timestamps'])
        timestamps = timestamps - timestamps[0]
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
        
        # Total torque magnitude
        torque_magnitude = np.linalg.norm(joint_torques, axis=1)
        ax1.plot(timestamps, torque_magnitude, 'b-', linewidth=2)
        ax1.set_ylabel('Total Torque Magnitude (Nm)')
        ax1.set_title('Controller Performance')
        ax1.grid(True)
        
        # Torque variation (smoothness metric)
        if len(joint_torques) > 1:
            torque_diff = np.diff(joint_torques, axis=0)
            torque_variation = np.linalg.norm(torque_diff, axis=1)
            ax2.plot(timestamps[1:], torque_variation, 'r-', linewidth=2)
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Torque Variation (Nm/s)')
            ax2.grid(True)
            
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, 'controller_performance.png'))
        plt.close()
        
    def generate_summary_stats(self, output_dir):
        """Generate summary statistics file"""
        stats_file = os.path.join(output_dir, 'summary_statistics.txt')
        
        with open(stats_file, 'w') as f:
            f.write("GRASP SIMULATION SUMMARY STATISTICS\n")
            f.write("=" * 50 + "\n\n")
            
            # Simulation duration
            if self.results['timestamps']:
                duration = self.results['timestamps'][-1] - self.results['timestamps'][0]
                f.write(f"Total simulation duration: {duration:.2f} seconds\n")
                
            # Grasp success
            if self.results['grasp_quality']:
                successful_grasps = sum(1 for q in self.results['grasp_quality'] 
                                      if q and q['force_closure'])
                total_checks = len([q for q in self.results['grasp_quality'] if q])
                if total_checks > 0:
                    success_rate = successful_grasps / total_checks * 100
                    f.write(f"Grasp success rate: {success_rate:.1f}%\n")
                    
            # Object displacement
            if self.results['object_pose'] and len(self.results['object_pose']) > 1:
                initial_pos = self.results['object_pose'][0][:3]
                final_pos = self.results['object_pose'][-1][:3]
                displacement = np.linalg.norm(final_pos - initial_pos)
                lift_height = final_pos[2] - initial_pos[2]
                f.write(f"Total object displacement: {displacement:.3f} m\n")
                f.write(f"Object lift height: {lift_height:.3f} m\n")
                
            # Contact forces
            if self.results['contact_forces']:
                all_forces = [f for forces in self.results['contact_forces'] for f in forces]
                if all_forces:
                    avg_force = np.mean(all_forces)
                    max_force = np.max(all_forces)
                    f.write(f"Average contact force: {avg_force:.2f} N\n")
                    f.write(f"Maximum contact force: {max_force:.2f} N\n")
                    
            # Controller effort
            if self.results['joint_torques']:
                torques = np.array(self.results['joint_torques'])
                avg_torque = np.mean(np.abs(torques))
                max_torque = np.max(np.abs(torques))
                f.write(f"Average joint torque: {avg_torque:.3f} Nm\n")
                f.write(f"Maximum joint torque: {max_torque:.3f} Nm\n")
                
    def create_presentation_slides(self, output_dir):
        """Create presentation slides summarizing results"""
        from matplotlib.backends.backend_pdf import PdfPages
        
        pdf_file = os.path.join(output_dir, 'grasp_simulation_report.pdf')
        
        with PdfPages(pdf_file) as pdf:
            # Title slide
            fig = plt.figure(figsize=(10, 7.5))
            fig.text(0.5, 0.6, 'Grasp Simulation and Control', 
                    ha='center', va='center', fontsize=28, weight='bold')
            fig.text(0.5, 0.4, 'Using MuJoCo and ROS2', 
                    ha='center', va='center', fontsize=20)
            fig.text(0.5, 0.2, datetime.now().strftime("%B %d, %Y"), 
                    ha='center', va='center', fontsize=16)
            plt.axis('off')
            pdf.savefig(fig)
            plt.close()
            
            # System architecture slide
            fig = plt.figure(figsize=(10, 7.5))
            ax = fig.add_subplot(111)
            
            # Draw architecture diagram
            components = [
                ('ROS2 Node', (0.2, 0.7), 'lightblue'),
                ('MuJoCo Sim', (0.5, 0.7), 'lightgreen'),
                ('Controller', (0.8, 0.7), 'lightyellow'),
                ('Grasp Planner', (0.2, 0.3), 'lightcoral'),
                ('Analysis', (0.5, 0.3), 'lightgray')
            ]
            
            for name, pos, color in components:
                box = FancyBboxPatch((pos[0]-0.1, pos[1]-0.05), 0.2, 0.1,
                                    boxstyle="round,pad=0.01",
                                    facecolor=color, edgecolor='black')
                ax.add_patch(box)
                ax.text(pos[0], pos[1], name, ha='center', va='center', fontsize=12)
                
            # Draw connections
            ax.arrow(0.3, 0.7, 0.1, 0, head_width=0.02, head_length=0.02, fc='black')
            ax.arrow(0.6, 0.7, 0.1, 0, head_width=0.02, head_length=0.02, fc='black')
            ax.arrow(0.5, 0.65, 0, -0.25, head_width=0.02, head_length=0.02, fc='black')
            
            ax.set_xlim(0, 1)
            ax.set_ylim(0, 1)
            ax.set_title('System Architecture', fontsize=18, weight='bold')
            ax.axis('off')
            pdf.savefig(fig)
            plt.close()
            
            # Results summary slide
            fig = plt.figure(figsize=(10, 7.5))
            ax = fig.add_subplot(111)
            
            summary_text = self._get_summary_text()
            ax.text(0.1, 0.8, 'Simulation Results Summary', fontsize=20, weight='bold')
            ax.text(0.1, 0.1, summary_text, fontsize=12, verticalalignment='bottom')
            ax.set_xlim(0, 1)
            ax.set_ylim(0, 1)
            ax.axis('off')
            pdf.savefig(fig)
            plt.close()
            
    def _get_summary_text(self):
        """Generate summary text for presentation"""
        lines = []
        
        if self.results['timestamps']:
            duration = self.results['timestamps'][-1] - self.results['timestamps'][0]
            lines.append(f"• Simulation Duration: {duration:.2f} seconds")
            
        if self.results['grasp_quality']:
            successful = any(q['force_closure'] for q in self.results['grasp_quality'] if q)
            lines.append(f"• Grasp Success: {'Yes' if successful else 'No'}")
            
        if self.results['object_pose'] and len(self.results['object_pose']) > 1:
            lift_height = self.results['object_pose'][-1][2] - self.results['object_pose'][0][2]
            lines.append(f"• Object Lift Height: {lift_height:.3f} m")
            
        lines.append("• Controller Type: Impedance Control")
        lines.append("• Contact Model: Soft Finger (SF)")
        
        return '\n'.join(lines)