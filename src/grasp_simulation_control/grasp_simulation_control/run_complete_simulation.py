#!/usr/bin/env python3

import mujoco
import mujoco.viewer
import numpy as np
import time
import argparse
import os
import sys

# Add the package directory to Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import calculationFunctions as calc
from grasp_controllers import PIDController, ImpedanceController, HybridController, AdaptiveGraspController
from grasp_planner import GraspPlanner, GraspType
from grasp_analysis import GraspAnalyzer

def run_complete_simulation(args):

    """Run the complete grasp simulation with visualization and analysis"""
    # Debug paths
    print(f"Current file: {__file__}")
    print(f"Current dir: {os.path.dirname(__file__)}")
    print(f"Parent dir: {os.path.dirname(os.path.dirname(__file__))}")
    
    # Try different path constructions
    path1 = os.path.join(os.path.dirname(__file__), "models/allegro_hand/scene_left_modified.xml")
    path2 = os.path.join(os.path.dirname(os.path.dirname(__file__)), "models/allegro_hand/scene_left_modified.xml")
    path3 = os.path.abspath("../models/allegro_hand/scene_left_modified.xml")
    
    print(f"Path 1: {path1} - Exists: {os.path.exists(path1)}")
    print(f"Path 2: {path2} - Exists: {os.path.exists(path2)}")
    print(f"Path 3: {path3} - Exists: {os.path.exists(path3)}")

    print("=" * 60)
    print("GRASP SIMULATION AND CONTROL IN MUJOCO")
    print("=" * 60)
    print(f"Controller: {args.controller}")
    print(f"Object: {args.object}")
    print(f"Grasp Type: {args.grasp}")
    print(f"Contact Model: {args.contact}")
    print("=" * 60)
    
    # Load MuJoCo model
    if args.object == 'cylinder':
        # Use the cylinder scene for cylinder object
        model_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 
                                  "models/allegro_hand/scene_left_zylinder.xml")
    elif args.object == 'box':
        # Use a box scene (not provided in this example, assuming similar structure)
        model_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 
                                  "models/allegro_hand/scene_left_box.xml")
    elif args.object == 'sphere':
        # Use a sphere scene (not provided in this example, assuming similar structure)
        model_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 
                                  "models/allegro_hand/scene_left_sphere.xml")
    else:
        raise ValueError(f"Unknown object type: {args.object}")
    print(f"Loading model from: {model_path}")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    # Initialize controller
    if args.controller == 'pid':
        controller = PIDController(model, data)
        print("Using PID Controller")
    elif args.controller == 'impedance':
        controller = ImpedanceController(model, data)
        print("Using Impedance Controller")
    elif args.controller == 'hybrid':
        controller = HybridController(model, data)
        print("Using Hybrid Position/Force Controller")
    elif args.controller == 'adaptive':
        controller = AdaptiveGraspController(model, data)
        print("Using Adaptive Impedance Controller")
    else:
        raise ValueError(f"Unknown controller: {args.controller}")
    
    # Initialize grasp planner
    planner = GraspPlanner(model, data)
    
    # Map grasp type
    grasp_map = {
        'power': GraspType.POWER_GRASP,
        'precision': GraspType.PRECISION_GRASP,
        'lateral': GraspType.LATERAL_GRASP,
        'spherical': GraspType.SPHERICAL_GRASP,
        'cylindrical': GraspType.CYLINDRICAL_GRASP
    }
    grasp_type = grasp_map.get(args.grasp, GraspType.CYLINDRICAL_GRASP)
    
    # Plan grasp
    grasp_plan = planner.plan_grasp(args.object, grasp_type)
    trajectory = grasp_plan['trajectory']
    
    # Initialize analyzer
    analyzer = GraspAnalyzer(model, data)
    
    # Simulation parameters
    trajectory_index = 0
    phase = 'approach'
    phase_timer = 0
    lift_start_height = None
    
    print("\nStarting simulation...")
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Warm-up phase
        freejoint_addr = 0  # Assuming palm's freejoint is at the start of qpos
        # Hold the hand at the origin before the lift phase
        z_pos = 0.0  # Initial Z position for the palm's freejoint

        print("Warming up simulation...")
        for _ in range(100):
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.03)
            data.qpos[freejoint_addr + 0] = 0.0  # x
            data.qpos[freejoint_addr + 1] = 0.0  # y
            data.qpos[freejoint_addr + 2] = 0.0  # z
            data.qpos[freejoint_addr + 3] = -1.0  # qw (identity quaternion)
            data.qpos[freejoint_addr + 4] = 1.0  # qx
            data.qpos[freejoint_addr + 5] = 0.0  # qy
            
        
        print("Beginning grasp sequence...")
        
        while viewer.is_running():
            
            
            # Control logic

            if phase == 'approach':
                if trajectory_index < len(trajectory):
                    target_pos = trajectory[trajectory_index]
                    trajectory_index += 1
                    #print(trajectory_index)
                else:
                    phase = 'grasp'
                    phase_timer = 0
                    print("Transitioning to GRASP phase")
                    #time.sleep(5)  # Pause before grasping
                    target_pos = grasp_plan['grasp']
                    
            elif phase == 'grasp':
                target_pos = grasp_plan['grasp']
                phase_timer += 1

                # Check for stable grasp after some time
                if phase_timer > 100:
                    contact_data = get_contact_data(model, data, args.object)
                    if contact_data['num_contacts'] >= 3:
                        phase = 'lift'
                        phase_timer = 0
                        # Record initial object height
                        obj_name = f"{args.object}_object"
                        obj_id = model.body(obj_name).id
                        lift_start_height = data.xpos[obj_id][2]
                        print("Grasp established, transitioning to LIFT phase")
                    else:
                        print("Insufficient contacts for stable grasp, closing hand...")
                        #here the hand has to close more to establish contact
                        
                        
                        
            elif phase == 'lift':
                target_pos = grasp_plan['grasp']
                

                # Move the hand upward by incrementing the palm's freejoint Z position
                palm_body_id = model.body('palm').id
                freejoint_addr = 0  # Assuming palm's freejoint is at the start of qpos
                # Only move for the first 200 steps
                if phase_timer <= 200:
                    # qpos[2] is Z position for freejoint (x, y, z, qw, qx, qy, qz)
                    z_pos += 0.0005  # Move up by 0.5mm per step
                
                    
                phase_timer += 1
                
                # Check if object has been lifted
                obj_name = f"{args.object}_object"
                obj_id = model.body(obj_name).id
                current_height = data.xpos[obj_id][2]
                
                if phase_timer > 100 and current_height > lift_start_height + 0.05:
                    phase = 'hold'
                    phase_timer = 0
                    print(f"Lift successful! Object raised {current_height - lift_start_height:.3f}m")
                    print("Transitioning to HOLD phase")
                    
            else:  # hold
                target_pos = grasp_plan['grasp']
                # Maintain upward force
                base_id = model.body('palm').id
                data.xfrc_applied[base_id, 2] = 3.0
            
            # Compute and apply control
            control_signal = controller.compute_control(target_pos)
            controller.set_control(control_signal)
            joint_error =  controller.get_joint_positions() - target_pos
            
            data.qpos[freejoint_addr + 0] = 0.0  # x
            data.qpos[freejoint_addr + 1] = 0.0  # y
            data.qpos[freejoint_addr + 2] = z_pos  # z
            data.qpos[freejoint_addr + 3] = -1.0  # qw (identity quaternion)
            data.qpos[freejoint_addr + 4] = 1.0  # qx
            data.qpos[freejoint_addr + 5] = 0.0  # qy
            data.qpos[freejoint_addr + 6] = 0.0  # qz


            # Step simulation
            mujoco.mj_step(model, data)
            viewer.sync()
            
            # Perform grasp analysis
            contact_data = get_contact_data(model, data, args.object)
            
            grasp_quality = None
            if contact_data['num_contacts'] > 0:
                try:
                    G_t, J = calc.grasp_matrix_transposed_and_jacobian(
                        contact_data['positions'],
                        contact_data['orientations'],
                        contact_data['joint_positions'],
                        contact_data['joint_directions'],
                        contact_data['object_position'],
                        args.contact
                    )
                    
                    grasp_quality = calc.compute_grasp_quality(G_t, args.contact)
                    
                    # Print grasp quality periodically
                    if phase_timer % 50 == 0 and phase != 'approach':
                        print(f"Phase: {phase}, Contacts: {contact_data['num_contacts']}, "
                              f"Force Closure: {grasp_quality['force_closure']}, "
                              f"Min SV: {grasp_quality['min_singular_value']:.3f}")
                        
                except Exception as e:
                    # Grasp matrix calculation might fail with insufficient contacts
                    pass
            
            # Record data for analysis (now also passing joint error and planned position)
            analyzer.record_state(controller, grasp_quality, joint_error, target_pos)
            
            # Check for termination
            if phase == 'hold' and phase_timer > 100:
                print("\nSimulation complete!")
                break
                
            time.sleep(0.01)  # Small delay for visualization
    
    # Generate report
    print("\nGenerating analysis report...")
    report_dir = analyzer.generate_report()
    print(f"Report saved to: {report_dir}")
    
    # Print final statistics
    print("\n" + "=" * 60)
    print("SIMULATION SUMMARY")
    print("=" * 60)
    
    if analyzer.results['grasp_quality']:
        successful = any(q['force_closure'] for q in analyzer.results['grasp_quality'] if q)
        print(f"Grasp Success: {'YES' if successful else 'NO'}")
        
    if analyzer.results['object_pose'] and len(analyzer.results['object_pose']) > 1:
        initial_height = analyzer.results['object_pose'][0][2]
        final_height = analyzer.results['object_pose'][-1][2]
        print(f"Object Lift Height: {final_height - initial_height:.3f} m")
        
    duration = analyzer.results['timestamps'][-1] - analyzer.results['timestamps'][0]
    print(f"Total Duration: {duration:.2f} seconds")
    
    # Controller-specific discussion
    print("\n" + "=" * 60)
    print("CONTROLLER DISCUSSION")
    print("=" * 60)
    
    if args.controller == 'impedance':
        print("Impedance Control Advantages:")
        print("- Provides compliance during contact, reducing impact forces")
        print("- Natural adaptation to object geometry through mechanical impedance")
        print("- Better handling of uncertainties in object position and properties")
        print("- Inherent stability in contact situations")
        
    elif args.controller == 'pid':
        print("PID Control Characteristics:")
        print("- Simple implementation with well-understood tuning")
        print("- Good for position tracking in free space")
        print("- May cause high contact forces without careful gain tuning")
        print("- Requires accurate trajectory planning")
        
    elif args.controller == 'hybrid':
        print("Hybrid Position/Force Control Benefits:")
        print("- Allows explicit force control in selected directions")
        print("- Position control for shape adaptation, force control for grasp strength")
        print("- Optimal for tasks requiring specific force profiles")
        print("- More complex but provides better task-specific performance")
        
    elif args.controller == 'adaptive':
        print("Adaptive Impedance Control Features:")
        print("- Automatically adjusts stiffness based on contact feedback")
        print("- Reduces control effort when stable grasp is achieved")
        print("- Increases robustness to object variations")
        print("- Combines benefits of impedance control with online adaptation")
    
    print("\nDisturbance Handling:")
    print("- Contact instabilities managed through compliant control")
    print("- Force feedback prevents excessive contact forces")
    print("- Adaptive gains (if used) respond to changing conditions")
    
def get_contact_data(model, data, object_type):
    """Extract contact data from simulation"""

    num_contacts = data.ncon
    contact_positions = []
    contact_orientations = []
    joint_positions_all = []
    joint_directions_all = []
    
    # Get object position
    obj_name = f"{object_type}_object"
    try:
        obj_id = model.body(obj_name).id
        object_position = data.xpos[obj_id].copy()
    except:
        object_position = np.zeros(3)
        
    # Process contacts
    for i in range(num_contacts):
        contact = data.contact[i]
        
        # Check if contact involves the object
        geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
        geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
        if geom1_name == "floor" or geom2_name == "floor":
            continue
        elif object_type in str(geom1_name) or object_type in str(geom2_name):
            contact_positions.append(contact.pos.copy())
            
            # Determine which geom is the finger
            if object_type in str(geom1_name):
                finger_geom = contact.geom2
            else:
                finger_geom = contact.geom1
                
            body_id = model.geom_bodyid[finger_geom]
            rot_mat = data.xmat[body_id].reshape(3, 3)
            contact_orientations.append(rot_mat)
            
            # Simplified joint data
            joint_pos = np.zeros((3, 4))
            joint_dir = np.zeros((3, 4))
            
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

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run grasp simulation')
    parser.add_argument('--controller', type=str, default='impedance',
                        choices=['pid', 'impedance', 'hybrid', 'adaptive'],
                        help='Controller type')
    parser.add_argument('--object', type=str, default='cylinder',
                        choices=['cylinder', 'box', 'sphere'],
                        help='Object to grasp')
    parser.add_argument('--grasp', type=str, default='cylindrical',
                        choices=['power', 'precision', 'lateral', 'spherical', 'cylindrical'],
                        help='Grasp type')
    parser.add_argument('--contact', type=str, default='SF',
                        choices=['SF', 'HF', 'FF'],
                        help='Contact model')
    
    args = parser.parse_args()
    run_complete_simulation(args)