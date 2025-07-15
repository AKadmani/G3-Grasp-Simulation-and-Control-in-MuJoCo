# Grasp Simulation and Control in MuJoCo

This project implements a complete grasp simulation pipeline using the Allegro hand in MuJoCo, controlled via ROS2, with support for multiple control strategies and grasp types.

## Project Structure

```
grasp_ws/
└── src/
    └── grasp_simulation_control/
        ├── grasp_simulation_control/
        │   ├── __init__.py
        │   ├── calculationFunctions.py
        │   ├── grasp_controllers.py
        │   ├── grasp_planner.py
        │   ├── grasp_simulation_node.py
        │   └── grasp_analysis.py
        ├── models/
        │   └── allegro_hand/
        │       ├── scene_grasp.xml
        │       └── [Allegro hand model files]
        ├── launch/
        │   └── grasp_simulation.launch.py
        ├── scripts/
        │   └── run_complete_simulation.py
        ├── setup.py
        ├── package.xml
        └── README.md
```

## Prerequisites

1. Ubuntu 20.04/22.04 with ROS2 Humble installed
2. MuJoCo (already built in your workspace)
3. Python dependencies:
   ```bash
   pip install numpy matplotlib mujoco
   ```

## Setup Instructions

1. **Download Allegro Hand Model**:
   ```bash
   cd ~/grasp_ws/src/grasp_simulation_control
   mkdir -p models/allegro_hand
   cd models/allegro_hand
   
   # Clone the mujoco_menagerie repository temporarily
   git clone https://github.com/google-deepmind/mujoco_menagerie.git temp_menagerie
   
   # Copy Allegro hand files
   cp -r temp_menagerie/wonik_allegro/* .
   
   # Clean up
   rm -rf temp_menagerie
   ```

2. **Create the scene file**:
   Copy the `scene_grasp.xml` content from the artifacts above into:
   ```
   ~/grasp_ws/src/grasp_simulation_control/models/allegro_hand/scene_grasp.xml
   ```

3. **Copy all Python files**:
   Place all the Python files from the artifacts above in their respective locations.

4. **Build the ROS2 package**:
   ```bash
   cd ~/grasp_ws
   colcon build --packages-select grasp_simulation_control
   source install/setup.bash
   ```

## Running the Simulation

### Option 1: Standalone Simulation (Recommended for testing)

```bash
cd ~/grasp_ws/src/grasp_simulation_control/grasp_simulation_control
python3 run_complete_simulation.py --controller impedance --object cylinder --grasp cylindrical --contact SF
```

Available options:
- `--controller`: pid, impedance, hybrid, adaptive
- `--object`: cylinder, box, sphere
- `--grasp`: power, precision, lateral, spherical, cylindrical
- `--contact`: SF (Soft Finger), HF (Hard Finger), FF (Frictionless)

### Option 2: ROS2 Node

```bash
# Terminal 1
ros2 launch grasp_simulation_control grasp_simulation.launch.py \
  controller_type:=impedance \
  object_type:=cylinder \
  grasp_type:=cylindrical \
  contact_model:=SF

# Terminal 2 (optional - to send commands)
ros2 topic pub /grasp_command std_msgs/String "data: reset"
```

### Option 3: Direct Python Script

```bash
cd ~/grasp_ws/src/grasp_simulation_control/grasp_simulation_control
python3 grasp_simulation_node.py
```

## Experiment Examples

1. **Cylindrical Grasp with Impedance Control**:
   ```bash
   python3 run_complete_simulation.py --controller impedance --object cylinder --grasp cylindrical
   ```

2. **Precision Grasp with PID Control**:
   ```bash
   python3 run_complete_simulation.py --controller pid --object box --grasp precision
   ```

3. **Spherical Grasp with Adaptive Control**:
   ```bash
   python3 run_complete_simulation.py --controller adaptive --object sphere --grasp spherical
   ```

## Understanding the Output

The simulation will:
1. Display the MuJoCo viewer showing the hand and objects
2. Execute the grasp sequence: approach → grasp → lift → hold
3. Print real-time grasp quality metrics
4. Generate a comprehensive report in the `reports/` directory

The report includes:
- Joint trajectory plots
- Contact force evolution
- Grasp quality metrics (force closure, singular values)
- Object trajectory
- Controller performance analysis
- Summary statistics
- PDF presentation slides

## Controller Descriptions

### PID Controller
- Simple position control with proportional, integral, and derivative terms
- Good for trajectory tracking in free space
- May cause high contact forces

### Impedance Controller
- Provides compliance during contact
- Better adaptation to object geometry
- Recommended for most grasp tasks

### Hybrid Controller
- Combines position and force control
- Allows explicit force control in selected directions
- Good for tasks requiring specific force profiles

### Adaptive Controller
- Automatically adjusts stiffness based on contact
- Best for handling uncertainties
- Reduces control effort when grasp is stable

## Grasp Types

1. **Power Grasp**: All fingers wrap around object
2. **Precision Grasp**: Thumb and index finger pinch
3. **Lateral Grasp**: Thumb against side of index
4. **Spherical Grasp**: Fingers evenly distributed
5. **Cylindrical Grasp**: Fingers wrap, thumb opposes

## Contact Models

- **SF (Soft Finger)**: Can transmit forces and one torque component
- **HF (Hard Finger)**: Can transmit only forces
- **FF (Frictionless)**: Can transmit only normal force

## Troubleshooting

1. **MuJoCo model not found**: Ensure the Allegro hand files are in the correct directory
2. **ROS2 package not found**: Run `source ~/grasp_ws/install/setup.bash`
3. **Visualization issues**: Check that your display is properly configured for MuJoCo

## Video Recording

To record a video of your simulation:
```bash
# Install recording software
sudo apt-get install simplescreenrecorder

# Or use OBS Studio
sudo apt-get install obs-studio
```

## Report Generation

Reports are automatically generated after each simulation run and saved in:
```
reports/grasp_report_YYYYMMDD_HHMMSS/
```

The report includes all necessary plots and analysis for your project submission.

## Code Explanation

### Grasp Matrix and Jacobian
The `calculationFunctions.py` module implements the mathematical foundations:
- Grasp matrix G relates contact forces to object wrench
- Jacobian J relates joint velocities to contact point velocities
- Contact models (SF, HF, FF) determine force transmission capabilities

### Control Implementation
All controllers compute joint torques/positions to achieve desired grasp:
- Motion is computed based on grasp planning and real-time feedback
- No hardcoded trajectories - all motion emerges from control laws

### Why Impedance Control?
Impedance control is preferred for grasping because:
1. Natural compliance prevents damage during contact
2. Automatic adaptation to object shape
3. Stable interaction with unknown environments
4. Handles disturbances through mechanical impedance

## Authors

Developed for the Grasp Simulation and Control course project.

## License

MIT License
