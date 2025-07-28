#!/bin/bash
cd $HOME/grasp_ws
source install/setup.bash
cd src/grasp_simulation_control/grasp_simulation_control
python3 run_complete_simulation.py "$@"
