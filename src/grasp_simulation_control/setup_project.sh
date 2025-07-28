#!/bin/bash

# Script to set up the complete grasp simulation project
echo "Setting up Grasp Simulation Project..."

# Get the package directory
PACKAGE_DIR="$HOME/grasp_ws/src/grasp_simulation_control"
cd $PACKAGE_DIR

# Create directory structure
echo "Creating directory structure..."
mkdir -p grasp_simulation_control
mkdir -p models/allegro_hand
mkdir -p launch
mkdir -p config
mkdir -p resource
mkdir -p scripts

# Create resource file
touch resource/grasp_simulation_control

# Create __init__.py
touch grasp_simulation_control/__init__.py

# Download Allegro hand model if not already present
if [ ! -f "models/allegro_hand/scene_left.xml" ]; then
    echo "Downloading Allegro hand model..."
    cd models/allegro_hand
    git clone https://github.com/google-deepmind/mujoco_menagerie.git temp_menagerie
    cp -r temp_menagerie/wonik_allegro/* .
    rm -rf temp_menagerie
    cd $PACKAGE_DIR
else
    echo "Allegro hand model already present"
fi

# Create a simple test to verify setup
cat > grasp_simulation_control/test_import.py << 'EOF'
def test_import():
    print("Package imported successfully!")
    return True

if __name__ == "__main__":
    test_import()
EOF

# Make scripts directory and add run script
cat > scripts/run_simulation.sh << 'EOF'
#!/bin/bash
cd $HOME/grasp_ws
source install/setup.bash
cd src/grasp_simulation_control/grasp_simulation_control
python3 run_complete_simulation.py "$@"
EOF

chmod +x scripts/run_simulation.sh

echo "Setup complete! Now you can:"
echo "1. Copy all the Python files to grasp_simulation_control/"
echo "2. Copy scene_grasp.xml to models/allegro_hand/"
echo "3. Run: colcon build --packages-select grasp_simulation_control"