import mujoco
import mujoco.viewer
import time

# Load the modified scene
model = mujoco.MjModel.from_xml_path('scene_left_modified.xml')
data = mujoco.MjData(model)

print("Scene loaded successfully!")
print("\nBodies in scene:")
for i in range(model.nbody):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
    if name:
        pos = data.xpos[i]
        print(f"  {name}: pos = [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")

# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("\nViewer launched! You should see the hand AND three objects.")
    
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)
