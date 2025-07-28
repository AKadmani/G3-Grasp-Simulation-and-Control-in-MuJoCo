import mujoco
import mujoco.viewer
import time

# Load the scene
model = mujoco.MjModel.from_xml_path('../models/allegro_hand/scene_grasp.xml')
data = mujoco.MjData(model)

print("Bodies in the scene:")
for i in range(model.nbody):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
    if name:
        pos = data.xpos[i]
        print(f"  {name}: position = {pos}")

# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Set camera to see the whole scene
    viewer.cam.distance = 2.0
    viewer.cam.elevation = -20
    viewer.cam.azimuth = 120
    viewer.cam.lookat[:] = [0, 0, 0.3]
    
    print("\nViewer launched. Use mouse to navigate:")
    print("- Scroll: zoom")
    print("- Left drag: rotate")
    print("- Right drag: pan")
    print("- Double-click: reset view")
    
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)
