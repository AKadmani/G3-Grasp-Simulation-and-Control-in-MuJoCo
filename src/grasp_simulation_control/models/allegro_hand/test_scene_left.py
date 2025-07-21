import mujoco
import mujoco.viewer
import time

# Load scene_left.xml directly
model = mujoco.MjModel.from_xml_path('scene_left.xml')
data = mujoco.MjData(model)

print("Scene loaded successfully!")
print(f"Number of bodies: {model.nbody}")
print(f"Number of joints: {model.njnt}")
print(f"Number of geoms: {model.ngeom}")

print("\nBodies in scene:")
for i in range(model.nbody):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
    if name:
        print(f"  {name}")

# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("\nViewer launched! The hand should be visible.")
    print("Controls: scroll=zoom, left-drag=rotate, right-drag=pan")
    
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)
