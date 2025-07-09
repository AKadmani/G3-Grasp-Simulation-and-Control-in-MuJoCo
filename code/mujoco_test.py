import mujoco
import mujoco.viewer
import numpy as np
import time

model = mujoco.MjModel.from_xml_path("/home/tim/Documents/project/wonik_allegro/scene_left.xml")
data = mujoco.MjData(model)


with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        # print("Joint Angles:")
        # for i in range(model.njnt):
        #     name = model.joint(i).name
        #     idx = model.joint(i).qposadr
        #     print(f"{name}: {float(data.qpos[idx]):.3f}")