import mujoco
import mujoco.viewer
import numpy as np
import time
import calculationFunctions as calc

model = mujoco.MjModel.from_xml_path("/home/tim/Documents/project/G3-Grasp-Simulation-and-Control-in-MuJoCo/wonik_allegro/scene_left.xml")
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    # Let the simulation run for a short time to establish contact
    warmup_steps = 100  # Adjust as needed
    for _ in range(warmup_steps):
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)  # Slow down for visualization, adjust or remove as needed

    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()

        # Use contacts for grasp calculation
        num_contacts = data.ncon
        contact_pos_all = []
        contact_rot_all = []
        joint_positions_all = []
        joint_directions_all = []

        for i in range(num_contacts):
            contact = data.contact[i]
            # Contact position in world coordinates
            contact_pos_all.append(contact.pos.copy())
            # Get the body id associated with the contact (e.g., finger body)
            body_id = model.geom_bodyid[contact.geom1]
            # Orientation of the body in world coordinates (as 3x3 matrix)
            rot_mat = data.xmat[body_id].reshape(3, 3)
            contact_rot_all.append(rot_mat)

            # Find all joints belonging to this body (finger)
            joint_ids = [j for j in range(model.njnt) if model.jnt_bodyid[j] == body_id]
            joint_pos = []
            joint_axis = []
            for joint_id in joint_ids:
                # Joint position in world coordinates
                joint_pos.append(data.xpos[model.jnt_bodyid[joint_id]])
                # Joint axis in world coordinates
                axis_local = model.jnt_axis[joint_id]
                axis_world = data.xmat[model.jnt_bodyid[joint_id]].reshape(3, 3) @ axis_local
                joint_axis.append(axis_world)
            # Pad to 4 joints if less (for consistent shape)
            while len(joint_pos) < 4:
                joint_pos.append(np.zeros(3))
                joint_axis.append(np.zeros(3))
            joint_positions_all.append(np.array(joint_pos).T)
            joint_directions_all.append(np.array(joint_axis).T)

        contact_pos_all = np.array(contact_pos_all)
        contact_rot_all = np.array(contact_rot_all)
        joint_positions_all = np.array(joint_positions_all)
        joint_directions_all = np.array(joint_directions_all)

        # Object position (e.g., palm or object body position)
        object_body_name = "object"  # Update to your object body name
        object_body_id = model.body(object_body_name).id
        object_position = data.xpos[object_body_id]

        contact_type = "SF"  # or "HF", "FF" as needed

        # Calculate G_t and J using contacts
        gt, J = calc.grasp_matrix_transposed_and_jacobian(
            contact_pos_all,
            contact_rot_all,
            joint_positions_all,
            joint_directions_all,
            object_position,
            contact_type
        )

        print("G_t shape:", gt.shape)
        print(gt)
        print("J shape:", J.shape)
        print(J)
        break  # Only calculate and print once after warmup