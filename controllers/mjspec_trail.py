import numpy as np
import mujoco as mj
import mujoco.viewer
import time
import os

def list_joints():
    # Get the absolute path to the dishwasher.xml file
    current_dir = os.path.dirname(os.path.abspath(__file__))
    scene_path = os.path.join(current_dir, "..", "bigym", "envs", "xmls", "props", "dishwasher", "dishwasher.xml")
    
    # Load the model spec from XML
    spec = mj.MjSpec.from_file(scene_path)
    
    # Create list to store joint information
    joints_list = []
    
    # Collect joint information
    for joint in spec.joints:
        joint_info = {
            'name': joint.name,
            'type': joint.type,
            'range': joint.range,
            'damping': joint.damping,
            'frictionloss': joint.frictionloss
        }
        joints_list.append(joint_info)
    
    return joints_list, spec, scene_path

if __name__ == "__main__":
    joints, spec, scene_path = list_joints()
    
    print(f"\n=== Dishwasher Joints (Total: {len(joints)}) ===")
    for joint in joints:
        print(f"\nJoint: {joint['name']}")
        print(f"  Type: {joint['type']}")
        print(f"  Range: {joint['range']}")
        print(f"  Damping: {joint['damping']}")
        print(f"  Friction Loss: {joint['frictionloss']}")
    
    # Create model and data from spec
    model = mj.MjModel.from_xml_path(scene_path)  
    data = mj.MjData(model)
    
    # Launch the viewer
    with mujoco.viewer.launch(model, data) as viewer:
        while viewer.is_running():
            # Step the simulation
            mj.mj_step(model, data)
            # Update the viewer
            viewer.sync()
            time.sleep(0.01)  # Small delay to control simulation speed
