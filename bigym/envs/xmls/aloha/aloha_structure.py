import xml.etree.ElementTree as ET
import numpy as np

def parse_aloha_xml(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()
    
    robot_info = {
        'joints': {},
        'links': {}
    }
    
    for joint in root.findall(".//joint"):
        name = joint.get('name')
        joint_type = joint.get('type', 'hinge') 
        axis = np.array([float(x) for x in joint.get('axis', '0 0 1').split()])
        range_str = joint.get('range', '-3.14159 3.14159')
        range_values = [float(x) for x in range_str.split()]
        
        robot_info['joints'][name] = {
            'type': joint_type,
            'axis': axis,
            'range': range_values
        }
    
    for body in root.findall(".//body"):
        name = body.get('name')
        pos = np.array([float(x) for x in body.get('pos', '0 0 0').split()])
        
        robot_info['links'][name] = {
            'pos': pos
        }
    
    return robot_info

aloha_info = parse_aloha_xml('/Users/almondgod/Repositories/aloha-bigym/bigym/envs/xmls/aloha/aloha.xml')
scene_info = parse_aloha_xml('/Users/almondgod/Repositories/aloha-bigym/bigym/envs/xmls/aloha/scene.xml')

combined_info = {
    'joints': {**aloha_info['joints'], **scene_info['joints']},
    'links': {**aloha_info['links'], **scene_info['links']}
}

print(combined_info)