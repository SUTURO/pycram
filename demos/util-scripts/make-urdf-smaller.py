from xml.etree import ElementTree as ET

# Load the URDF file
file_path = ''
from xml.etree import ElementTree as ET

# Load the URDF file
file_path = '/home/vee/robocup_workspaces/pycram_ws/src/pycram/resources/suturo_lab_version_15.urdf'
tree = ET.parse(file_path)
root = tree.getroot()

# Create a new URDF tree with the robot element
new_root = ET.Element('robot', attrib={'name': 'iai-kitchen', 'version': '1.0'})

# Collect necessary links and joints
links_to_keep = set()
joints_to_keep = set()

# Add the links for popcorntable and shelf
for element in root.findall('link'):
    if 'popcorn' in element.attrib.get('name'):
        links_to_keep.add(element.attrib['name'])

# Add the joints that connect the relevant links
for element in root.findall('joint'):
    parent_link = element.find('parent').attrib.get('link')
    child_link = element.find('child').attrib.get('link')
    if parent_link in links_to_keep or child_link in links_to_keep:
        joints_to_keep.add(element.attrib['name'])
        links_to_keep.add(parent_link)
        links_to_keep.add(child_link)

# Add the collected links and joints to the new URDF tree
for element in root.findall('link'):
    if element.attrib.get('name') in links_to_keep:
        new_root.append(element)

for element in root.findall('joint'):
    if element.attrib.get('name') in joints_to_keep:
        new_root.append(element)

# Save the new URDF tree to a file
new_file_path = '/home/vee/robocup_workspaces/pycram_ws/src/pycram/resources/suturo_lab_version_15_reduced_corrected_v7.urdf'
new_tree = ET.ElementTree(new_root)
new_tree.write(new_file_path)

new_file_path
