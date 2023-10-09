import xml.etree.ElementTree as ET

# Parse the GraphML file
tree = ET.parse('/home/simonli/Documents/Multi-Purpose-MPC-master/src/maps/Competition_track_graph_mod.graphml')
root = tree.getroot()

# Define the namespace
ns = {'graphml': 'http://graphml.graphdrawing.org/xmlns'}

# Add a new key element for the new attribute
new_key = ET.Element('key', attrib={
    'attr.name': 'intersection',
    'attr.type': 'boolean',
    'for': 'node',
    'id': 'd3',
})
root.insert(2, new_key)

active_node_ids = {'48', '57', '39', '75', '84', '10', '64', '30', '20'}

# Add a new data element for the new attribute to each node
for node in root.findall('graphml:graph/graphml:node', namespaces=ns):
    new_data = ET.Element('data', attrib={'key': 'd3'})
    if node.attrib['id'] in active_node_ids:
        new_data.text = 'true'
    else:
        new_data.text = 'false'
    node.append(new_data)

# Write the modified tree to a new GraphML file
tree.write('/home/simonli/Documents/Multi-Purpose-MPC-master/src/maps/Competition_track_graph_mod.graphml')
