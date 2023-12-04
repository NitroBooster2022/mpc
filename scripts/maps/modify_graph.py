import networkx as nx
import os

current_dir = os.path.dirname(os.path.realpath(__file__))

attributes = ["normal", "crosswalk", "intersection", "oneway", "highwayLeft", "highwayRight", "roundabout"]
# Load the graphml file
file_path = current_dir + '/Competition_track_graph.graphml'
print('Loading graphml file from: ' + file_path)
graph = nx.read_graphml(file_path)

crosswalk = [415, 416, 417, 418, 402, 403, 404, 439, 440, 441, 442, 163,164, 165, 67, 66, 166, 167, 7, 152,153, 176,177, 8, 451, 452, 453, 454]
highwayLeft = []
for i in range(263, 282):
    highwayLeft.append(i)
for i in range(216, 235):
    highwayLeft.append(i)
highwayRight = []
for i in range(283, 302):
    highwayRight.append(i)
for i in range(239, 259):
    highwayRight.append(i)

oneway = [30]
for i in range(448, 480):
    oneway.append(i)
for i in range(401, 424):
    oneway.append(i)
for i in range(480, 490):
    oneway.append(i)

roundabout = []
for i in range(358, 369):
    roundabout.append(i)
# Add an attribute to every node with a default value of 0
for node in graph.nodes:
    if int(node) in crosswalk:
        graph.nodes[node]['new_attribute'] = 1
    elif int(node) in highwayLeft:
        graph.nodes[node]['new_attribute'] = 4
    elif int(node) in highwayRight:
        graph.nodes[node]['new_attribute'] = 5
    elif int(node) in oneway:
        graph.nodes[node]['new_attribute'] = 3
    elif int(node) in roundabout:
        graph.nodes[node]['new_attribute'] = 6
    else:
        graph.nodes[node]['new_attribute'] = 0

# Save the graphml file
file_path = current_dir + '/Competition_track_graph_modified.graphml'
nx.write_graphml(graph, file_path)

print('Done')