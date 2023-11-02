import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as alex

G = nx.read_graphml('maps/Competition_track_graph_mod.graphml')

pos = {}
intersection = {}

for node, data in G.nodes(data=True):
    print("node: ", node)
    print("data: ", data)
    pos[node] = (data['x'], 15-data['y'])
    intersection[node] = data['intersection']
# Initialize waypoints
wp_x = []
wp_y = []

# Find the shortest path using Dijkstra's algorithm
# Assuming you are finding path between nodes '1' and '6'
path = nx.dijkstra_path(G, source='86', target='467')
# path = nx.dijkstra_path(G, source='23', target='24')

# print(path)
# exit()

# Extract edges along the path
path_edges = [(path[i], path[i + 1]) for i in range(len(path) - 1)]
for node in path:
    if not intersection[node]:
        x, y = pos[node]
        wp_x.append(x)
        wp_y.append(y)
    else:
        print("intersection: ", node)

path2 = nx.dijkstra_path(G, source='467', target='302')
path_edges2 = [(path2[i], path2[i + 1]) for i in range(len(path2) - 1)]
# for node in path2:
    # if not intersection[node]:
#         x, y = pos[node]
#         wp_x.append(x)
#         wp_y.append(y)

path3 = nx.dijkstra_path(G, source='302', target='264')
path_edges3 = [(path3[i], path3[i + 1]) for i in range(len(path3) - 1)]
# for node in path3:
#     if not intersection[node]:
#         x, y = pos[node]
#         wp_x.append(x)
#         wp_y.append(y)

path4 = nx.dijkstra_path(G, source='264', target='81')
print("path4: ", path4)
path_edges4 = [(path4[i], path4[i + 1]) for i in range(len(path4) - 1)]
# for node in path4:
#     if not intersection[node]:
#         x, y = pos[node]
#         wp_x.append(x)
#         wp_y.append(y)
#     else:
#         print("intersection: ", node)

# keep 2 decimal places
wp_x = [round(x, 2) for x in wp_x]
wp_y = [round(y, 2) for y in wp_y]
wp = list(zip(wp_x, wp_y))

# Print waypoints
print("Waypoints: ", wp)

# save as npy
np_x = alex.array(wp_x)
np_y = alex.array(wp_y)
waypoints = alex.array([np_x, np_y])
alex.save("speedrun1_noint.npy", waypoints)

# Read the image
img = mpimg.imread('maps/map.png')
print("img: ", img.shape)
# Create the plot
fig, ax = plt.subplots()

# Display the image
ax.imshow(img, extent=[0, 15, 0, 15])  # Set the extent to match the actual coordinates in your map

# Draw the graph
nx.draw(G, pos, ax=ax, with_labels=True, node_size=20, node_color='r', font_size=6)

# Highlight the path
# nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='g', width=2)
# nx.draw_networkx_edges(G, pos, edgelist=path_edges2, edge_color='g', width=2)
# nx.draw_networkx_edges(G, pos, edgelist=path_edges3, edge_color='g', width=2)
nx.draw_networkx_edges(G, pos, edgelist=path_edges4, edge_color='g', width=2)

plt.show()
