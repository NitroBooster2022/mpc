import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as alex
import os

class GlobalPlanner:
    def __init__(self):
        self.current_dir = os.path.dirname(os.path.realpath(__file__))
        self.G = nx.read_graphml(self.current_dir + '/maps/Competition_track_graph_modified.graphml')
        # self.G = nx.read_graphml(self.current_dir + '/maps/Competition_track_graph.graphml')
        self.pos = {}
        self.attribute = {}
        for node, data in self.G.nodes(data=True):
            # print("node: ", node)
            self.pos[node] = (data['x'], 15-data['y'])
            self.attribute[node] = data['new_attribute']
        self.wp_x = []
        self.wp_y = []
        self.place_names = {}

        self.place_names = {
            "parallel_parking": 454,
            "highway_entrance_east": 262,
            "highway_entrance_west": 194,
            "roundabout_entrance_east": 370,
            "roundabout_exit_east": 302,
            "roundabout_entrance_west": 230,
            "roundabout_exit_west": 260,
            "roundabout_entrance_north": 331,
            "roundabout_exit_north": 332,
            "roundabout_entrance_south": 357,
            "roundabout_exit_south": 372,
            "uniri_square_entrance": 480,
            "avram_entance": 404,
            "speed_entrance_south": 7,
            "speed_entrance_west": 66,
            "bus_lane_entrance": 427,
            "start": 472,
            "end": 467,
        }
        max_x = float('-inf')
        max_y = float('-inf')

        # Loop through each node in the graph
        for _, data in self.G.nodes(data=True):
            # Update the maximum x value
            if 'x' in data:
                max_x = max(max_x, float(data['x']))
            # Update the maximum y value
            if 'y' in data:
                max_y = max(max_y, float(data['y']))

        # Print out the maximum x and y values
        print(f"The maximum x value is: {max_x}")
        print(f"The maximum y value is: {max_y}")
        
    def plan_path(self, start, end):
        path = nx.dijkstra_path(self.G, source=str(start), target=str(end))
        path_edges = [(path[i], path[i + 1]) for i in range(len(path) - 1)]
        wp_x = []
        wp_y = []
        wp_attributes = []
        for node in path:
            x, y = self.pos[node]
            wp_x.append(x)
            wp_y.append(y)
            wp_attributes.append(self.attribute.get(node, 0))
        return alex.array([wp_x, wp_y]), path_edges, wp_attributes
    def illustrate_path(self, start, end):
        _, path_edges, _ = self.plan_path(start, end)
        img = mpimg.imread(self.current_dir+'/maps/Competition_track_graph.png')
        # img = mpimg.imread(self.current_dir+'/maps/Competition_track_graph.png')
        print("img: ", img.shape)
        # Create the plot
        fig, ax = plt.subplots()

        color_map = {
            0: 'blue',     # normal
            1: 'yellow',   # crosswalk
            2: 'green',    # intersection
            3: 'red',      # oneway
            4: 'purple',   # highwayLeft
            5: 'orange',    # highwayRight
            6: 'pink'      # roundabout
        }
        node_colors = [color_map[self.attribute.get(node, 0)] for node in self.G.nodes()]
        # Display the image
        # ax.imshow(img, extent=[0, 20.5, 1.2, 14.35]) 

        # Draw the graph
        nx.draw(self.G, self.pos, ax=ax, with_labels=True, node_size=20, node_color=node_colors, font_size=6)
        
        # Highlight the path
        nx.draw_networkx_edges(self.G, self.pos, edgelist=path_edges, edge_color='g', width=2)
        
        plt.show()
  
if __name__ == "__main__":
    planner = GlobalPlanner()
    planner.illustrate_path(86, 467)
    