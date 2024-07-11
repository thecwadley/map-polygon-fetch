import osmnx as ox
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import networkx as nx
from flask import Flask, jsonify
from flask_cors import CORS
import json
import matplotlib.pyplot as plt
import random
import numpy as np

app = Flask(__name__)
CORS(app)

def calculate_angle(G, a, b, c):
    a = np.array([G.nodes[a]['x'], G.nodes[a]['y']])  # First
    b = np.array([G.nodes[b]['x'], G.nodes[b]['y']])  # Mid
    c = np.array([G.nodes[c]['x'], G.nodes[c]['y']])  # End

    radians = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
    if radians < 0:
        radians = radians + (2 * np.pi)
    angle = np.abs(radians * 180.0 / np.pi)

    return round(angle, 1)

def get_polygons(G, tG):
    letters = ['A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z']
    current_letter = 0
    letter_nodes = dict()

    all_shapes = []
    all_edges = dict()
    
    visits_needed = []
    for e in G.edges:
        visits_needed.append(e)
        all_edges[e] = []

        if not (e[0] in letter_nodes):
            letter_nodes[e[0]] = letters[current_letter // 26] + letters[current_letter % 26]
            current_letter += 1
        if not (e[1] in letter_nodes):
            letter_nodes[e[1]] = letters[current_letter // 26] + letters[current_letter % 26]
            current_letter += 1
    
    while len(visits_needed) > 0:
        # print('creating shape...')
        starting_edge = random.choice(visits_needed)
        shape = []
        if len(all_edges[starting_edge]) == 0 or (len(all_edges[starting_edge]) == 1 and all_edges[starting_edge][0] == starting_edge[0]):
            shape = list(starting_edge)
            all_edges[starting_edge].append(starting_edge[1])
        elif len(all_edges[starting_edge]) == 1:
            shape = [starting_edge[1], starting_edge[0]]
            all_edges[starting_edge].append(starting_edge[0])

        # print('\trandom starting edge of', [letter_nodes[n] for n in shape])

        if len(all_edges[starting_edge]) == 2:
            visits_needed.remove(starting_edge)
            # print('\tedge', [letter_nodes[n] for n in shape], 'has been visited twice, removing from visits_needed')

        while shape[-1] != shape[0]:
            # print('\tbranching from node', letter_nodes[shape[-1]])
            smallest_angle = 360
            smallest_angle_point = shape[-2]
            for e in G.edges(shape[-1]):
                e = list(e)
                e.remove(shape[-1])
                angle = calculate_angle(G, shape[-2], shape[-1], e[0])
                # print(f'\t\tangle {letter_nodes[shape[-2]]}-{letter_nodes[shape[-1]]}-{letter_nodes[e[0]]} has measure of {angle} degrees')
                if angle < smallest_angle and angle != 0:
                    smallest_angle = angle
                    smallest_angle_point = e[0]

            edge = (shape[-1], smallest_angle_point)
            # print(f'\tsmallest angle is {letter_nodes[smallest_angle_point]} with measure {smallest_angle}')

            if not (edge in all_edges.keys()):
                edge = (smallest_angle_point, shape[-1])

            shape.append(smallest_angle_point)

            # print(f'\tnext edge is {[letter_nodes[n] for n in edge]}, shape is {[letter_nodes[n] for n in shape]}')
            
            all_edges[edge].append(smallest_angle_point)
            # print('\tupdated node in dictionary')
            if len(all_edges[edge]) == 2:
                # print('\tremoved from needed list')
                visits_needed.remove(edge)
            if len(all_edges[edge]) > 2:
                shape = []
                break

            # print('\t-----')

        all_shapes.append(shape)
        # print(f'added shape {[letter_nodes[n] for n in shape]} to list')
    # print([[letter_nodes[n] for n in shape] for shape in all_shapes])

    fig, ax = ox.plot_graph(tG, edge_linewidth=3, node_size=0, show=False, close=False)
    osthingy = ox.graph_to_gdfs(tG, nodes=True, edges=True)
    for index, row in osthingy[0].iterrows():
        text = letter_nodes[index]
        ax.annotate(text, (row['x'], row['y']), c='y')
        
    plt.show()

    # print(all_shapes)
    return all_shapes

def get_cycles(coordinates):
    polygon = Polygon(coordinates)
    G = ox.graph_from_polygon(polygon, network_type="all_public")
    undi_G = nx.Graph(G)
    cycles = []
    cycles = get_polygons(undi_G, G)

    shapes_coordinates = []

    # ox.plot_graph(G)

    for shape in cycles:
        if len(shape) <= 3:
            continue
        shape_coordinates = []
        shape_x = []
        shape_y = []
        for intersection_numb in range(len(shape) - 1):
            shape_coordinates.append([G.nodes[shape[intersection_numb]]['x'],G.nodes[shape[intersection_numb]]['y']])
            shape_x.append(G.nodes[shape[intersection_numb]]['x'])
            shape_y.append(G.nodes[shape[intersection_numb]]['y'])
            if shape[intersection_numb + 1] in G[shape[intersection_numb]] and 'geometry' in G[shape[intersection_numb]][shape[intersection_numb + 1]][0]:
                xx, yy = G[shape[intersection_numb]][shape[intersection_numb + 1]][0]['geometry'].coords.xy
                for coord in zip(xx, yy):
                    shape_coordinates.append(list(coord))
                    shape_x.append(coord[0])
                    shape_y.append(coord[1])
        # plt.scatter(shape_x, shape_y)
        # plt.plot(shape_x, shape_y)
        # plt.show()
        shapes_coordinates.append(shape_coordinates)

    for shape in shapes_coordinates:
        nodes_outside = 0
        nodes_inside = 0
        for node in undi_G.nodes:
            point = Point(G.nodes[node]['x'], G.nodes[node]['y'])
            polygon = Polygon(shape)
            if polygon.contains_properly(point):
                nodes_inside += 1
            else:
                nodes_outside += 1
        if nodes_inside > 0:
            shapes_coordinates.remove(shape)

    return shapes_coordinates
            

@app.route('/get_polygons/<string_coordinates>')
def main(string_coordinates):
    coordinates = json.loads(string_coordinates)

    shapes_coordinates = get_cycles(coordinates)

    return jsonify(shapes_coordinates)

if __name__ == "__main__":
    app.run(host='0.0.0.0')