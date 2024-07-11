import osmnx as ox
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import networkx as nx
from flask import Flask, jsonify
from flask_cors import CORS
import json
import matplotlib.pyplot as plt
import random
# import copy
import numpy as np

def calculate_angle(G, a, b, c):
    a = np.array([G.nodes[a]['x'], G.nodes[a]['y']])  # First
    b = np.array([G.nodes[b]['x'], G.nodes[b]['y']])  # Mid
    c = np.array([G.nodes[c]['x'], G.nodes[c]['y']])  # End

    radians = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
    if radians < 0:
        radians = radians + (2 * np.pi)
    angle = np.abs(radians * 180.0 / np.pi)

    # if angle > 180.0:
    #     angle = 360 - angle

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
            # print('\tbranching from point', letter_nodes[shape[-1]])
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

            path = (shape[-1], smallest_angle_point)
            # print(f'\tsmallest angle is {letter_nodes[smallest_angle_point]} with measure {smallest_angle}')

            if not (path in all_edges.keys()):
                path = (smallest_angle_point, shape[-1])

            shape.append(smallest_angle_point)

            # print(f'\tnext edge is {[letter_nodes[n] for n in path]}, shape is {[letter_nodes[n] for n in shape]}')
            
            all_edges[path].append(smallest_angle_point)
            # print('\tupdated point in dictionary')
            if len(all_edges[path]) == 2:
                # print('\tremoved from needed list')
                visits_needed.remove(path)
            if len(all_edges[path]) > 2:
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
    # c = edge['geometry'].centroid
        ax.annotate(text, (row['x'], row['y']), c='y')
        
    plt.show()

    # print(all_shapes)
    return all_shapes


    # make list of all edges
    # pick random starting edge
    # go to the rightmost node
    # mark edge as visited from node start to node end
    # mark edge in dictionary as visited
    # once rightmost edge equals starting edge, define as shape
    # pick new edge

coordinates_complex=[[-92.67135532191914,37.685714457810896],[-92.65835579781118,37.67589605069125],[-92.64917989981242,37.683392595609675],[-92.65050640456855,37.68554125776477],[-92.65804359477826,37.685652539534445],[-92.65826219481298,37.68672683962893],[-92.6547667125931,37.68665321166851],[-92.65490713233096,37.69007348793363],[-92.65557782174392,37.69293895512361],[-92.65897970096512,37.695841102943604],[-92.66447289550727,37.69243280231237],[-92.66829600346317,37.69213580433914],[-92.6681555837253,37.68876415238195],[-92.67088675829456,37.68661558359242]]
coordinates_simple=[[-92.66580862851671,37.68859125435237],[-92.66421590806539,37.688533866930456],[-92.66417738023868,37.68557914389185],[-92.66569728083961,37.685656967277154]]
coordinates_bendy=[[-92.66344494593348,37.68685645231407],[-92.66222902378017,37.68681237584109],[-92.66237349790343,37.688183096392855],[-92.66263898720273,37.68859303374913],[-92.66339893854916,37.68839142313682]]

app = Flask(__name__)
CORS(app)

def get_cycles(coordinates):
    polygon = Polygon(coordinates)
    G = ox.graph_from_polygon(polygon, network_type="all_public")
    undi_G = nx.Graph(G)
    # cycles_generator = nx.chordless_cycles(undi_G)
    # cycles = nx.cycle_basis(undi_G)
    # letter_cycles = copy.deepcopy(cycles)
    cycles = []
    cycles = get_polygons(undi_G, G)
    # for cycle in cycles_generator:
    #     cycles.append(cycle)

    # key = dict()
    # values = dict()
    # letters = ['A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z']
    # current_letter = 0
    # for shape_index in range(len(cycles)):
    #     for point_index in range(len(cycles[shape_index])):
    #         if not cycles[shape_index][point_index] in key.keys():
    #             key[cycles[shape_index][point_index]] = letters[current_letter]
    #             values[letters[current_letter]] = cycles[shape_index][point_index]
    #             current_letter += 1
    #         letter_cycles[shape_index][point_index] = key[cycles[shape_index][point_index]]

    # print(letter_cycles)
            

    shapes_coordinates = []

    # ox.plot_graph(G)

    for shape in cycles:
        if len(shape) == 0:
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
        # print('shape length', len(shape),':')
        falses = 0
        trues = 0
        for node in undi_G.nodes:
            point = Point(G.nodes[node]['x'], G.nodes[node]['y'])
            polygon = Polygon(shape)
            if polygon.contains_properly(point):
                trues += 1
            else:
                falses += 1
        # print('falses:', falses)
        # print('trues: ', trues)
        if trues > 0:
            # print("LOLLLLOLOLLLOLLOOLLLLLL")
            shapes_coordinates.remove(shape)

    return shapes_coordinates
            

@app.route('/get_polygons/<string_coordinates>')
def main(string_coordinates):
    coordinates = json.loads(string_coordinates)

    shapes_coordinates = get_cycles(coordinates)

    return jsonify(shapes_coordinates)

if __name__ == "__main__":
    app.run(host='0.0.0.0')

# coordinates_broke = [[-92.66319338565718,37.693064308591204],[-92.66141439528823,37.69114112804267],[-92.65733757855429,37.69388564650602],[-92.65908531966421,37.695617388673746]]
# print(get_cycles(coordinates_broke))