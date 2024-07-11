import osmnx as ox
from shapely.geometry import Polygon
import networkx as nx
from flask import Flask, jsonify
from flask_cors import CORS
import json
import matplotlib.pyplot as plt
# import copy

coordinates_complex=[[-92.67135532191914,37.685714457810896],[-92.65835579781118,37.67589605069125],[-92.64917989981242,37.683392595609675],[-92.65050640456855,37.68554125776477],[-92.65804359477826,37.685652539534445],[-92.65826219481298,37.68672683962893],[-92.6547667125931,37.68665321166851],[-92.65490713233096,37.69007348793363],[-92.65557782174392,37.69293895512361],[-92.65897970096512,37.695841102943604],[-92.66447289550727,37.69243280231237],[-92.66829600346317,37.69213580433914],[-92.6681555837253,37.68876415238195],[-92.67088675829456,37.68661558359242]]
coordinates_simple=[[-92.66580862851671,37.68859125435237],[-92.66421590806539,37.688533866930456],[-92.66417738023868,37.68557914389185],[-92.66569728083961,37.685656967277154]]
coordinates_bendy=[[-92.66344494593348,37.68685645231407],[-92.66222902378017,37.68681237584109],[-92.66237349790343,37.688183096392855],[-92.66263898720273,37.68859303374913],[-92.66339893854916,37.68839142313682]]

app = Flask(__name__)
CORS(app)

def get_cycles(coordinates):
    polygon = Polygon(coordinates)
    G = ox.graph_from_polygon(polygon, network_type="all_public")
    undi_G = nx.Graph(G)
    cycles_generator = nx.chordless_cycles(undi_G)
    # cycles = nx.cycle_basis(undi_G)
    # letter_cycles = copy.deepcopy(cycles)
    cycles = []
    for cycle in cycles_generator:
        cycles.append(cycle)

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
        shape_coordinates = []
        shape_x = []
        shape_y = []
        shape.append(shape[0])
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