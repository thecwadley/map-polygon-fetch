import osmnx as ox
from shapely.geometry import Polygon
import networkx as nx
from flask import Flask, jsonify
from flask_cors import CORS
import json
import matplotlib.pyplot as plt

coordinates_complex=[[-92.67135532191914,37.685714457810896],[-92.65835579781118,37.67589605069125],[-92.64917989981242,37.683392595609675],[-92.65050640456855,37.68554125776477],[-92.65804359477826,37.685652539534445],[-92.65826219481298,37.68672683962893],[-92.6547667125931,37.68665321166851],[-92.65490713233096,37.69007348793363],[-92.65557782174392,37.69293895512361],[-92.65897970096512,37.695841102943604],[-92.66447289550727,37.69243280231237],[-92.66829600346317,37.69213580433914],[-92.6681555837253,37.68876415238195],[-92.67088675829456,37.68661558359242]]
coordinates_simple=[[-92.66580862851671,37.68859125435237],[-92.66421590806539,37.688533866930456],[-92.66417738023868,37.68557914389185],[-92.66569728083961,37.685656967277154]]
coordinates_bendy=[[-92.66344494593348,37.68685645231407],[-92.66222902378017,37.68681237584109],[-92.66237349790343,37.688183096392855],[-92.66263898720273,37.68859303374913],[-92.66339893854916,37.68839142313682]]

app = Flask(__name__)
CORS(app)

def get_cycles(coordinates):
    polygon = Polygon(coordinates)
    G = ox.graph_from_polygon(polygon, network_type="all_public")
    undi_G = nx.Graph(G)
    cycles = nx.cycle_basis(undi_G)

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

# coordinates_broke = [[37.68837090098686,-92.66874914438237],[37.686024024297225,-92.67177656588937],[37.68278847932924,-92.66737583812692],[37.685184822108695,-92.6647074211527]]
# get_cycles(coordinates_broke)