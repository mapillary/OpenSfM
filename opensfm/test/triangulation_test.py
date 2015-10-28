import numpy as np
import networkx as nx
import opensfm.reconstruction

def triangulate_track_equirectangular_test():
    graph = nx.Graph()
    graph.add_node('im1', bipartite=0)
    graph.add_node('im2', bipartite=0)
    graph.add_node('1', bipartite=1)
    graph.add_edge('im1', '1', feature=(0,0))
    graph.add_edge('im2', '1', feature=(-0.1, 0))

    reconstruction = {
        "cameras": {
            "theta": {
                "projection_type": "equirectangular"
            }
        },

        "shots" : {
            'im1': {
                "camera": "theta",
                "rotation": [0.0, 0.0, 0.0],
                "translation": [0.0, 0.0, 0.0],
            },
            'im2': {
                "camera": "theta",
                "rotation": [0, 0, 0.0],
                "translation": [-1, 0, 0.0],
            },
        },

        "points" : {
        },
    }

    opensfm.reconstruction.triangulate_track_equirectangular(
        '1', graph, reconstruction, {}, 0.01, 2.0)
    assert '1' in reconstruction['points']
    p = reconstruction['points']['1']['coordinates']
    assert np.allclose(p, [0, 0, 1.3763819204711])



if __name__ == "__main__":
    triangulate_track_equirectangular_test()
