import networkx as nx

from opensfm import types
import opensfm.reconstruction


def test_shot_neighborhood_linear_graph():
    graph = nx.Graph()
    reconstruction = types.Reconstruction()
    graph.add_node('im0', bipartite=0)
    shot = types.Shot()
    shot.id = 'im0'
    reconstruction.add_shot(shot)
    for i in range(1, 4):
        shot = types.Shot()
        shot.id = 'im' + str(i)
        point = types.Point()
        point.id = str(i)
        prev_shot_id = 'im' + str(i - 1)
        reconstruction.add_shot(shot)
        reconstruction.add_point(point)
        graph.add_node(shot.id, bipartite=0)
        graph.add_node(point.id, bipartite=1)
        graph.add_edge(shot.id, point.id)
        graph.add_edge(prev_shot_id, point.id)

    interior, boundary = opensfm.reconstruction.shot_neighborhood(
        graph, reconstruction, 'im2', 1)
    assert interior == set(['im2'])
    assert boundary == set(['im1', 'im3'])

    interior, boundary = opensfm.reconstruction.shot_neighborhood(
        graph, reconstruction, 'im2', 2)
    assert interior == set(['im1', 'im2', 'im3'])
    assert boundary == set(['im0'])

    interior, boundary = opensfm.reconstruction.shot_neighborhood(
        graph, reconstruction, 'im2', 3)
    assert interior == set(['im0', 'im1', 'im2', 'im3'])
    assert boundary == set()


def test_shot_neighborhood_complete_graph():
    graph = nx.Graph()
    reconstruction = types.Reconstruction()

    point = types.Point()
    point.id = '1'
    reconstruction.add_point(point)
    graph.add_node(point.id, bipartite=1)

    for i in range(4):
        shot = types.Shot()
        shot.id = 'im' + str(i)
        reconstruction.add_shot(shot)
        graph.add_node(shot.id, bipartite=0)
        graph.add_edge(shot.id, point.id)

    interior, boundary = opensfm.reconstruction.shot_neighborhood(
        graph, reconstruction, 'im2', 2)
    assert interior == set(['im0', 'im1', 'im2', 'im3'])
    assert boundary == set()
