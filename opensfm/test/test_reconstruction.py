import networkx as nx

from opensfm import types
import opensfm.reconstruction


def _add_shot(graph, reconstruction, shot_id):
    graph.add_node(shot_id, bipartite=0)
    shot = types.Shot()
    shot.id = shot_id
    reconstruction.add_shot(shot)


def _add_point(graph, reconstruction, point_id, observations):
    graph.add_node(point_id, bipartite=1)
    for shot_id in observations:
        graph.add_edge(shot_id, point_id)
    point = types.Point()
    point.id = point_id
    reconstruction.add_point(point)


def test_shot_neighborhood_linear_graph():
    graph = nx.Graph()
    reconstruction = types.Reconstruction()
    _add_shot(graph, reconstruction, 'im0')
    for i in range(1, 4):
        p, n = 'im' + str(i - 1), 'im' + str(i)
        _add_shot(graph, reconstruction, n)
        _add_point(graph, reconstruction, str(i), [p, n])

    interior, boundary = opensfm.reconstruction.shot_neighborhood(
        graph, reconstruction, 'im2',
        radius=1, min_common_points=1, max_interior_size=10)
    assert interior == set(['im2'])
    assert boundary == set(['im1', 'im3'])

    interior, boundary = opensfm.reconstruction.shot_neighborhood(
        graph, reconstruction, 'im2',
        radius=2, min_common_points=1, max_interior_size=10)
    assert interior == set(['im1', 'im2', 'im3'])
    assert boundary == set(['im0'])

    interior, boundary = opensfm.reconstruction.shot_neighborhood(
        graph, reconstruction, 'im2',
        radius=3, min_common_points=1, max_interior_size=10)
    assert interior == set(['im0', 'im1', 'im2', 'im3'])
    assert boundary == set()

    interior, boundary = opensfm.reconstruction.shot_neighborhood(
        graph, reconstruction, 'im2',
        radius=3, min_common_points=1, max_interior_size=3)
    assert interior == set(['im1', 'im2', 'im3'])
    assert boundary == set(['im0'])


def test_shot_neighborhood_complete_graph():
    graph = nx.Graph()
    reconstruction = types.Reconstruction()
    for i in range(4):
        _add_shot(graph, reconstruction, 'im' + str(i))
    _add_point(graph, reconstruction, '1', reconstruction.shots.keys())

    interior, boundary = opensfm.reconstruction.shot_neighborhood(
        graph, reconstruction, 'im2',
        radius=2, min_common_points=1, max_interior_size=10)
    assert interior == set(['im0', 'im1', 'im2', 'im3'])
    assert boundary == set()


def test_shot_neighborhood_sorted_results():
    graph = nx.Graph()
    reconstruction = types.Reconstruction()
    _add_shot(graph, reconstruction, 'im0')
    _add_shot(graph, reconstruction, 'im1')
    _add_shot(graph, reconstruction, 'im2')
    _add_point(graph, reconstruction, '1', ['im0', 'im1'])
    _add_point(graph, reconstruction, '2', ['im0', 'im1'])
    _add_point(graph, reconstruction, '3', ['im0', 'im2'])

    interior, boundary = opensfm.reconstruction.shot_neighborhood(
        graph, reconstruction, 'im0',
        radius=2, min_common_points=1, max_interior_size=2)
    assert interior == set(['im0', 'im1'])
    assert boundary == set(['im2'])

    _add_point(graph, reconstruction, '4', ['im0', 'im2'])
    _add_point(graph, reconstruction, '5', ['im0', 'im2'])

    interior, boundary = opensfm.reconstruction.shot_neighborhood(
        graph, reconstruction, 'im0',
        radius=2, min_common_points=1, max_interior_size=2)
    assert interior == set(['im0', 'im2'])
    assert boundary == set(['im1'])
