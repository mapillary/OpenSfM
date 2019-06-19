import networkx as nx

from opensfm import types
from opensfm import reconstruction


def _add_shot(graph, rec, shot_id):
    graph.add_node(shot_id, bipartite=0)
    shot = types.Shot()
    shot.id = shot_id
    rec.add_shot(shot)


def _add_point(graph, rec, point_id, observations):
    graph.add_node(point_id, bipartite=1)
    for shot_id in observations:
        graph.add_edge(shot_id, point_id)
    point = types.Point()
    point.id = point_id
    rec.add_point(point)


def test_shot_neighborhood_linear_graph():
    graph = nx.Graph()
    rec = types.Reconstruction()
    _add_shot(graph, rec, 'im0')
    for i in range(1, 4):
        p, n = 'im' + str(i - 1), 'im' + str(i)
        _add_shot(graph, rec, n)
        _add_point(graph, rec, str(i), [p, n])

    interior, boundary = reconstruction.shot_neighborhood(
        graph, rec, 'im2',
        radius=1, min_common_points=1, max_interior_size=10)
    assert interior == set(['im2'])
    assert boundary == set(['im1', 'im3'])

    interior, boundary = reconstruction.shot_neighborhood(
        graph, rec, 'im2',
        radius=2, min_common_points=1, max_interior_size=10)
    assert interior == set(['im1', 'im2', 'im3'])
    assert boundary == set(['im0'])

    interior, boundary = reconstruction.shot_neighborhood(
        graph, rec, 'im2',
        radius=3, min_common_points=1, max_interior_size=10)
    assert interior == set(['im0', 'im1', 'im2', 'im3'])
    assert boundary == set()

    interior, boundary = reconstruction.shot_neighborhood(
        graph, rec, 'im2',
        radius=3, min_common_points=1, max_interior_size=3)
    assert interior == set(['im1', 'im2', 'im3'])
    assert boundary == set(['im0'])


def test_shot_neighborhood_complete_graph():
    graph = nx.Graph()
    rec = types.Reconstruction()
    for i in range(4):
        _add_shot(graph, rec, 'im' + str(i))
    _add_point(graph, rec, '1', rec.shots.keys())

    interior, boundary = reconstruction.shot_neighborhood(
        graph, rec, 'im2',
        radius=2, min_common_points=1, max_interior_size=10)
    assert interior == set(['im0', 'im1', 'im2', 'im3'])
    assert boundary == set()


def test_shot_neighborhood_sorted_results():
    graph = nx.Graph()
    rec = types.Reconstruction()
    _add_shot(graph, rec, 'im0')
    _add_shot(graph, rec, 'im1')
    _add_shot(graph, rec, 'im2')
    _add_point(graph, rec, '1', ['im0', 'im1'])
    _add_point(graph, rec, '2', ['im0', 'im1'])
    _add_point(graph, rec, '3', ['im0', 'im2'])

    interior, boundary = reconstruction.shot_neighborhood(
        graph, rec, 'im0',
        radius=2, min_common_points=1, max_interior_size=2)
    assert interior == set(['im0', 'im1'])
    assert boundary == set(['im2'])

    _add_point(graph, rec, '4', ['im0', 'im2'])
    _add_point(graph, rec, '5', ['im0', 'im2'])

    interior, boundary = reconstruction.shot_neighborhood(
        graph, rec, 'im0',
        radius=2, min_common_points=1, max_interior_size=2)
    assert interior == set(['im0', 'im2'])
    assert boundary == set(['im1'])
