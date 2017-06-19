import numpy as np

from opensfm.large import tools


def test_connected_reconstructions():
    reconstruction_shots = {}
    reconstruction_shots["r1"] = { "s1": True }
    reconstruction_shots["r2"] = { "s2": True }

    connections = tools.connected_reconstructions(reconstruction_shots)

    assert len(connections) == 0

    reconstruction_shots = {}
    reconstruction_shots["r1"] = { "s1": True }
    reconstruction_shots["r2"] = { "s1": True }

    connections = tools.connected_reconstructions(reconstruction_shots)

    assert len(connections) == 1
    assert "r1" in connections[0]
    assert "r2" in connections[0]

    reconstruction_shots = {}
    reconstruction_shots["r1"] = { "s1": True }
    reconstruction_shots["r2"] = { "s1": True }
    reconstruction_shots["r3"] = { "s1": True }

    connections = tools.connected_reconstructions(reconstruction_shots)
    connections = [tuple(sorted(list(c))) for c in connections]
    connections.sort(key=lambda c: c[0] + c[1])

    assert len(connections) == 3
    assert ("r1", "r2") in connections
    assert ("r1", "r3") in connections
    assert ("r2", "r3") in connections


def test_corresponding_tracks():
    t1 = { 1: { "feature_id": 1 } }
    t2 = { 1: { "feature_id": 2 } }

    correspondences = tools.corresponding_tracks(t1, t2)
    assert len(correspondences) == 0

    t1 = { 1: { "feature_id": 3 } }
    t2 = { 2: { "feature_id": 3 } }

    correspondences = tools.corresponding_tracks(t1, t2)
    assert len(correspondences) == 1
    assert correspondences[0] == (1, 2)

    t1 = { 1: { "feature_id": 3 }, 2: { "feature_id": 4 } }
    t2 = { 1: { "feature_id": 4 }, 2: { "feature_id": 5 } }

    correspondences = tools.corresponding_tracks(t1, t2)
    assert len(correspondences) == 1
    assert correspondences[0] == (2, 1)

    t1 = { 1: { "feature_id": 5 }, 2: { "feature_id": 6 } }
    t2 = { 3: { "feature_id": 5 }, 4: { "feature_id": 6 } }

    correspondences = tools.corresponding_tracks(t1, t2)
    correspondences.sort(key=lambda c: c[0] + c[1])
    assert len(correspondences) == 2
    assert correspondences[0] == (1, 3)
    assert correspondences[1] == (2, 4)
