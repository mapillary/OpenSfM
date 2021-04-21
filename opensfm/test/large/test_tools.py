import numpy as np
from opensfm.large import tools


def test_connected_reconstructions():
    reconstruction_shots = {}
    reconstruction_shots["r1"] = {"s1": True}
    reconstruction_shots["r2"] = {"s2": True}

    connections = tools.connected_reconstructions(reconstruction_shots)

    assert len(connections) == 0

    reconstruction_shots = {}
    reconstruction_shots["r1"] = {"s1": True}
    reconstruction_shots["r2"] = {"s1": True}

    connections = tools.connected_reconstructions(reconstruction_shots)

    assert len(connections) == 1
    assert ("r1", "r2") in connections

    reconstruction_shots = {}
    reconstruction_shots["r1"] = {"s1": True}
    reconstruction_shots["r2"] = {"s1": True}
    reconstruction_shots["r3"] = {"s1": True}

    connections = tools.connected_reconstructions(reconstruction_shots)
    connections = [tuple(sorted(list(c))) for c in connections]
    connections.sort(key=lambda c: c[0] + c[1])

    assert len(connections) == 3
    assert ("r1", "r2") in connections
    assert ("r1", "r3") in connections
    assert ("r2", "r3") in connections
