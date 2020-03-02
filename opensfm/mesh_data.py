import logging
import time

from opensfm import dataset
from opensfm import mesh
from opensfm import types

logger = logging.getLogger(__name__)


def run(data):
    start = time.time()
    graph = data.track_graph_of_images
    reconstructions = data.reconstructions

    for i, r in enumerate(reconstructions):
        for shot in r.shots.values():
            if shot.id in graph:
                vertices, faces = mesh.triangle_mesh(shot.id, r, graph,
                                                     data)
                shot.mesh = types.ShotMesh()
                shot.mesh.vertices = vertices
                shot.mesh.faces = faces

    data.save_reconstruction(reconstructions,
                             filename='reconstruction.meshed.json',
                             minify=True)

    end = time.time()
    with open(data.profile_log(), 'a') as fout:
        fout.write('mesh: {0}\n'.format(end - start))
