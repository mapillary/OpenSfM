from opensfm import mesh
from opensfm.dataset import DataSetBase


def run_dataset(data: DataSetBase):
    """ Add delaunay meshes to the reconstruction. """

    tracks_manager = data.load_tracks_manager()
    reconstructions = data.load_reconstruction()

    all_shot_ids = set(tracks_manager.get_shot_ids())
    for r in reconstructions:
        for shot in r.shots.values():
            if shot.id in all_shot_ids:
                vertices, faces = mesh.triangle_mesh(shot.id, r, tracks_manager)
                shot.mesh.vertices = vertices
                shot.mesh.faces = faces

    data.save_reconstruction(
        reconstructions, filename="reconstruction.meshed.json", minify=True
    )
