from opensfm import io, reconstruction, types
from opensfm.dataset import DataSetBase

def run_dataset(data: DataSetBase, input, output):
    recs_base = data.load_reconstruction(input)
    if len(recs_base) == 0:
        return

    rec_base = recs_base[0]
    tracks_manager = data.load_tracks_manager()
    rec_base.add_correspondences_from_tracks_manager(tracks_manager)

    images = data.images()
    remaining_images = set(images) - set(rec_base.shots)
    gcp = data.load_ground_control_points()
    report = {}
    rec_report = {}
    report["extend_reconstruction"] = [rec_report]
    rec, rec_report["grow"] = reconstruction.grow_reconstruction(
        data,
        tracks_manager,
        rec_base,
        remaining_images,
        gcp,
    )
    rec_report["num_remaining_images"] = len(remaining_images)
    report["not_reconstructed_images"] = list(remaining_images)
    data.save_reconstruction([rec], output)
    data.save_report(io.json_dumps(report), "reconstruction.json")
