from opensfm import commands
from opensfm.test import data_generation


def test_run_all(tmpdir):
    data = data_generation.create_berlin_test_folder(tmpdir)
    run_all_commands = [
        commands.extract_metadata,
        commands.detect_features,
        commands.match_features,
        commands.create_tracks,
        commands.reconstruct,
        commands.bundle,
        commands.mesh,
        commands.undistort,
        commands.compute_depthmaps,
        commands.export_ply,
        commands.export_visualsfm,
        commands.export_openmvs,
        commands.export_pmvs,
        commands.export_bundler,
        commands.export_colmap
    ]

    for module in run_all_commands:
        command = module.Command()
        command.run_dataset(command.options_type(), data)

    reconstruction = data.load_reconstruction()
    assert len(reconstruction[0].shots) == 3
    assert len(reconstruction[0].points) > 1000
