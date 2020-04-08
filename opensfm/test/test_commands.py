import argparse

from opensfm import commands
from opensfm.test import data_generation


def run_command(command, args):
    parser = argparse.ArgumentParser()
    command.add_arguments(parser)
    parsed_args = parser.parse_args(args)
    command.run(parsed_args)


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
        run_command(command, [data.data_path])

    reconstruction = data.load_reconstruction()
    assert len(reconstruction[0].shots) == 3
    assert len(reconstruction[0].points) > 1000
