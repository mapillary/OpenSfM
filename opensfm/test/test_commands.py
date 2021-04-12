import argparse
from os.path import join

from opensfm import commands, dataset
from opensfm.test import data_generation, utils


def run_command(command, args):
    parser = argparse.ArgumentParser()
    command.add_arguments(parser)
    parsed_args = parser.parse_args(args)
    command.run(dataset.DataSet(parsed_args.dataset), parsed_args)


def test_run_all(tmpdir):
    data = data_generation.create_berlin_test_folder(tmpdir)
    run_all_commands = [
        commands.extract_metadata,
        commands.detect_features,
        commands.match_features,
        commands.create_tracks,
        commands.reconstruct,
        commands.bundle,
        commands.reconstruct_from_prior,
        commands.mesh,
        commands.undistort,
        commands.compute_depthmaps,
        commands.export_ply,
        commands.export_visualsfm,
        commands.export_openmvs,
        commands.export_pmvs,
        commands.export_bundler,
        commands.export_colmap,
        commands.compute_statistics,
        commands.export_report,
    ]

    output_rec_path = join(data.data_path, "rec_prior.json")
    command_options = {
        commands.reconstruct_from_prior: [
            "--input",
            join(data.data_path, "reconstruction.json"),
            "--output",
            output_rec_path,
        ]
    }

    for module in run_all_commands:
        command = module.Command()
        options = command_options.get(module, [])
        run_command(command, [data.data_path] + options)

    check_reconstruction(data)
    check_prior(data, output_rec_path)


def check_reconstruction(data: dataset.DataSet):
    reconstruction = data.load_reconstruction()
    assert len(reconstruction[0].shots) == 3
    assert len(reconstruction[0].points) > 1000


def check_prior(data: dataset.DataSet, output_rec_path: str):
    reconstruction = data.load_reconstruction()  # load old reconstruction
    prior_rec = data.load_reconstruction(output_rec_path)
    for shot_id, shot in reconstruction[0].shots.items():
        utils.assert_shots_equal(shot, prior_rec[0].shots[shot_id])

    assert len(prior_rec[0].points) > 1000
