from opensfm.actions import undistort

from . import command


class Command(command.CommandBase):
    name = "undistort"
    help = "Save radially undistorted images"

    def run_impl(self, dataset, args):
        undistort.run_dataset(
            dataset,
            args.reconstruction,
            args.reconstruction_index,
            args.tracks,
            args.output,
            args.skip_images
        )

    def add_arguments_impl(self, parser):
        parser.add_argument(
            "--reconstruction",
            help="reconstruction to undistort",
            type=str
        )
        parser.add_argument(
            "--reconstruction-index",
            help="index of the reconstruction component to undistort",
            type=int,
            default=0,
        )
        parser.add_argument(
            "--tracks",
            help="tracks graph of the reconstruction",
        )
        parser.add_argument(
            "--output",
            help="output folder",
            default="undistorted",
            type=str
        )
        parser.add_argument(
            "--skip-images",
            help="if set, only undistort the reconstruction and NOT the images.",
            action="store_true",
        )
