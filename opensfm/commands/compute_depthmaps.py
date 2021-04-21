from opensfm.actions import compute_depthmaps

from . import command


class Command(command.CommandBase):
    name = "compute_depthmaps"
    help = "Compute depthmap"

    def run_impl(self, dataset, args):
        compute_depthmaps.run_dataset(dataset, args.subfolder, args.interactive)

    def add_arguments_impl(self, parser):
        parser.add_argument(
            "--subfolder",
            help="undistorted subfolder where to load and store data",
            default="undistorted",
        )
        parser.add_argument(
            "--interactive",
            help="plot results as they are being computed",
            action="store_true",
        )
