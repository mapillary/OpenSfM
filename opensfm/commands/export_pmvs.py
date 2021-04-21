from opensfm.actions import export_pmvs

from . import command


class Command(command.CommandBase):
    name = "export_pmvs"
    help = "Export reconstruction to PMVS"

    def run_impl(self, dataset, args):
        export_pmvs.run_dataset(
            dataset, args.points, args.image_list, args.output, args.undistorted
        )

    def add_arguments_impl(self, parser):
        parser.add_argument("--points", action="store_true", help="export points")
        parser.add_argument(
            "--image_list",
            type=str,
            help="Export only the shots included in this file (path to .txt file)",
        )
        parser.add_argument("--output", help="output pmvs directory")
        parser.add_argument(
            "--undistorted",
            action="store_true",
            help="export the undistorted reconstruction",
        )
