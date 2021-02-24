from opensfm.actions import export_ply

from . import command


class Command(command.CommandBase):
    name = "export_ply"
    help = "Export reconstruction to PLY format"

    def run_impl(self, dataset, args):
        export_ply.run_dataset(dataset, args.no_cameras, args.no_points, args.depthmaps, args.point_num_views)

    def add_arguments_impl(self, parser):
        parser.add_argument(
            "--no-cameras",
            action="store_true",
            default=False,
            help="Do not save camera positions",
        )
        parser.add_argument(
            "--no-points", action="store_true", default=False, help="Do not save points"
        )
        parser.add_argument(
            "--depthmaps",
            action="store_true",
            default=False,
            help="Export per-image depthmaps as pointclouds",
        )
        parser.add_argument(
            "--point-num-views",
            action="store_true",
            default=False,
            help="Export the number of observations associated with each point"
        )
