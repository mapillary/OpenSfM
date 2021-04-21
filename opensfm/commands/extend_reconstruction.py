from opensfm.actions import extend_reconstruction

from . import command


class Command(command.CommandBase):
    name = "extend_reconstruction"
    help = "Extend a reconstruction"

    def run_impl(self, dataset, args):
        extend_reconstruction.run_dataset(dataset, args.input, args.output)

    def add_arguments_impl(self, parser):
        parser.add_argument("--input", help="file name of the prior reconstruction")
        parser.add_argument(
            "--output", help="file name where to store the reconstruction"
        )
