from . import command
from opensfm.actions import compute_statistics


class Command(command.CommandBase):
    name = "compute_statistics"
    help = "Compute statistics and save them in the stats folder"

    def run_impl(self, dataset, args):
        compute_statistics.run_dataset(dataset)

    def add_arguments_impl(self, parser):
        pass
