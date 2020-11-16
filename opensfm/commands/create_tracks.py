from opensfm.actions import create_tracks

from . import command


class Command(command.CommandBase):
    name = "create_tracks"
    help = "Link matches pair-wise matches into tracks"

    def run_impl(self, dataset, args):
        create_tracks.run_dataset(dataset)

    def add_arguments_impl(self, parser):
        pass
