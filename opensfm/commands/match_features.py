from opensfm.actions import match_features

from . import command


class Command(command.CommandBase):
    name = "match_features"
    help = "Match features between image pairs"

    def run_impl(self, dataset, args):
        match_features.run_dataset(dataset)

    def add_arguments_impl(self, parser):
        pass
