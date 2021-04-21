from opensfm.actions import detect_features

from . import command


class Command(command.CommandBase):
    name = "detect_features"
    help = "Compute features for all images"

    def run_impl(self, dataset, args):
        detect_features.run_dataset(dataset)

    def add_arguments_impl(self, parser):
        pass
