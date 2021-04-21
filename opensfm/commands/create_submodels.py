from opensfm.actions import create_submodels

from . import command


class Command(command.CommandBase):
    name = "create_submodels"
    help = "Split the dataset into smaller submodels"

    def run_impl(self, dataset, args):
        create_submodels.run_dataset(dataset)

    def add_arguments_impl(self, parser):
        pass
