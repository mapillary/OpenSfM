from opensfm.actions import align_submodels

from . import command


class Command(command.CommandBase):
    name = "align_submodels"
    help = "Align submodel reconstructions"

    def run_impl(self, dataset, args):
        align_submodels.run_dataset(dataset)

    def add_arguments_impl(self, parser):
        pass
