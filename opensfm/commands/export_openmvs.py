from . import command
from opensfm.actions import export_openmvs


class Command(command.CommandBase):
    name = 'export_openmvs'
    help = "Export reconstruction to openMVS format"

    def run_impl(self, dataset, args):
        export_openmvs.run_dataset(dataset)

    def add_arguments_impl(self, parser):
        pass
