from . import command
from opensfm.actions import export_report


class Command(command.CommandBase):
    name = "export_report"
    help = "Export a nice report based on previously generated statistics"

    def run_impl(self, dataset, args):
        export_report.run_dataset(dataset)

    def add_arguments_impl(self, parser):
        pass
