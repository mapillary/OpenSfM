from opensfm.dataset import DataSet

class CommandBase:
    """ Base class for executable commands."""
    name = "Undefined command"
    help = "Undefined command help"

    def run(self, args):
        self.run_impl(DataSet(args.dataset), args)

    def add_arguments(self, parser):
        parser.add_argument('dataset', help='dataset to process')
        self.add_arguments_impl(parser)

    def run_impl(self, dataset, args):
        raise NotImplementedError("Command " + self.name + " not implemented")

    def add_arguments_impl(self, parser):
        raise NotImplementedError("Command " + self.name + " not implemented")
