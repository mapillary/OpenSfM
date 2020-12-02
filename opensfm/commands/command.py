from timeit import default_timer as timer


class CommandBase:
    """ Base class for executable commands."""

    name = "Undefined command"
    help = "Undefined command help"

    def run(self, data, args):
        start = timer()
        self.run_impl(data, args)
        end = timer()
        with open(data.profile_log(), "a") as fout:
            fout.write(type(self).name + ": {0}\n".format(end - start))

    def add_arguments(self, parser):
        parser.add_argument("dataset", help="dataset to process")
        self.add_arguments_impl(parser)

    def run_impl(self, dataset, args):
        raise NotImplementedError("Command " + self.name + " not implemented")

    def add_arguments_impl(self, parser):
        raise NotImplementedError("Command " + self.name + " not implemented")
