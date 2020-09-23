class CommandBase:
    """ Base class for executable commands taking a dataset's path
    plus additionnal arguments parsed as options.

    The core body should only take a dataset as input for easier re-use.
    """

    def __init__(self):
        self.name = "Undefined command"
        self.help = "Undefined command help"
        self.args = {}
        self.args["dataset"] = {"help": "dataset to process"}

    def add_arguments(self, parser):
        for name, options in self.args.items():
            final_options = {}
            for options_args in ["help", "choices", "type", "nargs", "action", "required", "default"]:
                if options_args in options:
                    final_options[options_args] = options[options_args]
            parser.add_argument(name, **final_options)

    def options_type(self):
        cleaned = []
        dummy_args = {}
        for s, options in self.args.items():
            if s.startswith('--'):
                s = s[2:]
            s = s.replace('-', '_')
            cleaned.append(s)
            v = options["default"] if "default" in options else None
            dummy_args[s] = v

        options_class = type(self.name + '_Options', (object,), {})
        options_object = options_class
        for s in cleaned:
            setattr(options_object, s, dummy_args[s])
        return options_object

    def run_dataset(self, options, dataset):
        raise NotImplementedError("Command " + self.name + " not implemented")
