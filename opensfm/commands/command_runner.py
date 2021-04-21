import argparse

from opensfm import log


def command_runner(all_commands_types, dataset_factory, dataset_choices):
    """ Main entry point for running the passed SfM commands types."""
    log.setup()

    # Create the top-level parser
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(
        help="Command to run", dest="command", metavar="command"
    )

    command_objects = [c.Command() for c in all_commands_types]

    for command in command_objects:
        subparser = subparsers.add_parser(command.name, help=command.help)
        command.add_arguments(subparser)
        subparser.add_argument(
            "--dataset-type",
            type=str,
            required=False,
            default="opensfm",
            choices=dataset_choices,
        )

    # Parse arguments
    args = parser.parse_args()

    # Instanciate datast
    data = dataset_factory(args.dataset, args.dataset_type)

    # Run the selected subcommand
    for command in command_objects:
        if args.command == command.name:
            command.run(data, args)
