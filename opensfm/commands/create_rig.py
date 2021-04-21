import json

from opensfm.actions import create_rig

from . import command


class Command(command.CommandBase):
    name = "create_rig"
    help = "Create rig by creating `rig_cameras.json` and `rig_assignments.json` files."

    def run_impl(self, dataset, args):
        create_rig.run_dataset(dataset, args.method, json.loads(args.definition), True)

    def add_arguments_impl(self, parser):
        parser.add_argument(
            "method",
            help="Method for creating the rigs",
            choices=["auto", "camera", "pattern"],
        )
        parser.add_argument(
            "definition",
            help=(
                "Defines each RigCamera as a JSON string dict with the form `{camera_id: definition, ...}`"
                "For `pattern`, the definition is expected to be a REGEX with the form (.*)"
                "For `camera`, the definition is expected to be a camera model ID"
            ),
        )
