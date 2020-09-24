from os.path import abspath, join, dirname
import sys
sys.path.insert(0, abspath(join(dirname(__file__), "..")))

from opensfm import commands

if __name__ == "__main__":
    commands.command_runner(commands.opensfm_commands)
