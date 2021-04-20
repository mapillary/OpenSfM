import argparse
import os
from typing import List

from flask import (
    Flask,
    abort,
    jsonify,
    Response,
    send_file,
)


app = Flask(__name__, static_folder="./", static_url_path="")


DATA = "data"
IMAGES = "images"


datapath = None


@app.route("/")
def index():
    return send_file(os.path.join(app.static_folder, "index.html"))


@app.route("/items")
def get_recs() -> Response:
    if datapath is None:
        return jsonify({"items": []})

    reconstructions = [
        {
            "children": [],
            "name": rec,
            "type": "RECONSTRUCTION",
            "url": [os.path.join(DATA, rec)],
        }
        for rec in reconstruction_files(datapath)
    ]

    return jsonify({"items": reconstructions})


@app.route("/data/<path:subpath>")
def get_data(subpath) -> Response:
    path = os.path.join(datapath, subpath)
    return verified_send(path)


@app.route("/image/<shot_id>")
def get_image(shot_id) -> Response:
    path = os.path.join(datapath, IMAGES, shot_id)
    return verified_send(path)


def json_files(path) -> List[str]:
    """List all json files under a dir recursively."""
    paths = []
    for root, _, files in os.walk(datapath):
        for file in files:
            if ".json" in file:
                absolute = os.path.join(root, file)
                relative = os.path.relpath(absolute, path)
                paths.append(relative)
    return paths


def probably_reconstruction(file) -> bool:
    """Decide if a path may be a reconstruction file."""
    return file.endswith("json") and "reconstruction" in file


def reconstruction_files(path) -> List[str]:
    """List all files that look like a reconstruction."""
    files = json_files(path)
    return sorted(filter(probably_reconstruction, files))


def verified_send(file) -> Response:
    if os.path.isfile(file):
        return send_file(file)
    else:
        abort(404)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-d", "--dataset", help="dataset to visualize")
    parser.add_argument(
        "-p", "--port", type=int, default=8080, help="port to bind server to"
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    global datapath
    if args.dataset is not None:
        datapath = os.path.abspath(args.dataset)
    return app.run(host="::", port=args.port)


if __name__ == "__main__":
    exit(main())
