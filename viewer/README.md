# OpenSfM viewer

## Workflow

### Setup
1. `./node_modules.sh`
    - Download npm dependencies from unpkg

### Serve a dataset
1. `python3 server.py -d <path/to/my/dataset>`
    - Serve the reconstruction for a dataset
2. Browse to `http://localhost:8080`
3. Select the reconstruction file

### Load datasets manually
Note that images will not be loaded for this case.
1. `python3 server.py`
    - Serve the reconstruction for a dataset
2. Browse to `http://localhost:8080`
3. Pick or drop the reconstruction files
