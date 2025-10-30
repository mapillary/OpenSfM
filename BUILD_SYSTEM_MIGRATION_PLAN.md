# Migration Plan: Setup.py to Modern Build System (pyproject.toml + scikit-build-core)

**Goal:** Migrate from legacy setup.py to PEP 517/518-compliant pyproject.toml with scikit-build-core. This will work with any modern Python build tool (pip, uv, build, poetry, etc.).

## Current State

### Build System
- **setup.py** (lines 1-110): Manually invokes CMake to build C++ extensions
- Custom `configure_c_extension()` and `build_c_extension()` functions
- Custom `platform_bdist_wheel` class to force platform-specific wheels

### Dependencies
- **requirements.txt**: 20 Python dependencies
- **C++ Dependencies** (via CMake):
  - Eigen3 (required)
  - Ceres Solver
  - Gflags (required)
  - Glog (required)
  - OpenCV (required: core, imgproc, calib3d)
  - SuiteSparse
  - LAPACK
  - OpenMP

### C++ Modules Built
- pybundle (opensfm/src/bundle/)
- pygeo (opensfm/src/geo/)
- pygeometry (opensfm/src/geometry/)
- pyrobust (opensfm/src/robust/)
- pyfeatures (opensfm/src/features/)
- pydense (opensfm/src/dense/)
- pysfm (opensfm/src/sfm/)
- pyfoundation (opensfm/src/foundation/)
- pymap (opensfm/src/map/)

### Package Data
- sensor_data.json
- camera_calibration.yaml
- bow/*.npz files

### Entry Points
- bin/opensfm_run_all
- bin/opensfm

---

## Target State

### Build Backend: scikit-build-core

**Important:** This migration is about adopting PEP 517/518 standards with pyproject.toml, NOT about locking into uv. The resulting build system will work with pip, build, uv, poetry, hatch, or any other PEP 517-compliant tool. uv is mentioned throughout as a recommended option for its speed, but is completely optional.

**Rationale:**
- Modern, PEP 517/518-compliant CMake integration
- Native successor to scikit-build
- Works seamlessly with any PEP 517-compliant build tool (pip, uv, build, poetry, etc.)
- Handles CMake-based extensions properly
- Active development and good documentation
- Tool-agnostic: users can choose their preferred build frontend

**Alternatives Considered:**
- setuptools + cmake extension: Too legacy, similar to current approach
- meson-python: Requires converting CMake to Meson (too disruptive)
- poetry-core: No native CMake support

---

## Implementation Plan

### Phase 1: Create pyproject.toml

#### 1.1 Build System Section
```toml
[build-system]
requires = ["scikit-build-core>=0.8.0", "pybind11"]
build-backend = "scikit_build_core.build"
```

#### 1.2 Project Metadata
```toml
[project]
name = "opensfm"
version = "0.5.2"
description = "A Structure from Motion library"
readme = "README.md"
requires-python = ">=3.8"
license = {text = "BSD"}
authors = [
    {name = "Mapillary"}
]
```

#### 1.3 Runtime Dependencies
Migrate from requirements.txt to:
```toml
[project.dependencies]
- cloudpickle
- exifread
- flask
- fpdf2
- joblib
- matplotlib
- networkx
- numpy
- Pillow
- pyproj
- python-dateutil
- pyyaml
- scipy
- xmltodict
- opencv-python (with platform markers)
```

#### 1.4 Optional Dependencies
```toml
[project.optional-dependencies]
dev = ["pytest", "wheel"]
docs = ["Sphinx"]
test = ["pytest"]
```

#### 1.5 Scripts/Entry Points
```toml
[project.scripts]
opensfm = "opensfm.commands:main"
opensfm_run_all = "opensfm.commands:run_all"
```

#### 1.6 URLs
```toml
[project.urls]
Homepage = "https://github.com/mapillary/OpenSfM"
Documentation = "https://docs.opensfm.org/"
```

### Phase 2: Configure scikit-build-core

#### 2.1 CMake Configuration
```toml
[tool.scikit-build]
cmake.source-dir = "opensfm/src"
cmake.build-type = "Release"
wheel.packages = ["opensfm"]
```

#### 2.2 Package Data Configuration
Configure inclusion of:
- *.so files (pybundle.*, pygeo.*, etc.)
- data/sensor_data.json
- data/camera_calibration.yaml
- data/bow/*.npz

```toml
[tool.scikit-build]
wheel.install-dir = "opensfm"
sdist.include = ["opensfm/data/**"]
```

#### 2.3 CMake Arguments (if needed)
```toml
[tool.scikit-build.cmake.define]
PYTHON_EXECUTABLE = {env = "PYTHON"}
OPENSFM_BUILD_TESTS = "OFF"  # Don't build C++ tests in wheel
```

### Phase 3: CMake Modifications

#### 3.1 Update Install Targets
Modify `opensfm/src/CMakeLists.txt` and subdirectory CMakeLists.txt to:
- Use proper `install()` commands for Python modules
- Install to `${SKBUILD_PLATLIB_DIR}/opensfm/` for .so files
- Ensure data files are installed correctly

Example for each module subdirectory:
```cmake
install(TARGETS pymodulename DESTINATION opensfm)
```

#### 3.2 Remove setup.py Build Logic
The CMake project should be self-contained and installable via scikit-build-core without custom Python build scripts.

### Phase 4: Remove/Update Legacy Files

#### 4.1 Keep but Update
- **setup.py**: Keep minimal version that imports from pyproject.toml (for backwards compatibility) OR remove entirely
- **requirements.txt**: Remove or keep as reference pointing to pyproject.toml

#### 4.2 Update Documentation
- **doc/source/using.rst**: Update build instructions
- **README.md**: Update installation section
- **CONTRIBUTING.md**: Update development setup

### Phase 5: Testing Strategy

#### 5.1 Local Development Testing
```bash
# Create virtual environment with uv
uv venv

# Install in editable mode with all dependencies
uv pip install -e ".[dev,test,docs]"

# Verify C++ extensions load
python -c "import opensfm.pybundle; print('OK')"

# Run tests
pytest
```

#### 5.2 Build Testing
```bash
# Build wheel and sdist
uv build

# Install from wheel in clean environment
uv venv test-env
uv pip install dist/*.whl

# Verify installation
python -c "import opensfm; print(opensfm.__version__)"
```

#### 5.3 Wheel Inspection
```bash
# Check wheel contents
unzip -l dist/opensfm-*.whl

# Verify:
# - All .so files present
# - Data files included
# - Platform tag correct (not "any")
```

### Phase 6: CI/CD Updates

#### 6.1 GitHub Actions Workflow
Update `.github/workflows/*.yml`:
- Install uv
- Use `uv build` instead of `python setup.py`
- Use `uv pip install` for dependencies
- Use `uv sync` for dev environment

Example:
```yaml
- name: Install uv
  run: curl -LsSf https://astral.sh/uv/install.sh | sh

- name: Build
  run: uv build

- name: Install
  run: uv pip install dist/*.whl

- name: Test
  run: uv pip install pytest && pytest
```

### Phase 7: Documentation Updates

#### 7.1 Installation Instructions
Update to:
```bash
# Using uv (recommended for speed)
uv pip install opensfm

# Using standard pip (no uv required)
pip install opensfm

# Development installation with uv
git clone https://github.com/mapillary/OpenSfM
cd OpenSfM
uv venv
uv pip install -e ".[dev]"

# Development installation with standard pip
git clone https://github.com/mapillary/OpenSfM
cd OpenSfM
python -m venv venv
source venv/bin/activate  # or `venv\Scripts\activate` on Windows
pip install -e ".[dev]"
```

#### 7.2 Development Workflow
```bash
# Sync dependencies
uv sync

# Run tests
uv run pytest

# Build documentation
uv run sphinx-build doc/source doc/build
```

---

## Migration Checklist

### Prerequisites
- [ ] Ensure CMake >= 3.0 available
- [ ] Ensure uv installed (`curl -LsSf https://astral.sh/uv/install.sh | sh`)
- [ ] Ensure C++ dependencies installed (Eigen, Ceres, etc.)

### Implementation Steps
- [ ] Create pyproject.toml with all sections
- [ ] Update CMakeLists.txt with proper install() commands
- [ ] Remove build logic from setup.py
- [ ] Test editable install: `uv pip install -e .`
- [ ] Test wheel build: `uv build`
- [ ] Test wheel install in clean environment
- [ ] Verify all C++ modules import correctly
- [ ] Run existing test suite
- [ ] Update documentation (using.rst, README.md)
- [ ] Update CI/CD workflows
- [ ] Test on multiple platforms (Linux, macOS, Windows)

---

## Potential Challenges & Solutions

### Challenge 1: CMake Install Paths
**Problem**: .so files may not install to correct location
**Solution**:
- Use `${SKBUILD_PLATLIB_DIR}` in install commands
- Test with `uv pip install -e .` to verify paths
- Check wheel contents with `unzip -l`

### Challenge 2: Package Data
**Problem**: Data files (JSON, YAML, NPZ) may not be included
**Solution**:
- Configure `[tool.scikit-build]` sdist.include
- Use MANIFEST.in if needed
- Verify with wheel inspection

### Challenge 3: Platform Wheels
**Problem**: Wheels must be platform-specific (not pure Python)
**Solution**:
- scikit-build-core handles this automatically
- Verify wheel filename includes platform tag (e.g., `cp311-linux_x86_64`)

### Challenge 4: System Dependencies
**Problem**: C++ libraries (Eigen, Ceres, etc.) still require system installation
**Solution**:
- Document system dependencies clearly
- Consider conda-forge for users needing pre-built binaries
- For development, document apt/brew install commands

### Challenge 5: Backwards Compatibility
**Problem**: Existing users/CI may rely on `python setup.py install`
**Solution**:
- Keep minimal setup.py that imports from pyproject.toml
- Add deprecation warnings
- Provide migration guide

### Challenge 6: Windows Support
**Problem**: Current setup.py has Windows-specific vcpkg logic
**Solution**:
- Port vcpkg toolchain configuration to scikit-build-core
- Use `[tool.scikit-build.cmake.define]` for Windows-specific variables
- Test on Windows CI

---

## Benefits After Migration

### Tool-Agnostic Build System
**The project will NOT be locked into uv.** The pyproject.toml + scikit-build-core approach follows PEP 517/518 standards, making it compatible with any compliant build tool:

- **pip**: `pip install .` or `pip install -e .`
- **build**: `python -m build`
- **uv**: `uv build` (recommended for speed)
- **poetry**: `poetry build`
- **hatch**: `hatch build`
- **pdm**: `pdm build`

All tools will work identically because they follow the same standard interface. uv is recommended for its speed and modern features, but it's completely optional. Users and CI systems can continue using pip or any other tool they prefer.

### Developer Experience
- Faster dependency resolution with uv (when chosen)
- `uv sync` for one-command environment setup (optional)
- Proper lockfile support (uv.lock, when using uv)
- Better reproducible builds
- Freedom to choose any PEP 517-compliant build tool

### Build System
- Modern, declarative configuration
- PEP 517/518 compliant
- Better IDE integration
- Easier CI/CD configuration

### Maintenance
- Less custom Python build code
- Separation of concerns (CMake for C++, pyproject.toml for Python)
- Easier to understand for contributors
- Better alignment with Python ecosystem standards

### Distribution
- Proper wheel building for PyPI
- Better platform tag handling
- Cleaner sdist generation
- Better dependency resolution for users

---

## Timeline Estimate

- **Phase 1-2** (pyproject.toml creation): 2-4 hours
- **Phase 3** (CMake updates): 3-6 hours
- **Phase 4** (Cleanup): 1-2 hours
- **Phase 5** (Testing): 4-8 hours
- **Phase 6-7** (CI/CD & Docs): 2-4 hours

**Total**: 12-24 hours of work, depending on testing requirements and platform coverage

---

## References

- [scikit-build-core documentation](https://scikit-build-core.readthedocs.io/)
- [PEP 517: Backend Interface](https://peps.python.org/pep-0517/)
- [PEP 518: pyproject.toml](https://peps.python.org/pep-0518/)
- [uv documentation](https://github.com/astral-sh/uv)
- [PyPA Packaging User Guide](https://packaging.python.org/)
