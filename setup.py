#!/usr/bin/env python3
import platform
import multiprocessing
import os
import subprocess
import sys

import setuptools
from sphinx.setup_command import BuildDoc
from wheel.bdist_wheel import bdist_wheel


VERSION = (0, 5, 2, "post6")


def version_str(version):
    return ".".join(map(str, version))


class platform_bdist_wheel(bdist_wheel):
    """Patched bdist_well to make sure wheels include platform tag."""

    def finalize_options(self):
        bdist_wheel.finalize_options(self)
        self.root_is_pure = False


def configure_c_extension():
    """Configure cmake project to C extension."""
    print(
        f"Configuring for python {sys.version_info.major}.{sys.version_info.minor}..."
    )
    os.makedirs("cmake_build", exist_ok=True)
    cmake_command = [
        "cmake",
        "../opensfm/src",
        "-DPYTHON_EXECUTABLE=" + sys.executable,
    ]

    # if platform.system() == 'Darwin':
    #     # Проверяем архитектуру процессора
    #     if platform.machine() == 'arm64':
    #         print('Detected Apple Silicon processor')
    #
    #         # Define the compilers
    #         c_compiler = os.path.abspath('/opt/homebrew/Cellar/gcc@12/12.3.0/bin/gcc-12')
    #         cxx_compiler = os.path.abspath('/opt/homebrew/Cellar/gcc@12/12.3.0/bin/g++-12')
    #         c_compiler_arg = f'-DCMAKE_C_COMPILER={c_compiler}'
    #         cxx_compiler_arg = f'-DCMAKE_CXX_COMPILER={cxx_compiler}'
    #
    #         cmake_command += [
    #             c_compiler_arg,
    #             cxx_compiler_arg,
    #         ]
    #         print(cmake_command)
    #         raise RuntimeError()

    if sys.platform == "win32":
        cmake_command += [
            "-DVCPKG_TARGET_TRIPLET=x64-windows",
            "-DCMAKE_TOOLCHAIN_FILE=../vcpkg/scripts/buildsystems/vcpkg.cmake",
        ]
    subprocess.check_call(cmake_command, cwd="cmake_build")


def build_c_extension():
    """Compile C extension."""
    print("Compiling extension...")
    if sys.platform == "win32":
        subprocess.check_call(
            ["cmake", "--build", ".", "--config", "Release"], cwd="cmake_build"
        )
    else:
        subprocess.check_call(
            ["make", "-s", "-j" + str(multiprocessing.cpu_count())], cwd="cmake_build"
        )


configure_c_extension()
build_c_extension()

lib_folder = os.path.dirname(os.path.realpath(__file__))
requirement_path = f"{lib_folder}/requirements.txt"
install_requires = []  # Here we'll add: ["gunicorn", "docutils>=0.3", "lxml==0.5a7"]
if os.path.isfile(requirement_path):
    with open(requirement_path) as f:
        install_requires = f.read().splitlines()

setuptools.setup(
    name="opensfm",
    version=version_str(VERSION),
    description="A Structure from Motion library",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/mapillary/OpenSfM",
    project_urls={
        "Documentation": "https://docs.opensfm.org/",
    },
    author="Mapillary",
    license="BSD",
    packages=setuptools.find_packages(),
    scripts=[
        "bin/opensfm_run_all",
        "bin/opensfm",
        "bin/opensfm_main.py"
    ],
    package_data={
        "opensfm": [
            "pybundle.*",
            "pygeo.*",
            "pygeometry.*",
            "pyrobust.*",
            "pyfeatures.*",
            "pydense.*",
            "pysfm.*",
            "pyfoundation.*",
            "pymap.*",
            "data/sensor_data.json",
            "data/camera_calibration.yaml",
            "data/bow/bow_hahog_root_uchar_10000.npz",
            "data/bow/bow_hahog_root_uchar_64.npz",
        ]
    },
    cmdclass={
        "bdist_wheel": platform_bdist_wheel,
        "build_doc": BuildDoc,
    },
    command_options={
        "build_doc": {
            "project": ("setup.py", "OpenSfM"),
            "version": ("setup.py", version_str(VERSION[:2])),
            "release": ("setup.py", version_str(VERSION)),
            "source_dir": ("setup.py", "doc/source"),
            "build_dir": ("setup.py", "build/doc"),
        }
    },
    install_requires=install_requires,
)