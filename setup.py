#!/usr/bin/env python3

import multiprocessing
import os
import platform
import subprocess
import sys
from pathlib import Path
from shutil import copytree, rmtree

import setuptools
from setuptools.command.install import install
from sphinx.setup_command import BuildDoc
from wheel.bdist_wheel import bdist_wheel


VERSION = (0, 5, 2, "post22")

THIRD_PARTY_INSTALL_DIR = Path(__file__).parent / "third_party_install"
THIRD_PARTY_BUILD_DIR = Path(__file__).parent / "third_party_build"
THIRD_PARTY_SOURCE_DIR = Path(__file__).parent / "opensfm/src/third_party"
THIRD_PARTY_BUILD_DIR.mkdir(exist_ok=True)
LIB_DIR = THIRD_PARTY_INSTALL_DIR / "usr/local/lib"
INCLUDE_DIR = THIRD_PARTY_INSTALL_DIR / "usr/local/include"

SHARE_DIR = THIRD_PARTY_INSTALL_DIR / "usr/local/share"
SHARED_CMAKE_DIR = THIRD_PARTY_INSTALL_DIR / "usr/local/lib/cmake/"

CMAKE_PREFIX_PATH = (
    f'-DCMAKE_PREFIX_PATH={SHARED_CMAKE_DIR};'
    f'{SHARE_DIR};'
    f'{SHARE_DIR}/eigen3;'
    f'{INCLUDE_DIR};'
    f'{INCLUDE_DIR}/eigen3;'
    f'{LIB_DIR};'
    f'{THIRD_PARTY_BUILD_DIR};'
    f'{THIRD_PARTY_BUILD_DIR / "SuiteSparse"}; '
)
CMAKE_OSX_SYSROOT = '-DCMAKE_OSX_SYSROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX10.14.sdk/'

os.environ['LDFLAGS'] = f'-L/opt/homebrew/Cellar/libomp/18.1.6/lib'
os.environ['C_INCLUDE_PATH'] = f'/opt/homebrew/Cellar/libomp/18.1.6/include/'
os.environ['CPLUS_INCLUDE_PATH'] = f'/opt/homebrew/Cellar/libomp/18.1.6/include/:/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX14.5.sdk/usr/include/c++/v1/'


def _is_apple_silicon() -> bool:

    if platform.system() == 'Darwin':
        if platform.machine() == 'arm64':
            return True
    return False


if _is_apple_silicon():
    os.environ['arch'] = 'arm64'

#


def version_str(version):
    return ".".join(map(str, version))


class platform_bdist_wheel(bdist_wheel):
    """Patched bdist_well to make sure wheels include platform tag."""

    def finalize_options(self):
        bdist_wheel.finalize_options(self)
        self.root_is_pure = False


class InstallPlatlib(install):
    def finalize_options(self):
        install.finalize_options(self)
        self.install_lib = self.install_platlib


def install_gflag(install_dir: Path):
    gflags_build_dir = THIRD_PARTY_BUILD_DIR / "gflags"
    gflag_source_dir = Path(__file__).parent.joinpath("opensfm/src/third_party/gflags/")
    if not gflags_build_dir.exists():
        gflags_build_dir.mkdir()
    cmake_command = [
        'cmake',
        CMAKE_OSX_SYSROOT,
        '-DCMAKE_CXX_FLAGS="-fPIC"',
        gflag_source_dir
    ]
    subprocess.check_call(
        cmake_command,
        cwd=gflags_build_dir.as_posix(),
        env=os.environ,
    )

    subprocess.check_call(["make", "-j8"], cwd=gflags_build_dir.as_posix())
    subprocess.check_call(["make", f"DESTDIR={install_dir}", "install"], cwd=gflags_build_dir.as_posix())


def install_glog(install_dir: Path):
    glog_build_dir = THIRD_PARTY_BUILD_DIR / "glog"
    glog_source_dir = THIRD_PARTY_SOURCE_DIR / "glog"

    if not glog_build_dir.exists():
        glog_build_dir.mkdir()

    cmake_command = [
        'cmake',
        CMAKE_OSX_SYSROOT,
        CMAKE_PREFIX_PATH,
        glog_source_dir.as_posix()
    ]

    subprocess.check_call(
        cmake_command,
        cwd=glog_build_dir.as_posix(),
        env=os.environ,
    )
    subprocess.check_call(["make", "-j8"], cwd=glog_build_dir.as_posix())
    subprocess.check_call(["make", f"DESTDIR={install_dir}", "install"], cwd=glog_build_dir.as_posix())


def install_eigen(install_dir: Path):
    eigen_build_dir = THIRD_PARTY_BUILD_DIR / "eigen"
    eigen_source_dir = THIRD_PARTY_SOURCE_DIR / "eigen"

    if not eigen_build_dir.exists():
        eigen_build_dir.mkdir()

    cmake_command = [
        'cmake',
        CMAKE_OSX_SYSROOT,
        CMAKE_PREFIX_PATH,
        eigen_source_dir.as_posix(),
    ]


    subprocess.check_call(
        cmake_command,
        cwd=eigen_build_dir.as_posix(),
        env=os.environ,
    )
    subprocess.check_call(["make", f"DESTDIR={install_dir}", "install"], cwd=eigen_build_dir.as_posix())


def install_tbb():
    tbb_source_dir = THIRD_PARTY_SOURCE_DIR / "tbb"
    tbb_build_dir = THIRD_PARTY_BUILD_DIR / "tbb"
    tbb_build_lib = tbb_build_dir / 'lib'
    tbb_build_include = tbb_build_dir / 'include'

    make_command = []
    make_command.extend(['make'])
    subprocess.check_call(
        make_command,
        cwd=tbb_source_dir.as_posix(),
        env=os.environ,
    )
    tbb_potential_dirs = list(tbb_source_dir.rglob("libtbb*"))
    if len(tbb_potential_dirs) == 0:
        raise RuntimeError('Could not find tbb lib')
    tbb_libb_dir = tbb_potential_dirs[0].parent
    print(f'Found tbb lib dir: {tbb_libb_dir}')
    if tbb_build_lib.exists():
        rmtree(tbb_build_lib)
    if tbb_build_include.exists():
        rmtree(tbb_build_include)

    copytree(tbb_libb_dir, tbb_build_lib)
    copytree(tbb_source_dir.joinpath("include"), tbb_build_include)
    os.environ['TBB_ROOT'] = tbb_build_dir.as_posix()
    os.environ['LD_LIBRARY_PATH'] = f"{tbb_build_lib.as_posix()}:{os.environ.get('LD_LIBRARY_PATH', '')}"
    os.environ['LIBRARY_PATH'] = f"{tbb_build_lib.as_posix()}:{os.environ.get('LD_LIBRARY_PATH', '')}"
    os.environ['C_INCLUDE_PATH'] = f"{tbb_build_include}:{os.environ.get('C_INCLUDE_PATH', '')}"
    os.environ['CPLUS_INCLUDE_PATH'] = f"{tbb_build_include}:{os.environ.get('CPLUS_INCLUDE_PATH', '')}"


def install_suitesparse(install_dir: Path):
    suitesparse_build_dir = THIRD_PARTY_BUILD_DIR / "SuiteSparse"
    suitesparse_source_dir = THIRD_PARTY_SOURCE_DIR / "SuiteSparse"
    suitesparse_build_lib = suitesparse_build_dir / 'lib'
    suitesparse_build_include = suitesparse_build_dir / 'include'

    if not suitesparse_build_dir.exists():
        suitesparse_build_dir.mkdir()

    subprocess.check_call(
        [
            "make",
            '-j8'
        ],
        cwd=suitesparse_source_dir.as_posix()
    )
    subprocess.check_call(
        [
            "make",
            'local'
        ],
        cwd=suitesparse_source_dir.as_posix()
    )
    subprocess.check_call(["make", f"DESTDIR={install_dir}", "install"], cwd=suitesparse_source_dir.as_posix())

    if suitesparse_build_lib.exists():
        rmtree(suitesparse_build_lib)
    if suitesparse_build_include.exists():
        rmtree(suitesparse_build_include)

    copytree(suitesparse_source_dir / "lib", suitesparse_build_lib)
    copytree(suitesparse_source_dir.joinpath("include"), suitesparse_build_include)


def install_ceres(install_dir: Path):
    ceres_build_dir = THIRD_PARTY_BUILD_DIR / "ceres_solver"
    ceres_source_dir = THIRD_PARTY_SOURCE_DIR / "ceres-solver"
    if not ceres_build_dir.exists():
        ceres_build_dir.mkdir()

    cmake_command = [
        'cmake',
        CMAKE_OSX_SYSROOT,
        CMAKE_PREFIX_PATH,
        f'-DSUITESPARSE_LIBRARY_DIR_HINTS={install_dir}',
        f'-DSUITESPARSE_INCLUDE_DIRS={THIRD_PARTY_BUILD_DIR / "SuiteSparse/include/"}',
        f'-DSUITESPARSE_LIBRARY_DIRS={THIRD_PARTY_BUILD_DIR / "SuiteSparse/lib/"}',
        f'-DSUITESPARSE_LIBRARIES={THIRD_PARTY_BUILD_DIR / "SuiteSparse/lib/"}',
        # f'-DTBB_ROOT={os.environ["TBB_ROOT"]}',

        ceres_source_dir.as_posix()
    ]
    subprocess.check_call(
        cmake_command,
        cwd=ceres_build_dir.as_posix()
    )
    subprocess.check_call(["make", "-j8"], cwd=ceres_build_dir.as_posix())
    subprocess.check_call(["make", f"DESTDIR={install_dir}", "install"], cwd=ceres_build_dir.as_posix(), env=os.environ)


def install_opensfm(install_dir: Path):

    if not install_dir.exists():
        install_dir.mkdir()

    cmake_command = [
        "cmake",
        CMAKE_OSX_SYSROOT,
        CMAKE_PREFIX_PATH,
        f'-DEigen3_DIR={INCLUDE_DIR}/eigen3',
        "../opensfm/src",
        "-DPYTHON_EXECUTABLE=" + sys.executable,
    ]

    if sys.platform == "win32":
        cmake_command += [
            "-DVCPKG_TARGET_TRIPLET=x64-windows",
            "-DCMAKE_TOOLCHAIN_FILE=../vcpkg/scripts/buildsystems/vcpkg.cmake",
        ]
    subprocess.check_call(
        cmake_command,
        cwd=install_dir.as_posix(),
        # env=os.environ
    )


def configure_c_extension():
    """Configure cmake project to C extension."""
    print(
        f"Configuring for python {sys.version_info.major}.{sys.version_info.minor}..."
    )

    if platform.system() == 'Darwin':
        # Third party install
        install_gflag(THIRD_PARTY_INSTALL_DIR)
        install_glog(THIRD_PARTY_INSTALL_DIR)
        install_eigen(THIRD_PARTY_INSTALL_DIR)
        install_tbb()
        install_suitesparse(THIRD_PARTY_INSTALL_DIR)
        install_ceres(THIRD_PARTY_INSTALL_DIR)

    install_opensfm(Path(__file__).parent / "cmake_build")


def build_c_extension():
    """Compile C extension."""
    print("Compiling extension...")
    if sys.platform == "win32":
        subprocess.check_call(
            ["cmake", "--build", ".", "--config", "Release"], cwd="cmake_build"
        )
    else:
        subprocess.check_call(
            ["make", "-j" + str(multiprocessing.cpu_count())], cwd="cmake_build"
        )


configure_c_extension()
build_c_extension()

install_requires = []
with open("requirements.txt") as f:
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
        "bin/import_colmap.py",
        "bin/opensfm.bat",
        "bin/plot_features",
        "bin/plot_tracks",
        "bin/create_calibrtion_pattern",
        "bin/import_video",
        "bin/opensfm_main.py",
        "bin/plot_gcp.py",
        "bin/run_bundler",
        "bin/export_geojson",
        "bin/iterative_self_calibration",
        "bin/opensfm_run_all",
        "bin/plot_inliers",
        "bin/update_geotag",
        "bin/export_gps",
        "bin/migrate_undistort.sh",
        "bin/opensfm_run_all.bat",
        "bin/plot_matches.py",
        "bin/import_bundler",
        "bin/opensfm",
        "bin/plot_depthmaps",
        "bin/plot_submodels_gps",
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
        "install": InstallPlatlib,
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
    install_requires=install_requires
)
