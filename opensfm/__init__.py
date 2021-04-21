import sys, os
if sys.platform == 'win32':
    # Python modules require the OpenCV DLLs
    # but these might be in a different path
    if os.environ.get("OPENCV_LIBS"):
        os.add_dll_directory(os.environ.get("OPENCV_LIBS"))