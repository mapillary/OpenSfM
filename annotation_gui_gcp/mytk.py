from __future__ import print_function
from __future__ import division
from __future__ import absolute_import
from __future__ import unicode_literals

import sys

if sys.version_info[0] == 2:
    # Python2
    import Tkinter as tk
    import tkFileDialog as filedialog
    from Tkinter import Listbox
else:
    # Python3
    import tkinter as tk
    from tkinter import filedialog
    from tkinter import Listbox
