
Python 2 and 3 compatibility
============================

The code must be compatible Python versions 2.7 and 3.6+.

Here are the basic rules to follow for all new code.  Existing code needs to be revised to follow these rules.  See the `official guide <https://docs.python.org/3/howto/pyporting.html>`_ for general rules.


Absolute imports
----------------

Always use absolute imports.  Import absolute imports from future to disable relative imports.

.. code-block:: python

  from __future__ import absolute_import


Print
-----

Use loggers instead of print when possible.  When using print, use it as a function with parenthesis.  Include print from future to disable Python 2 style print.

.. code-block:: python

  from __future__ import print_function


Division
--------

Always add

.. code-block:: python

  from __future__ import division

to make sure that division acts the Python 3 way.  Use ``//`` when you need integer division.


Text
----

All text should be Unicode.  Encode Unicode text from and to bytes using UTF-8 encoding when doing I/O operations.  Encoding and decoding is done as close as possible to the I/O operations.  Some people refer to that as the `Unicode sandwich <https://nedbatchelder.com/text/unipain/unipain.html#35>`_.

By default, string literals are byte strings in Python 2 and Unicode strings in Python 3.  Import Unicode literals from future to make all string literals Unicode in any Python version.

.. code-block:: python

  from __future__ import unicode_literals

When you really need a byte string literal create it with ``b""``.

Use ``opensfm.io.open_rt`` and ``opensfm.io.open_wt`` to open text files for reading and writing.  This functions take care of decoding and encoding UTF-8 files from and into Unicode.

Use ``opensfm.io.json_load``, ``opensfm.io.json_loads``, ``opensfm.io.json_dump``, and ``opensfm.io.json_dumps`` to encode and decode JSON documents.  This functions make sure that the JSON representation is Unicode text and that, when written in a file, it is written using UTF-8 encoding.
