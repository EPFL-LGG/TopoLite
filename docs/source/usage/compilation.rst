Compilation
===========

Clone the repository, run CMake to generate Makefiles or CMake/Visual Studio project files, and the rest should just work automatically.

MacOS/Ubuntu
------------

.. code-block:: bash

    $ mkdir build
    $ cd build
    $ cmake -DCMAKE_BUILD_TYPE=Release ..
    $ make -j 16

.. note::

    the integer following `make -j` is the number of threads supported by your CPU architecture. Replace it with your optimal value.

Windows
-------

currently unavailable.