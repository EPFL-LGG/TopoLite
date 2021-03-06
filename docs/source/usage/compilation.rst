Compilation
===========

- Clone the repository.
- Run CMake to build the project based on CMakefiles or CMake/Visual Studio project files.

MacOS
-----

Xcode >= 11.5 (C++ 17)

.. code-block:: bash

    $ mkdir build
    $ cd build
    $ cmake -DCMAKE_BUILD_TYPE=Release ..
    $ make -j 16

.. note::

    the integer following `make -j` is the number of threads supported by your CPU architecture. Replace it with the optimal value for you.

Linux (Ubuntu)
--------------

gcc >= 7.5.0 (C++ 17)

.. code-block:: bash

    $ sudo apt-get install libglu1-mesa-dev
    $ sudo apt-get install libxxf86vm-dev
    $ sudo apt-get install libxrandr-dev
    $ sudo apt-get install libxinerama-dev
    $ sudo apt-get install libxcursor-dev
    $ sudo apt-get install libxi-dev
    $ sudo apt-get install libx11-dev
    $ sudo apt-get install coinor-libipopt-dev

    $ mkdir build
    $ cd build
    $ cmake -DCMAKE_BUILD_TYPE=Release ..
    $ make -j 16

Windows
-------

Compilation notes not available yet.
