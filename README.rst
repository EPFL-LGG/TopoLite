TopoLite
=========


.. begin_brief_description

TopoLite is a cross-platform tool for creating topological interlocking shell structures. It supports surface tiling, contact detection, globally interlocking verification, rigid body equilibrium, and topological interlocking block generation. Please refer to our 2019 Siggraph Asia paper(https://lgg.epfl.ch/publications/2019/Topological_Interlocking/index.php). 

.. end_brief_description


Compilation
-----------
Clone the repository, run CMake to generate Makefiles or CMake/Visual Studio project files, and the rest should just work automatically.

On MacOS/Ubuntu:

.. code-block:: bash
    $ mkdir build
    $ cd build
    $ cmake -DCMAKE_BUILD_TYPE=Release ..
    $ make -j 16

> Note: the integer following `make -j` is the number of threads supported by
> your CPU architecture. Replace it with your optimal value.

On Windows:

Currently unavailable

Documenting the code
------------

Install Sphinx and Pandoc. The example below is given for macOS:

.. code-block:: bash
    pip install sphinx
    brew install pandoc

Usage:

.. code-block:: bash
    cd docs/
    make html

then, open `build/index.html` .
