TopoLite
=========

.. begin_brief_description

.. image:: https://github.com/EPFL-LGG/TopoLite/raw/master/resources/Teaser.png
        :alt: Teaser
        :align: center

TopoLite is a cross-platform tool for creating topological interlocking shell structures. It supports surface tiling, contact detection, globally interlocking verification, rigid body equilibrium, and topological interlocking block generation. Please check our Topo2019SigA_ paper for more technical details.

.. _Topo2019SigA: https://lgg.epfl.ch/publications/2019/Topological_Interlocking/index.php




GUI Interface
----------------------------------------------------------------------------------------

.. image:: https://github.com/EPFL-LGG/TopoLite/raw/master/resources/screenshot.png
   :alt: Screenshot of TopoCreator
   :align: center

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

On Windows: currently unavailable.

Documenting the code
------------
On MacOS

.. code-block:: bash

    pip install sphinx
    brew install pandoc
    cd docs/
    make html

The generated documents is in ``docs/_build``
