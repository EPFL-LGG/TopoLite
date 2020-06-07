TopoLite
=========

.. begin_brief_description
TopoLite is a cross-platform tool for creating topological interlocking shell structures. It supports surface tiling, contact detection, globally interlocking verification, rigid body equilibrium, and topological interlocking block generation. Please refer to our 2019 Siggraph Asia paper(https://lgg.epfl.ch/publications/2019/Topological_Interlocking/index.php).
.. end_brief_description

Compiling
=========
Mac
---

* Building the project. In Topolite root folder:

``` bash
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make -j 16
```

Ubuntu
------

* Tested with Ubuntu 18.04 and 20.04 .
* Building the project. In Topolite root folder:

``` bash
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make -j 16
```

> Note: the integer following `make -j` is the number of threads supported by
> your CPU architecture. Replace it with your optimal value.

Windows
-------

* Install Visual Studio Linux Development Tools


Documenting the code
====================

Requirements
------------

Install Sphinx and Pandoc. The example below is given for macOS

``` bash
    pip install sphinx
    brew install pandoc
```

Usage
-----

``` bash
    cd docs/
    make html
```

then, open `build/index.html` .
