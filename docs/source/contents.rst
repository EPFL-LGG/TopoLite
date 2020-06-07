Compiling
=========

Mac
---

-  Building the project. In Topolite root folder:

.. code:: bash

       mkdir build && cd build
       cmake -DCMAKE_BUILD_TYPE=Release ..
       make -j 16

Windows
-------

-  Install Visual Studio Linux Development Tools

Ubuntu
------

-  Tested with Ubuntu 18.04 and 20.04 .
-  Building the project. In Topolite root folder:

.. code:: bash

       mkdir build && cd build
       cmake -DCMAKE_BUILD_TYPE=Release ..
       make -j 16

..

   Note: the integer following ``make -j`` is the number of threads
   supported by your CPU architecture. Replace it with your optimal
   value.

Documenting the code
====================

Requirements
------------

Install Sphinx and Pandoc. The example below is given for macOS

.. code:: bash

       pip install sphinx
       brew install pandoc

Usage
-----

.. code:: bash

       cd docs/
       make html

then, open ``build/index.html`` .
