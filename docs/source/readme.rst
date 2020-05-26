Compiling
=========

Mac
---

-  Install boost

.. code:: bash

       brew install boost

-  Building the project. In Topolite root folder:

.. code:: bash

       mkdir build && cd build
       cmake -DCMAKE_BUILD_TYPE=Release ..
       make -j 16

Windows
-------

-  Install boost
-  Install Visual Studio Linux Development Tools

Ubuntu
------

-  Tested with Ubuntu 18.04 and 20.04 .
-  Install boost:

.. code:: bash

       # If not already on apt
       sudo add-apt-repository ppa:boost-latest/ppa
       sudo apt-get update
       #
       sudo apt install libboost-all-dev

-  Building the project. In Topolite root folder:

.. code:: bash

       mkdir build && cd build
       cmake -DCMAKE_BUILD_TYPE=Release ..
       make -j 16

..

   Note: the integer following ``make -j`` is the number of threads
   supported by your CPU architecture. Replace it with your optimal
   value.
