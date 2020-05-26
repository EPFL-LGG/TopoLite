.. Topolite documentation master file, created by
   sphinx-quickstart on Mon May 25 16:13:52 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

########
Topolite
########

Description
===========

Theoretical context
-------------------

.. note::
   cite the article + summarize this more ? Maybe one or two pictures of assemblies/a movie?


We study assemblies of convex rigid blocks regularly arranged to approximate a given freeform surface.
Our designs rely solely on the geometric arrangement of blocks to form a stable assembly, neither
requiring explicit connectors or complex joints, nor relying on friction between blocks. The convexity
of the blocks simplifies fabrication, as they can be easily cut from different materials such as stone,
wood, or foam. However, designing stable assemblies is challenging, since adjacent pairs ofblocks are
restricted in their relative motion only in the direction orthogonal to a single common planar interface
surface. We show that despite this weak interaction, structurally stable, and in some cases, globally
interlocking assemblies can be found for a variety of freeform designs. Our optimization algorithm is
based on a theoretical link between static equilibrium conditions and a geometric, global interlocking
property ofthe assembly—that an assembly is globally in- terlocking if and only if the equilibrium
conditions are satisfied for arbitrary external forces and torques. Inspired by this connection, we
define a measure of stability that spans from single-load equilibrium to global interlocking, motivated
by tilt analysis experiments used in structural engineering. We use this measure to optimize the geometry
of blocks to achieve a static equilibrium for a maximal cone of directions, as opposed to considering
only self-load scenarios with a single gravity direction. In the limit, this optimization can achieve
globally interlocking structures. We show how different geometric patterns give rise to a variety of
design options and validate our results with physical prototypes.


Topolite
--------

.. note::
   short presentation of the program: purpose, what's great with it, cite Ipopt and the important libs used.

Lorem Ipsum is simply dummy text of the printing and typesetting industry. Lorem Ipsum has been the industry's
standard dummy text ever since the 1500s, when an unknown printer took a galley of type and scrambled it to
make a type specimen book. It has survived not only five centuries, but also the leap into electronic
typesetting, remaining essentially unchanged. It was popularised in the 1960s with the release of Letraset
sheets containing Lorem Ipsum passages, and more recently with desktop publishing software like Aldus
PageMaker including versions of Lorem Ipsum.


License
=======

NanoGUI is provided under a MIT-style license that can be found in the licence section. By using, distributing,
or contributing to this project, you agree to the terms and conditions of this license.


Contents
========

.. toctree::
   :glob:
   :maxdepth: 2

   Introduction <self>
   usage/install
   usage/examples
   api/api
   usage/license

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`


