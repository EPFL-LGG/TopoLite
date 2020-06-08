.. begin_brief_description

TopoLite
========

.. image:: https://github.com/EPFL-LGG/TopoLite/raw/master/resources/Teaser.png
        :alt: Teaser
        :align: center

TopoLite is a cross-platform tool for creating topological interlocking shell
structures. It supports surface tiling, contact detection, globally interlocking
verification, rigid body equilibrium, and topological interlocking block
generation.

Abstract
--------

We study assemblies of convex rigid blocks regularly arranged to approximate a
given freeform surface. Our designs rely solely on the geometric arrangement of
blocks to form a stable assembly, neither requiring explicit connectors or
complex joints, nor relying on friction between blocks. The convexity of the
blocks simplifies fabrication, as they can be easily cut from different
materials such as stone, wood, or foam. However, designing stable assemblies is
challenging, since adjacent pairs of blocks are restricted in their relative
motion only in the direction orthogonal to a single common planar interface
surface. We show that despite this weak interaction, structurally stable, and in
some cases, globally interlocking assemblies can be found for a variety of
freeform designs.

Our optimization algorithm is based on a theoretical link
between static equilibrium conditions and a geometric, global interlocking
property of the assembly-that an assembly is globally interlocking if and only
if the equilibrium conditions are satisfied for arbitrary external forces and
torques. Inspired by this connection, we define a measure of stability that
spans from single-load equilibrium to global interlocking, motivated by tilt
analysis experiments used in structural engineering. We use this measure to
optimize the geometry of blocks to achieve a static equilibrium for a maximal
cone of directions, as opposed to considering only self-load scenarios with a
single gravity direction. In the limit, this optimization can achieve globally
interlocking structures. We show how different geometric patterns give rise to a
variety of design options and validate our results with physical prototypes.

Please check our Topo2019SigA_ paper for more technical details.

.. _Topo2019SigA: https://lgg.epfl.ch/publications/2019/Topological_Interlocking/index.php


GUI Interface
-------------

We provide a GUI to users if you want to visualise assemblies. An example of usage
is given in :ref:`gui_examples`.

.. image:: https://github.com/EPFL-LGG/TopoLite/raw/master/resources/screenshot.png
   :alt: Screenshot of TopoCreator :align: center

.. end_brief_description


License
^^^^^^^

NanoGUI is provided under a MIT-style license that can be found in the licence
section. By using, distributing, or contributing to this project, you agree to
the terms and conditions of this license.