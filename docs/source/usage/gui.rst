.. _gui_description:

User Interface
==============

We provide a GUI to users if you want to visualise your assemblies.

The user Interface is based on Wenzel Jakob's project NanoGUI. For more informations about
this project, look `here  <https://nanogui.readthedocs.io/en/latest/>`_.


TopoCreator
-----------

The purpose of this program is to load an already defined assembly (as JSON
file). Modify the properties of the assembly visually; save the modifications
into a new file. Then, this file can be used as an input file for checking
whether or not the new assembly is interlocking.

.. note::

   We are currently adding the possibility of checking directly in the software if
   the modified assembly is Interlocking.

Quickstart
^^^^^^^^^^

- Use the `Load` button to read a JSON file which contains your assembly.
- The `Scene` button gives you access to a bunch of rendering settings.
- The `Parameters` button triggers a menu that lists the available modifications to do
  on the assembly.
- Once the assembly looks like what you wanted. Click on `Save`.

.. only:: html

   .. figure:: ../imgs/gui_cut.gif

      Cut Up. and Cut. Low parameters modify the height of the blocks.

   .. figure:: ../imgs/gui_patterns_diff.gif

      Ptn. ID modifies the global shape of the patterns.

   .. figure:: ../imgs/gui_patterns_size.gif

      Ptn. Radius increases/decreases the density of the pattern.

   .. figure:: ../imgs/gui_angle.gif

      Aug. angle 


InterlockingGUI
---------------

.. note::
    Under development