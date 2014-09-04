^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package applanix_srvs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2014-09-04)
------------------

0.0.0 (2014-09-04)
------------------
* Fixing CMakeLists for building
* Changing maintainer of the applanix modules to be me
* Removed a bunch of the chunky comments from the CMakeLists.txt for all
  of the sub projects.
* Moved mapping.py back to applanix_msgs because it's needed by
  applanix_bridge and it makes more sense for it to be there. Also updated
  applanix_bridge to use the new applanix_srvs package.
* Renamed applanix_generated_msgs to applanix_srvs since it really
  generates services not messages. I tried putting the generation into the
  same project as messages, but because the services have the same name as
  the messages, it would end up with name clashes. Hence the rename, it
  makes it clearer what is where.
* Contributors: Kareem Shehata
