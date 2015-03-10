^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package applanix_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.8 (2015-03-10)
------------------

0.0.7 (2015-02-28)
------------------

0.0.6 (2014-12-10)
------------------

0.0.5 (2014-12-09)
------------------

0.0.4 (2014-09-18)
------------------

0.0.2 (2014-09-16)
------------------
* Added help output to mapping in applanix_msgs and an error code on invalid arguments. This prevents bad builds from sneaking through.
* Fixed bug in generation of services that would produce zero length files. Turns out that cmake send arguments from a variable as one argument and not split up by spaces. You have to separate the arguments by semicolons to make this work.
* Optimized CMakeLists.txt for applanix_msgs:
  * changed macro to function
  * replaced SET with LIST(APPEND ...)
* Moved the generation code into a macro to clean up the CMakeLists.txt
* Removed custom targets, since those aren't actually necessary.
* Merged applanix_msgs with applanix_srvs and adapted the mapping.py
  script to generate all of the required elements. Updated the
  CMakeLists.txt for applanix_msgs to generate services and AllMsgs.
  Updated applanix_bridge for the changes.
* Contributors: Kareem Shehata

0.0.1 (2014-09-04)
------------------

0.0.0 (2014-09-04)
------------------
* Fixing CMakeLists for building
* Added comment for reasoning behind the use of setup.py in a msgs package
* Removing roslib.load_manifest, no longer needed
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
* Moved service generation script to consolidate messages and services
* Updated applanix_msgs to generate messages so that applanix_bridge can
  use them.
* Added some minimal documentation to the manifests, moved publisher to its own package.
* Adding code headers to everything.
* Fixed diagnostics numbering to match documentation.
* Node to republish status info on the diagnostics topic.
* Adding remaining constants to GeneralStatus message.
* Added params for base gnss setup.
* Adding COM port configuration
* Stomped on merge changes. 'master' should now match 'work'.
* Broken-out IMU message added.
* Auto-subscribe now working.
* Add nav_mode messages every 5s to keep connection alive.
* Last commit before refactoring to separate thread class per socket.
* Add remaining groups, change the array serialization strategy.
* First commit.
* Added existing applanix files.
* Contributors: Administrator, Alex Bencz, Kareem Shehata, Mike Purvis
