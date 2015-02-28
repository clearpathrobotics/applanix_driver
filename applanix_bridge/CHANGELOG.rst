^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package applanix_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.7 (2015-02-28)
------------------
* Adding launch file for publishing Odometry and TF.
  Useful for adding Odometry and TF's when using bag files that don't include them
* Fixed orientation mapping when publishing Odometry and converting from NED to ENU
* Contributors: Kareem Shehata

0.0.6 (2014-12-10)
------------------
* Gams param max_heading_error_rms has to be assigned to 3.0 degrees default and cannot be zero.
* Contributors: Vaibhav Kumar Mehta

0.0.5 (2014-12-09)
------------------
* Typo in the imu angular velocity z component is fixed. All the required NED (/reference or /vehicle for Applanix) to ENU (/base_footprint or /base_link in ROS) frame conversions are applied to get correct Pose and twist data.
* gams params, dmi params and some geometry params added to be configured correctly. some sample values are provided in the example.launch also added in this commit
* launch file updated as auxiliary gnss to reference frame lever is not neeeded.
* updated the launch file with correct installation parameters for applanix
* Origin now includes the z dimension or altitude so that the odometry msgs are w.r.t. the current surface and not the sea level. quaternion assignment in tf broadcaster had a bug which is fixed now.
* Fixing typo in publisher.py, we Transfrom != Transform
* Contributors: Vaibhav Kumar Mehta

0.0.4 (2014-09-18)
------------------
* Removing diagnostic_msgs from find_package because it isn't needed at build, it's handled by the run_depends, and it makes the build servers cry.
* Contributors: Kareem Shehata

0.0.2 (2014-09-16)
------------------
* Increased queue size to 5 since data smoke test fails otherwise
* Added queue sizes on Publishers to make Indigo complain less
* Added dependency on geodesy
* Merged applanix_msgs with applanix_srvs and adapted the mapping.py
  script to generate all of the required elements. Updated the
  CMakeLists.txt for applanix_msgs to generate services and AllMsgs.
  Updated applanix_bridge for the changes.
* Contributors: Kareem Shehata

0.0.1 (2014-09-04)
------------------

0.0.0 (2014-09-04)
------------------
* Adding install targets
* Replaced the gps_utm.py with a call to geodesy.utm. Should work, but not
  well tested.
* Cleaned up the dependencies from applanix_bridge
* Changing maintainer of the applanix modules to be me
* Fixed package names in launch file
* Removed a bunch of the chunky comments from the CMakeLists.txt for all
  of the sub projects.
* Moved the smoke test into the applanix_bridge package.
  It seems to work. At the very least it passes, though it does spit out a
  bunch of warnings.
* Moved mapping.py back to applanix_msgs because it's needed by
  applanix_bridge and it makes more sense for it to be there. Also updated
  applanix_bridge to use the new applanix_srvs package.
* Updated applanix_msgs to generate messages so that applanix_bridge can
  use them.
* Making applanix_bridge into a proper rospy package that you can run. Can
  now run things, though they fail because of the missing applanix_msgs.
  Also note that I've removed a bunch of the dependencies until I can
  figure out how to do those properly.
* First stage of catkinizing: catkin metapackage, and consolidate the
  python stuff into applanix_bridge. Have yet to update the CMakeLists.txt
  for the applanix_bridge package to work properly. Also need a setup.py
* Fixed translator for use in Hydro. Seems to work in testing on the Tango
  Tasmania platform.
* Switch the translator to use separate genpy package.
* Adding capture file and basic smoke test to driver.
* Moving gps_utm file to correct package.
* Added some minimal documentation to the manifests, moved publisher to its own package.
* Adding code headers to everything.
* Now publishing the origin of our UTM coordinates wrt. the UTM zone zero point
* Small cleanup of unnecessary shebangs, etc.
* Adding option to autoset a UTM origin
* Added params for base gnss setup.
* Adding COM port configuration
* Adding translation node to convert Applanix messages into standard ROS messages
* Stomped on merge changes. 'master' should now match 'work'.
* Merge branch 'work'
* rename applanix_ctl to applanix_generated_msgs
* Auto-subscribe now working.
* Add nav_mode messages every 5s to keep connection alive.
* Working COM port params.
* Initial support for services.
* Last commit before refactoring to separate thread class per socket.
* Add remaining groups, change the array serialization strategy.
* First commit.
* Contributors: Administrator, Alex Bencz, Kareem Shehata, Mike Purvis, Ryan Gariepy
