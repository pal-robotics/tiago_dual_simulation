^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tiago_dual_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.10 (2023-10-25)
-------------------
* Merge branch 'add-moveit-capability-loader' into 'erbium-devel'
  Add capability unless public sim
  See merge request robots/tiago_dual_simulation!21
* remove world folder
* Add table with two cylinders world
* Set enable_moveit_camera arg in tiago_controllers.launch
* set use_moveit_camera arg in tiago_controllers correct
* Add advanced_grasping launch arguments
* Add capability unless public sim
* Contributors: David ter Kuile, Sai Kishor Kothakota, sergiacosta

2.3.9 (2023-09-22)
------------------
* Merge branch 'arm-type-param' into 'erbium-devel'
  Add arm type to pal_robot_info
  See merge request robots/tiago_dual_simulation!30
* Add arm type to pal_robot_info
* Merge branch 'davidterkuile-erbium-devel-patch-62924' into 'erbium-devel'
  Remove extra gz paths that were move to pal_gazebo_worlds
  See merge request robots/tiago_dual_simulation!28
* Remove extra gz paths that were move to pal_gazebo_worlds
* Contributors: David ter Kuile, Jordan Palacios, Sai Kishor Kothakota, davidterkuile

2.3.8 (2023-06-12)
------------------

2.3.7 (2023-03-09)
------------------

2.3.6 (2023-03-07)
------------------

2.3.5 (2023-02-23)
------------------

2.3.4 (2023-01-31)
------------------

2.3.3 (2023-01-30)
------------------

2.3.2 (2023-01-23)
------------------
* Merge branch 'robot-state-publisher' into 'erbium-devel'
  update to robot_state_publisher
  See merge request robots/tiago_dual_simulation!22
* update to robot_state_publisher
* Contributors: David ter Kuile, Jordan Palacios

2.3.1 (2022-07-21)
------------------
* Merge branch 'add_omni_tiago_dual' into 'erbium-devel'
  Add base_type to the missing launch files
  See merge request robots/tiago_dual_simulation!19
* =Add base_type to the missing launch files
* Contributors: saikishor, thomaspeyrucain

2.3.0 (2022-05-03)
------------------
* Merge branch 'no-end-effector-bugfix' into 'erbium-devel'
  No end effector bugfix
  See merge request robots/tiago_dual_simulation!18
* file_suffix consistency
* Merge branch 'no-end-effector-bugfix' of gitlab:robots/tiago_dual_simulation into no-end-effector-bugfix
* bools to true
* Apply 1 suggestion(s) to 1 file(s)
* merge
* update for tiago with an arm missing
* Changing default arg
* override pid gain and goal tolerance in case of no-ee
* generate new files
* override pid gain and goal tolerance in case of no-ee
* generate new files
* Contributors: David ter Kuile, saikishor

2.2.5 (2022-03-21)
------------------

2.2.4 (2021-11-25)
------------------
* Merge branch 'fix-omni-base' into 'erbium-devel'
  Fix omni base
  See merge request robots/tiago_dual_simulation!16
* removing the need for duplicated pids config file
* Contributors: antoniobrandi, saikishor

2.2.3 (2021-11-22)
------------------

2.2.2 (2021-11-22)
------------------

2.2.1 (2021-11-18)
------------------
* Merge branch 'pal_robot_info' into 'erbium-devel'
  Setup the info of all the robot configuration in pal_robot_info
  See merge request robots/tiago_dual_simulation!13
* Setup the info of all the robot configuration in pal_robot_info
* Contributors: Sai Kishor Kothakota, saikishor

2.2.0 (2021-11-03)
------------------
* Merge branch 'omni_base_robot' into 'erbium-devel'
  Creating omni base robot
  See merge request robots/tiago_dual_simulation!12
* Creating omni base robot
* Contributors: antoniobrandi, saikishor

2.1.0 (2021-05-06)
------------------

2.0.19 (2021-04-13)
-------------------

2.0.18 (2020-07-30)
-------------------
* Merge branch 'rename_tf_prefix' into 'erbium-devel'
  Rename tf_prefix param
  See merge request robots/tiago_dual_simulation!8
* Rename tf_prefix param
* Contributors: davidfernandez, victor

2.0.17 (2020-05-27)
-------------------
* Merge branch 'tiago_dual_screen' into 'erbium-devel'
  Add has_screen to some launch files
  See merge request robots/tiago_dual_simulation!9
* Add has_screen to some launch files
* Contributors: Victor Lopez, victor

2.0.16 (2020-04-08)
-------------------
* Merge branch 'add-arm-sides' into 'erbium-devel'
  Add arm_left and arm_right
  See merge request robots/tiago_dual_simulation!7
* Add arm_left and arm_right
* Contributors: Victor Lopez, victor

2.0.15 (2019-10-16)
-------------------
* Merge branch 'refactor' into 'erbium-devel'
  Refactor
  See merge request robots/tiago_dual_simulation!6
* removed joystick from sim
* fixed twist mux usage
* Contributors: Procópio Stein, Victor Lopez

2.0.14 (2019-10-10)
-------------------
* Merge branch 'remove-sonar-cloud' into 'erbium-devel'
  remove sonar cloud
  See merge request robots/tiago_dual_simulation!5
* removed dep
* remove sonar cloud
* Contributors: Procópio Stein, Victor Lopez

2.0.13 (2019-10-02)
-------------------
* Remove speed limit
* Contributors: Victor Lopez

2.0.12 (2019-09-27)
-------------------
* Merge branch 'speed-limit' into 'erbium-devel'
  removed speed limit dep as it is in bringup
  See merge request robots/tiago_dual_simulation!4
* removed speed limit dep as it is in bringup
* Contributors: Procópio Stein, Victor Lopez

2.0.11 (2019-09-26)
-------------------

2.0.10 (2019-09-26)
-------------------

2.0.9 (2019-08-07)
------------------
* Merge branch 'fix_nav_simulation' into 'erbium-devel'
  Fixing name and launches files due to the refactoring of the tiago_2d_nav
  See merge request robots/tiago_dual_simulation!3
* Fixing name and launches files due to the refactoring of the tiago_2d_nav
* Contributors: Victor Lopez, alessandrodifava

2.0.8 (2019-08-01)
------------------

2.0.7 (2019-05-02)
------------------

2.0.6 (2019-04-16)
------------------
* Fix wrong install rule
* Contributors: Victor Lopez

2.0.5 (2019-04-16)
------------------
* Initial commit
* Contributors: Victor Lopez
