^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tiago_dual_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.13.0 (2026-07-17)
-------------------
* start apps using localization manager
* Contributors: antoniobrandi

4.12.0 (2026-06-02)
-------------------
* stores_server log level
* separate advanced navigation
* Contributors: antoniobrandi

4.11.0 (2026-04-13)
-------------------
* Add support for new gazebo
* Contributors: thomaspeyrucain

4.10.1 (2025-12-15)
-------------------
* missing deps
* Contributors: antoniobrandi

4.10.0 (2025-12-11)
-------------------
* Point Cloud filter pipeline
* Contributors: antoniobrandi

4.9.0 (2025-10-28)
------------------
* Revert "Merge branch 'fix/aca/missing-dep-public-sim' into 'humble-devel'"
  This reverts merge request !57
* Contributors: antoniobrandi

4.8.1 (2025-10-27)
------------------
* added missing dep for public sim
* Contributors: andreacapodacqua

4.8.0 (2025-10-09)
------------------
* added gzclient and rviz args
* Contributors: martinaannicelli

4.7.1 (2025-06-05)
------------------
* Add pal_urdf_utils as env var path for gazebo
* Contributors: Aina

4.7.0 (2025-05-07)
------------------
* add use_sim_time to robot_info
* Contributors: antoniobrandi

4.6.0 (2025-05-06)
------------------
* rviz use_sim_time
* Contributors: antoniobrandi

4.5.2 (2025-05-05)
------------------
* fix deps
* Contributors: andreacapodacqua

4.5.1 (2025-04-09)
------------------
* rviz_config_file for public nav
* Contributors: salvatorepiccolo

4.5.0 (2025-04-03)
------------------
* store robot_info in tmp
* Contributors: antoniobrandi

4.4.0 (2025-04-03)
------------------
* Adopt pal configuration
* Contributors: antoniobrandi

4.3.0 (2024-12-02)
------------------
* Merge branch 'abr/feat/docking' into 'humble-devel'
  use docking arg
  See merge request robots/tiago_dual_simulation!48
* always start docking with adv navigation
* use docking arg
* Contributors: antoniobrandi

4.2.3 (2024-11-04)
------------------
* Merge branch 'dtk/fix/move-group-base' into 'humble-devel'
  Pass missing base_type to move_group.launch.py
  See merge request robots/tiago_dual_simulation!46
* Pass missing base_type to move_group.launch.py
* Contributors: David ter Kuile, davidterkuile

4.2.2 (2024-09-18)
------------------
* Merge branch 'dtk/fix/fix-navigation-launch' into 'humble-devel'
  Launch correct navigation file and add dependency
  See merge request robots/tiago_dual_simulation!45
* Launch correct navigation file and add dependency
* Contributors: David ter Kuile, antoniobrandi

4.2.1 (2024-09-12)
------------------
* Merge branch 'tpe/add_missing_dependency' into 'humble-devel'
  Add missing dependecy
  See merge request robots/tiago_dual_simulation!47
* Add missing dependecy
* Contributors: andreacapodacqua, thomas.peyrucain

4.2.0 (2024-08-26)
------------------
* Update parameters
* Add motor model parameter
* Contributors: Aina, thomas.peyrucain

4.1.5 (2024-07-23)
------------------
* add tuck arm arg and condition
* Contributors: sergiacosta

4.1.4 (2024-07-09)
------------------
* Add missing navigation related launch arguments
* Add slam argument for navigation
* Contributors: Noel Jimenez

4.1.3 (2024-06-26)
------------------
* Merge branch 'dtk/move-robot-args' into 'humble-devel'
  Change import for launch args
  See merge request robots/tiago_dual_simulation!42
* Change import for launch args
* Contributors: David ter Kuile, davidterkuile

4.1.2 (2024-06-10)
------------------
* Merge branch 'dtk/fix/add-public-sim-arg' into 'humble-devel'
  Pass public sim arg to tiago_bringup launch
  See merge request robots/tiago_dual_simulation!40
* Pass public sim arg to tiago_bringup launch
* Merge branch 'fix/srdf_files' into 'humble-devel'
  add arguments for moveit
  See merge request robots/tiago_dual_simulation!39
* add arguments for moveit
* Contributors: Aina Irisarri, David ter Kuile, davidterkuile

4.1.1 (2024-05-09)
------------------
* Merge branch 'omm/feat/public_sim' into 'humble-devel'
  Added is_public_sim check
  See merge request robots/tiago_dual_simulation!38
* Added is_public_sim check
* Merge branch 'omm/fix/launch_standarization' into 'humble-devel'
  Launch standarization
  See merge request robots/tiago_dual_simulation!36
* Launch standarization
* Contributors: Oscar, davidterkuile, oscarmartinez

4.1.0 (2024-02-28)
------------------
* Launch MoveIt 2 by default
* add pal robotiq description package
* Update package name of pal_hey5
* Contributors: Aina Irisarri, David ter Kuile, Noel Jimenez

4.0.1 (2024-01-18)
------------------
* Add tiago_dual_moveit_config dependency
* Contributors: David ter Kuile

4.0.0 (2024-01-18)
------------------
* Merge branch 'ros2-tiago-gazebo' into 'humble-devel'
  Ros2 tiago gazebo
  See merge request robots/tiago_dual_simulation!31
* Add maintainer
* Remove test dependency
* Update copyright
* Update authors
* CMake version to 3.8
* remove space for linter
* update to  new robot argument method
* Fix flake test
* fix name of move group launch file
* Update launch file layout and use new scoped_launch_file function
* Add website tags
* Setup declared args  properly
* Add tuck arm node
* Update propagation of gazebo env var in launchfile
* Remove default value to robot spawn.launch
* LAunch structure read but gazebo not yet workingg
* Add launch files
* Only package xml + cmakelist
* Contributors: David ter Kuile, davidterkuile

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
