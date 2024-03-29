^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tiago_dual_controller_configuration_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.10 (2023-10-25)
-------------------
* Merge branch 'add-moveit-capability-loader' into 'erbium-devel'
  Add capability unless public sim
  See merge request robots/tiago_dual_simulation!21
* Ensure enable_camera argument is set correctly
* Set enable_moveit_camera arg in tiago_controllers.launch
* set use_moveit_camera arg in tiago_controllers correct
* Add advanced_grasping launch arguments
* Add moveit sensor manager param to move_group
* Add capability unless public sim
* Contributors: David ter Kuile, Sai Kishor Kothakota

2.3.9 (2023-09-22)
------------------
* Merge branch 'updated-pids-joint2' into 'erbium-devel'
  Updated pid Joint2 to fix tolerance violation
  See merge request robots/tiago_dual_simulation!29
* Updated joint limits to be safter
* Updated pid valued to fix tolerance violation
* Contributors: Sai Kishor Kothakota, sergiacosta

2.3.8 (2023-06-12)
------------------
* Merge branch 'control-period-reduced' into 'erbium-devel'
  updated pids for the reduction of the control period
  See merge request robots/tiago_dual_simulation!27
* pids updated after a test for all the motions
* updated joint 1 pid
* updated pids for the reduction of the control period reported in the
  gazebo simulation
* Contributors: Sai Kishor Kothakota, ileniaperrella

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
* Apply 1 suggestion(s) to 1 file(s)
* bools to true
* remove goal tolerance artefact
* update for tiago with an arm missing
* override pid gain and goal tolerance in case of no-ee
* generate new files
* override pid gain and goal tolerance in case of no-ee
* generate new files
* Contributors: David ter Kuile, saikishor

2.2.5 (2022-03-21)
------------------
* Merge branch 'add_robotiq_epick_gripper' into 'erbium-devel'
  Add config files for gazebo hardware
  See merge request robots/tiago_dual_simulation!17
* Add config files for gazebo hardware
* Contributors: saikishor, thomaspeyrucain

2.2.4 (2021-11-25)
------------------
* Merge branch 'fix-omni-base' into 'erbium-devel'
  Fix omni base
  See merge request robots/tiago_dual_simulation!16
* adapting to the new omni drive controller
* removing the need for duplicated pids config file
* Contributors: antoniobrandi, saikishor

2.2.3 (2021-11-22)
------------------

2.2.2 (2021-11-22)
------------------

2.2.1 (2021-11-18)
------------------

2.2.0 (2021-11-03)
------------------
* Merge branch 'omni_base_robot' into 'erbium-devel'
  Creating omni base robot
  See merge request robots/tiago_dual_simulation!12
* updated configuration for tiago with omni_base
* Creating omni base robot
* Contributors: antoniobrandi, saikishor

2.1.0 (2021-05-06)
------------------
* Merge branch 'robotiq_gripper' into 'erbium-devel'
  Robotiq gripper
  See merge request robots/tiago_dual_simulation!11
* added missing pal_robotiq_controller_configuration_gazebo dependency
* added the joint trajectory controllers configuration launches of robotiq grippers
* generate the hardware configuration files
* Contributors: Sai Kishor Kothakota, saikishor

2.0.19 (2021-04-13)
-------------------
* Merge branch 'custom-end-effector' into 'erbium-devel'
  feat: enable custom end effector in simulation
  See merge request robots/tiago_dual_simulation!10
* feat: enable custom end effector in sumulation
* Contributors: daniellopez, jordanpalacios

2.0.18 (2020-07-30)
-------------------
* Merge branch 'rename_tf_prefix' into 'erbium-devel'
  Rename tf_prefix param
  See merge request robots/tiago_dual_simulation!8
* Rename tf_prefix param
* Contributors: davidfernandez, victor

2.0.17 (2020-05-27)
-------------------

2.0.16 (2020-04-08)
-------------------
* Merge branch 'add-arm-sides' into 'erbium-devel'
  Add arm_left and arm_right
  See merge request robots/tiago_dual_simulation!7
* Add arm_left and arm_right
* Contributors: Victor Lopez, victor

2.0.15 (2019-10-16)
-------------------

2.0.14 (2019-10-10)
-------------------

2.0.13 (2019-10-02)
-------------------

2.0.12 (2019-09-27)
-------------------

2.0.11 (2019-09-26)
-------------------

2.0.10 (2019-09-26)
-------------------

2.0.9 (2019-08-07)
------------------
* Merge branch 'fix_nav_simulation' into 'erbium-devel'
  Fixing name and launches files due to the refactoring of the tiago_2d_nav
  See merge request robots/tiago_dual_simulation!3
* Fixed the use of the moveit_camera
* Contributors: Victor Lopez, alessandrodifava

2.0.8 (2019-08-01)
------------------
* Add extra joints joint torque sensor state controller
* Contributors: Victor Lopez

2.0.7 (2019-05-02)
------------------
* Merge branch 'fix_wsg_path' into 'erbium-devel'
  Fix path for WSG gripper config files
  See merge request robots/tiago_dual_simulation!1
* Fix path for WSG gripper config files
* Contributors: Victor Lopez, davidfernandez

2.0.6 (2019-04-16)
------------------

2.0.5 (2019-04-16)
------------------
* Use proper tiago dual moveit package
* Initial commit
* Contributors: Victor Lopez
