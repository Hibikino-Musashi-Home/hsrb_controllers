^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hsrc_ex_base_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.10.1 (2021-06-10)
-------------------
* Add hsrb_hardware_interface to hsrb_base_controller CMakelists
* Contributors: Takafumi Mizuno

0.10.0 (2020-10-19)
-------------------
* Remove unused private argument from omni_base_odometry-test.cpp
* Remove unnecessary icnludes, modify interfaces
* Uniform argument orders, fix typo, fix test names
  * 引数の順番を位置・速度に統一
  * typoの修正
  * 不適切なテスト名の修正
  * その他細かい修正
* Use static_cast instead of reinterpret_cast
  * bionic/melodic環境だとstatic_castでないとメモリがずれて動作しない
* Use return value of urdf->getJoint() directory
  * kinetic/melodicでの互換性を保つために，urdf->getJoint()の返り値を直接利用する
* Fix filter test, fix action callback arguments
  * フィルタのテストのパラメータを調整
  * kinetic/melodic両方への対応としてアクションのコールバックの引数を変更
* Refactor hsrc_ex_base_controllers
* Contributors: Keisuke Takeshita

0.9.3 (2020-05-20)
------------------

0.9.2 (2020-05-11)
------------------
* Send a action result of PATH_TOLERANCE_VIOLATED if the follow action is active
* fix lint
* fix kinetic support
* fix test
* Contributors: Hiromichi Nakashima, Keisuke Takeshita

0.9.1 (2019-10-15)
------------------
* Wheel velocity outlier check absolute value.
* Apply CR comments.
* Add guard for outlier wheel velocity.
* Reset control method when starting.
* Contributors: Keisuke Takeshita, syuuhei_shiro

0.9.0 (2019-04-22)
------------------
* Fix README and comply with CR comments.
* Change processing period of mock hardware.
* Remove meaningless test.
* Change to subscribe odometry topic.
* Transform velocity for controller state.
* Contributors: 田中 和仁, 竹下 佳佑

0.8.1 (2018-11-28)
------------------
* Fix last_trajectory_input_time.
* Fix publish rate of state.
* Contributors: 竹下 佳佑

0.8.0 (2018-05-22)
------------------
* Add command_velocity_filter
* Make unit test faster
* use wheel odometry's twist when use_laser_odom is True.
* Use default tolerances if not specified in goal
* Add test cases for base controller.
* Parameterize stop_velocity_threshold.
* Contributors: 寺田 耕志, 田中 和仁, 竹下 佳佑, 西野 環

0.7.0 (2017-10-26)
------------------
* Add labels which shows the importance.
* Add velocity limit
* Merge kinetic-devel.
* Contributors: Keisuke Takeshita, Satoru Onoda

0.6.5 (2017-04-13)
------------------
* Normalize yaw angle at initial point.
* Contributors: kazuhito_tanaka

0.6.4 (2016-11-30)
------------------
* apply flake8 to hsrc_ex_base_controllers
* apply lint to hsrc_ex_base_controllers
* Update copyright
* Contributors: Keisuke Takeshita, 西野 環

0.6.3 (2016-10-06)
------------------
* Fix timestamp in stop trajectory.
* Contributors: kazuhito_tanaka

0.6.2 (2016-09-23)
------------------

0.6.1 (2016-04-11)
------------------
* update comment
* shutdown subscriber to avoid deadlock
* Contributors: Keisuke Takeshita

0.6.0 (2016-03-23)
------------------
* Fix README.
* Subscribe laser odometry topic insted of TF.
* Publish wheel odometry.
* add multi_interface_controller for OmniBaseController
* Contributors: Keisuke Takeshita, kazuhito_tanaka

0.5.1 (2015-11-30)
------------------
* Change control mode to trajectory control when trajectory subscribed.
* Modify desired_state calculation.
* Contributors: kazuhito_tanaka

0.5.0 (2015-11-17)
------------------
* Merge branch 'feature/null-check' of /var/git/repositories/hsr/hsrb_controllers into develop
* Fix according to review comments
* add null check in OmniBaseController::UpdateTrajectory
* Restore the order of UpdateBaseState()
* Check trajectory segment only when trajectory list is not empty
* Fix segmenation fault when fail to find trajectory at current time.
* Contributors: Keisuke Takeshita, 田中　和仁, 西野 環

0.4.1 (2015-07-31)
------------------
* Add JointTrajectory's validity decision.
* Contributors: kazuhito_tanaka

0.4.0 (2015-07-27)
------------------
* Stop twist velocity when stopping
* accpeted package renamings
* Merge branch 'feature/comply-with-issues' of /var/git/repositories/hsr/hsrb_controllers into develop
* Modify setting parameter error levels.
* Change Control mode after trajectory following.
* Add test.
* Fix variable names
* Comply with joint permutation.
* Fix base parameter calculation.
* Modify reading urdf description.
* Read base parameters from urdf.
* Add internla state publisher to omni_base_controller
* Contributors: Akiyoshi Ochiai, Keisuke Takeshita, kazuhito_tanaka, yutaka_takaoka, 落合　亮吉

0.3.0 (2015-07-07)
------------------
* Comply with proposal3002.
* Comply with change of package names.
* change package name
* Contributors: Yoshimi Iyoda, kazuhito_tanaka

0.2.1 (2015-06-11)
------------------
* Modify test initialization of hsrb_base_controller.
* Add null-check test for hsrb_base_controller.
* Add test for tmc_hsrc_ex_base_controllers.
* Fix omni_base_controller's initialization.
* Contributors: kazuhito_tanaka

0.2.0 (2015-05-27)
------------------
* Modify version of package.xml
* Make CHANGELOG.rst
* Prepare for auto-release.
* rename library name
* Fix catkin_lint warnings.
* Modify test_depend description.
* ROS pbuilder用Docker環境でbuildを通すために、build_dependを追加
* Put find_package to rostest into CATKIN_ENABLE_TESTING conditional
* Add missing rostest dependency.
* Change initial control mode to velocity control.
* Contributors: Akiyoshi Ochiai, Yoshimi Iyoda, kazuhito_tanaka, 寺田　耕志

0.1.1 (2015-03-16 19:10:11 +0900)
---------------------------------
* Delete unused parameter.
* catkin_lint clean
* Contributors: kazuhito_tanaka

0.1.0 (2015-03-12)
------------------
* Fix reviewed points at tmc_hsrc_ex_base_controllers.
* Modify time parameter initialization.
* Fix reviewed points at tmc_hsrc_ex_base_controllers.
* Delete unused comment.
* Fix laser odometry process.
* Refactoring codes.
* Fix typo.
* Add README.
* Add tests to tmc_hsrc_ex_base_controllers.
* Modify base coordinates index name.
* Comply with over PI yaw trajectory.
* Delete unnecessary process.
* Modify base command mode decision.
* temporary fix some bug by terada.
* Fix blocking in relatime loop.
  - リアルタイムループの中でwaitfortransformしていたので、cantransformに変更
* Add laser odometry to omni_base_controller.
* Add hsrc_ex_base_controllers.
* Contributors: kazuhito_tanaka, 寺田　耕志
