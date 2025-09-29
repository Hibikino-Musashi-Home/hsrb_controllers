hsrc_ex_base_controllers
===============================================================================


概要
-------------------------------

魔改造HSR-Cの台車のリアルタイム制御を実現するためros_controllerプラグイン。
ros-controlと、Linuxのリアルタイムプロセス機構を利用している。


開発者
----------------------------

- 田中　和仁(kazuhito_tanaka@mail.toyota.co.jp)



omni_base_controllerプラグイン
-------------------------------------------------------------------------------

魔改造HSR-Cの全方位台車速度制御を行わせるプラグイン

* 台車の時間軌道追従機能をactionlibインタフェースとtopicインターフェースで提供する
 - 入力時間軌道軌道に追従させるための全方位台車の各ジョイント指令値を計算する
 - 台車yaw軸の時間軌道の入力点は、±π以内正規化しその間の最短距離を結ぶように滑らかに補間する
 - 空の軌道を入力すると停止軌道を作成し停止する
* 台車の指令速度追従機能を提供する
 - 時間軌道追従時以外は直接速度を入力して台車を操作することができる

* 台車の追従においては、時間軌道追従が優先される。
 - 指令速度追従時に時間軌道が入力された場合は時間軌道追従に切り替わる
 - 時間軌道追従時はロックがかかり、時間軌道追従完了まで指令速度は無視される


### ROSインターフェース

#### 出版するトピック
- ~state (control_msgs/JointTrajectoryControllerState) : 台車追従の状態(指令値・現在地・誤差)

- wheel_odom (nav_msgs/Odometry) : 台車のホイールオドメトリ

- tf (tf/tfMessage) : ホイールオドメトリのtf


#### 購読するトピック

- ~joint_trajectory (trajectory_msgs/JointTrajectory) : 台車への入力時間軌道

- command_velocity (geometry_msgs/Twist) : 台車への入力速度

- odom (nav_msgs/Odometry) : 入力オドメトリ

#### 提供するアクション

- ~follow_joint_trajectory (control_msgs/FollowJointTrajectory) : 台車の時間軌道追従


#### パラメータ
- ~joints/steer (string) : 全方位台車のステア軸ジョイント名。指定必須
- ~joints/l_wheel (string) : 全方位台車の左車輪ジョイント名。指定必須
- ~joints/r_wheel (string) : 全方位台車の右車輪ジョイント名。指定必須
- ~model_name(string) : 読み込むURDFのモデル名 (default : '/robot_description')
- ~odom_x/p_gain (double) : 全方位台車のx方向ずれに対するP制御のゲイン。(default : 1.0)
- ~odom_y/p_gain (double) : 全方位台車のy方向ずれに対するP制御のゲイン。(default : 1.0)
- ~odom_t/p_gain (double) : 全方位台車の姿勢方向ずれに対するP制御のゲイン。(default : 1.0)
- ~constraints/odom_x/trajectory (double): 全方位台車のx方向ずれの許容しきい値[m]。
- ~constraints/odom_y/trajectory (double): 全方位台車のy方向ずれの許容しきい値[m]。
- ~constraints/odom_t/trajectory (double): 全方位台車の姿勢方向ずれの許容しきい値[rad]。
- ~base_coordinates (string[]) : 全方位台車を制御する座標系の各軸の名前。指定必須
- ~stop_trajectory_duration (double) : 台車の停止軌道を作成する際の停止時間[s]。(default : 0.5)
- ~state_publish_rate (double) : 台車追従の状態のパブリッシュ周波数[Hz]。(default : 50.0)
- ~odometry_publish_rate (double) : オドメトリトピックのパブリッシュ周波数[Hz]。(default : 30.0)
- ~transform_publish_rate (double) : オドメトリTFのパブリッシュ周波数[Hz]。(default : 30.0)
- ~action_monitor_rate (double) : アクションの状態更新周波数[Hz]。(default : 50.0)
- ~command_timeout (double) : 台車の指定速度途絶とみなす判定時間[s]。(default : 0.5)
- ~tf_prefix (string) : ホイールオドメトリの基準フレーム名とフレーム名に付与するプレフィクス。(default : '')
- ~wheel_odom_map_frame (string) : ホイールオドメトリの基準フレーム名。(default : 'odom')
- ~wheel_odom_base_frame (string) : ホイールオドメトリのフレーム名。(default : 'base_footprint_wheel')
- ~wheel_command_velocity_filter/a (double[]) : ホイールの速度指令フィルタの係数a (default : [1.0])
- ~wheel_command_velocity_filter/b (double[]) : ホイールの速度指令フィルタの係数b (default : [1.0])
- ~steer_command_velocity_filter/a (double[]) : 旋回軸の速度指令フィルタの係数a (default : [1.0])
- ~steer_command_velocity_filter/b (double[]) : 旋回軸の速度指令フィルタの係数b (default : [1.0])
- ~yaw_velocity_limit (double) : 旋回軸指令速度リミット。超える場合はこの値に丸める[rad/s] (defalt : [1.8])
- ~wheel_velocity_limit (double) : 車輪指令速度リミット。超える場合はこの値に丸める[rad/s] (defalt : [8.5])
- ~yaw_actual_velocity_threshold (double) : 旋回軸エンコーダ値速度閾値。超える場合は、異常値とみなし無視する[rad/s] (defalt : [1000.0])
- ~wheel_actual_velocity_threshold (double) : 車輪エンコーダ値速度閾値。超える場合は、異常値とみなし無視する[rad/s] (defalt : [1000.0])

制御モードの切り替えテスト
-------------------------------------------------
omni_base_controllerでは、時間軌道追従と、指令速度追従の2つの制御モードを入力により自動で切り替えている。
そのため、制御モード切り替え時のタイムスタンプ等の扱いにバグが入る可能性がある。
リリース前に以下のテストを実行し、成功することを確認すること。

     $ rostest hsrc_ex_base_controllers change_control_mode.test

参考
-------------------------------------------------

- [ros-controls](https://github.com/ros-controls/)
