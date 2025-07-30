`hsrb_gripper_controller`
===============================================================================


概要
-------------------------------------------------------------------------------

HSR-B搭載グリッパのコントローラ


開発者
----------------------------

- 宗玄 清宏(`kiyohiro_sogen@mail.toyota.co.jp`)

クラス構成
-------------------------------------------------------------------------------

### `HrhGripperController`

High-Ratio-Hypoidグリッパのコントローラ

後述する3つのアクションを提供・管理する

### `IHrhGripperAction`

High-Ratio-Hypoidグリッパコントローラが提供するアクションのインターフェースクラス

後述する3つのアクションはHrhGripperActionを経由して、このクラスを継承する

### `HrhGripperAction`

High-Ratio-Hypoidグリッパコントローラが提供するアクションのテンプレート実装クラス

後述する3つのアクションはこのクラスを継承する

### `HrhGripperFollowTrajectoryAction`

軌道追従アクション

'ros_controllers'の'JointTrajectoryController'が提供する軌道追従アクション＋トピックのインターフェースと極力揃えた

'control_msgs/FollowJointTrajectoryAction'型のアクションと'trajectory_msgs/JointTrajectory'型のトピックインターフェースを提供

ただし、'control_msgs/FollowJointTrajectoryAction'型のアクションの'path_tolerance'には未対応

軌道補間の方法も、指令軌道の最後のpointの位置に、time_from_startで指定した時間に到達するように線形補間する簡易的な形で、
'JointTrajectoryController'の軌道補間方法と異なる

### `HrhGripperGraspAction`

握り込み制御アクション

'tmc_control_msgs/GripperApplyEffortAction'型のアクションを提供する

握り込んで指が物体や対向指に衝突してstallした場合に、アクションが完了する

### `HrhGripperApplyForceAction`

力制御アクション

'tmc_control_msgs/GripperApplyEffortAction'型のアクションを提供する

指に指令力が加わるまで指が閉じ、あるいは開き、stall状態になったらアクションが完了する

ROSインターフェース
---------------------

#### 出版するトピック

なし

#### 購読するトピック

- `~command : trajectory_msgs/JointTrajectory`

    軌道指令

#### 提供するアクション

- `~follow_joint_trajectory : control_msgs/FollowJointTrajectoryAction`

    軌道追従アクション

- `~grasp : tmc_control_msgs/GripperApplyEffortAction`

    握り込み制御アクション

- `~apply_force : tmc_control_msgs/GripperApplyEffortAction`

    力制御アクション

#### パラメータ

- `~joints : string[]`

    関節名

- `~follow_joint_trajectory_action_monitor_rate : double`

    軌道追従アクションをモニタする周期[Hz] default : 20.0

- `~position_goal_tolerance : double`

    軌道追従アクションにおけるゴール位置の許容誤差[rad] default : 0.05

- `~position_goal_time_tolerance : double`

    軌道追従アクションにおけるゴール到達時刻の許容誤差[s] default : 0.05

- `~grasp_action_monitor_rate : double`

    握り込み制御アクションをモニタする周期[Hz] default : 20.0

- `~torque_goal_tolerance : double`

    握り込み制御アクションにおけるゴールトルクの許容誤差[Nm] default : 1.0

- `~apply_force_action_monitor_rate : double`

    力制御アクションをモニタする周期[Hz] default : 20.0

- `~force_goal_tolerance : double`

    力制御アクションにおけるゴール力の許容誤差[N] default : 1.0

- `~stall_velocity_threshold : double`

    力制御アクションにおけるstall判定とする速度閾値[rad/s] default : 0.2

- `~stall_velocity_threshold : double`

    力制御アクションにおけるstall判定とする時間[s] default : 0.5

- `~force_calib_data_path : string`

    力制御アクションにおいて指先力を補正するために必要なキャリブデータのパス

- `~force_lpf_coeff : double`

    力制御アクションにおいて指先力にかけているRCローパスフィルタの差分方程式の係数 min : 0.0, max : 1.0, default : 0.8

- `~force_control_pgain : double`

    力制御アクションにおいて指先力から目標位置を決定するために行っているPID制御のPゲイン default : 0.1

- `~force_control_igain : double`

    力制御アクションにおいて指先力から目標位置を決定するために行っているPID制御のIゲイン default : 0.15

- `~force_control_dgain : double`

    力制御アクションにおいて指先力から目標位置を決定するために行っているPID制御のDゲイン default : 5.0

- `~force_ierr_max : double`

    力制御アクションにおいて行っているPID制御の誤差積分蓄積最大値 default : 0.15

テスト
-------
* 軌道追従アクションにゴールを送信し、アクションが成功する
* 軌道追従アクションに全てのパラメータを設定したゴールを送信し、アクションが成功する
* 軌道追従アクションにおいて不正な関節名でゴールを送信し、アクションが失敗する
* 軌道追従アクションにおいて不正な関節数でゴールを送信し、アクションが失敗する
* 軌道追従アクションにおいてtime_from_startが0でゴールを送信し、アクションが失敗する
* 軌道追従アクションにおいて目標到達時刻が過去になるようにゴールを送信し、アクションが失敗する
* 軌道追従アクションにおいて空軌道のゴールを送信し、アクションが失敗する
* 軌道追従アクションにおいてコントローラの停止によりアクションが失敗する
* 軌道追従アクションにおいて目標関節角度に到達せずアクションが失敗する
* トピックによる軌道追従
* 握り込み制御アクションにゴールを送信し、アクションが成功する
* 握り込み制御アクションにゴールを送信し、stall時の現在値と指令値に乖離があり、アクションが失敗する
* 握り込み制御アクションにおいてコントローラの停止によりアクションが失敗する
* 力制御アクションにゴールを送信し、アクションが成功する
* 力制御アクションにゴールを送信し、stall時の現在値と指令値に乖離があり、アクションが失敗する
* 力制御アクションにおいてコントローラの停止によりアクションが失敗する
* 力制御アクションにおいて速度が出続けてアクションが終わらない
* 一部のパラメータは不正な値であってもデフォルト値でコントローラが立ち上がる
* follow_joint_trajectory_action_monitor_rateが非正でありコントローラが立ち上がらない
* grasp_action_monitor_rateが非正でありコントローラが立ち上がらない
* apply_force_action_monitor_rateが非正でありコントローラが立ち上がらない
* jointsがHandleの名前と一致せずコントローラが立ち上がらない
* jointsが2つありコントローラが立ち上がらない

