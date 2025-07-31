⸻

プロジェクト概要

本プロジェクトは、ROS2およびGazebo上でTurtleBot3を用いた強化学習（PPO）による自律移動学習を実装したものです。
ロボットが障害物を回避しながら移動するタスクを対象としています。

⸻

開発環境
	•	Ubuntu 22.04 (推奨)
	•	ROS 2 Humble
	•	Gazebo (gazebo_ros_pkgs)
	•	Python 3.13
	•	Stable-Baselines3


⸻

実行方法

1.	GazeboでTurtleBot3ワールドをセットアップ・起動：
   
        cd && git clone --recursive https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
  	ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
  	
3.	強化学習スクリプトを実行：

	python3 train_turtlebot.py
python3 train_turtlebot.py
	python3 test.py


⸻

ファイル構成

以下は本リポジトリ内の主要ファイルです。

train_turtlebot.py
	•	PPO（Proximal Policy Optimization）を用いたTurtleBot3の学習スクリプト。
	•	Gazebo環境と連携し、学習を行います。

turtlebot_env.py
	•	TurtleBot3用の強化学習環境クラス（Gym形式）。
	•	LIDARセンサ情報を状態空間とし、ロボット速度指令を行動空間として定義。
	•	報酬関数：障害物回避、前進ボーナス、衝突ペナルティ。

test.py
	•	Gazebo環境と連携し、評価を行います。

⸻

外部リポジトリ

以下の外部リポジトリを利用：
	•	TurtleBot3 Simulation




