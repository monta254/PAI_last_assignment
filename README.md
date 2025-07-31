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

依存パッケージのインストール

ROS 2パッケージ

TurtleBot3とそのシミュレーション環境を導入します：

sudo apt update
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-gazebo

また、turtlebot3_simulationsは以下からクローン：

cd ~/ros2_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

Pythonパッケージ

pip install stable-baselines3[extra] gym==0.26.2 shimmy matplotlib


⸻

実行方法

1.	GazeboでTurtleBot3ワールドを起動：

	ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

2.	強化学習スクリプトを実行：

	python3 train_turtlebot.py


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

test.txt
	•	Gazebo環境と連携し、評価を行います。

⸻

外部リポジトリ

以下の外部リポジトリを利用：
	•	TurtleBot3 Simulation
 このうち、

これらは自作コードには含めず、上記コマンドで各自インストールしてください。

⸻
了解です！コンテナ環境でROS 2 + Gazeboを使った迷路走行ロボットの強化学習なら、比較的スタンダードな方法で構築できます。

⸻

🔧 前提条件
	•	コンテナ（Docker）上でROS 2 + Gazeboが動作している
	•	VSCode Dev Containerなどでコンテナ内に入って作業可能
	•	Python（強化学習コード）、ROS 2ノード、Gazebo環境が連携可能な状態

⸻

**全体の流れ**

1. ROS 2 + Gazebo環境準備
	•	ROS 2 Humble (or Foxy/Galactic)
	•	gazebo_ros_pkgsをインストール

sudo apt install ros-humble-gazebo-ros-pkgs

2. 迷路ワールドの作成/利用

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py


⸻

3. 強化学習環境とのブリッジ
	•	Gym + ROS 2を接続するために、gym-gazebo2を利用するのが一般的

pip install gym
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-rmw-cyclonedds-cpp

	•	gym-gazebo2の環境構築:

git clone https://github.com/AcutronicRobotics/gym-gazebo2.git
cd gym-gazebo2
pip install -e .


⸻

4. 強化学習アルゴリズム（例: PPO）
	•	stable-baselines3を使用：

pip install stable-baselines3

例コード:

import gym
import gym_gazebo2
from stable_baselines3 import PPO

env = gym.make('TurtleBot3Maze-v0')  # gym-gazebo2の迷路環境
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=50000)


⸻

5. ポリシー適用・評価

obs = env.reset()
for _ in range(1000):
    action, _ = model.predict(obs)
    obs, reward, done, info = env.step(action)
    if done:
        obs = env.reset()


⸻

6. 学習映像の記録（rosbag or video）
	•	rosbagで記録：

ros2 bag record /camera/image_raw /odom /cmd_vel

	•	Pythonで動画保存：

import cv2
frames = []
# loop内でenv.render()したフレームをframesにappendし、最後にcv2.VideoWriterで保存


⸻

🔑 参考リポジトリ
	•	gym-gazebo2 (AcutronicRobotics)
	•	TurtleBot3 Simulations (ROBOTIS)
	•	Stable Baselines3 Docs

⸻

👉 次は「迷路環境の作成」からやる？それとも「TurtleBot3シミュレーションを動かしてRLに繋ぐ」ところからやる？
