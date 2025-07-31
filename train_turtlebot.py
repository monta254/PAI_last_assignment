# train_turtlebot.py

import os
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
import gymnasium as gym

# 自作環境をインポート
from env_turtlebot import TurtleBotEnv

def make_env():
    env = TurtleBotEnv()
    return Monitor(env)   # Monitor でログを取る

if __name__ == "__main__":
    # 並列不要なら DummyVecEnv 1つだけ
    env = DummyVecEnv([make_env])

    # モデル定義
    model = PPO(
        policy="MlpPolicy",
        env=env,
        verbose=1,
        tensorboard_log="./ppo_turtlebot_tensorboard/"
    )

    # 学習開始
    model.learn(total_timesteps=60)

    # モデル保存
    model.save("ppo_turtlebot")

    env.close()
