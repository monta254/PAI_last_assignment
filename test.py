# test.py

import os
import rclpy

# Matplotlibキャッシュ警告を消すなら書き込み可能なディレクトリを指定
os.environ.setdefault('MPLCONFIGDIR', os.path.expanduser('~/tmp/matplotlib_config'))
os.makedirs(os.environ['MPLCONFIGDIR'], exist_ok=True)

from env_turtlebot import TurtleBotEnv
from stable_baselines3 import PPO

def main():
    # 1回だけROS2コンテキストを初期化
    rclpy.init()

    env = TurtleBotEnv()
    model = PPO.load("ppo_turtlebot")

    obs, _ = env.reset()
    try:
        while True:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)

            if terminated or truncated:
                obs, _ = env.reset()
    except KeyboardInterrupt:
        pass
    finally:
        # init済みなら確実にshutdown
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass
        env.close()

if __name__ == "__main__":
    main()
