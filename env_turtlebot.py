# env_turtlebot.py

import rclpy
import gymnasium as gym
from gymnasium import spaces
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import time
import subprocess

class TurtleBotEnv(gym.Env):
    metadata = {"render_modes": ["human"]}

    def __init__(self):
        super().__init__()
        self.node = rclpy.create_node('turtlebot_rl')
        self.pub_cmd = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_data = None
        self.node.create_subscription(LaserScan, '/scan',
                                      self.scan_callback, 10)

        # Action / Observation spaces
        self.action_space = spaces.Discrete(3)  # 前進・左旋回・右旋回
        self.observation_space = spaces.Box(
            low=0.0, high=3.5, shape=(24,), dtype=np.float32
        )

    def scan_callback(self, msg):
        # 360°→24点にサンプリング
        raw = np.array(msg.ranges[::15], dtype=np.float32)

        # 無限大や nan を sensor の最大値・最小値に置き換え
        # high=3.5, low=0.0 で定義しているのでそこにマッピング
        clean = np.nan_to_num(
            raw,
            nan=self.observation_space.high,        # nan → maxレンジ
            posinf=self.observation_space.high,     # +inf → maxレンジ
            neginf=self.observation_space.low       # -inf → minレンジ
        )
        # さらにセンサーの計測可能範囲（0.0～3.5）にはめ込む
        clean = np.clip(clean, self.observation_space.low, self.observation_space.high)

        self.scan_data = clean

    def step(self, action):
        # 1) コマンド送信
        twist = Twist()
        if action == 0:
            twist.linear.x = 0.2
        elif action == 1:
            twist.angular.z = +0.5
        else:
            twist.angular.z = -0.5
        self.pub_cmd.publish(twist)

        # 2) センサー更新待ち
        rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.scan_data is None:
            state = np.ones(self.observation_space.shape, dtype=np.float32)
        else:
            # NaN/±inf → 有効なレンジ値に, さらに clip
            state = np.nan_to_num(
                self.scan_data,
                nan=self.observation_space.high,
                posinf=self.observation_space.high,
                neginf=self.observation_space.low
            )
            state = np.clip(state,
                            self.observation_space.low,
                            self.observation_space.high)

        # 3) 終了判定・報酬設計
        # 障害物までの最短距離が 0.2m 未満なら終端
        terminated = bool(np.min(state) < 0.2)
        # エピソード長制限を使わないなら常に False
        truncated = False
        # 衝突時は大きなペナルティ、それ以外は一定報酬
        reward = -10.0 if terminated else 1.0

        # 4) info に補足
        info = {"min_range": float(np.min(state))}

        # 5) 5要素を返却
        return state, reward, terminated, truncated, info

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        subprocess.run([
            'ros2', 'service', 'call',
            '/reset_simulation', 'std_srvs/srv/Empty'
        ], check=True, stdout=subprocess.DEVNULL)
        time.sleep(1.0)

        obs = np.ones(24, dtype=np.float32)
        return obs, {}

    def close(self):
        rclpy.shutdown()
