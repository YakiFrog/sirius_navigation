#!/usr/bin/env python3
# Copyright 2025
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
自律移動をキャンセルするプログラム
このプログラムは実行中のナビゲーションタスクをキャンセルします。
"""

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator


def main():
    """メイン関数：ナビゲーションタスクをキャンセル"""
    rclpy.init()

    navigator = BasicNavigator()

    print('ナビゲーションシステムに接続中...')
    
    # ナビゲーションが実行中かチェック
    if not navigator.isTaskComplete():
        print('実行中のナビゲーションタスクをキャンセルしています...')
        navigator.cancelTask()
        print('ナビゲーションタスクがキャンセルされました。')
    else:
        print('実行中のナビゲーションタスクはありません。')

    # クリーンアップ
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
