#!/usr/bin/env python3
"""Frenet 静态障碍局部规划器的 ROS 2 入口脚本。

该文件只负责把 package 内的 `pnc_rc.frenet.node.main()` 暴露成可直接执行的
脚本，实际参数声明、订阅发布和规划状态机都在 `node.py` 中。
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pnc_rc.frenet.node import main

if __name__ == "__main__":
    main()
