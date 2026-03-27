#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
日志过滤脚本：只显示 main_fsm (demo_fsm) 和 ego_controller 相关的日志

使用方法：
    # 过滤最新的 ROS 日志
    python3 filter_log.py
    
    # 过滤指定日志文件
    python3 filter_log.py /path/to/your/logfile.log
    
    # 管道方式
    cat logfile.log | python3 filter_log.py
"""

import sys
import re
import os
import glob

# 定义要过滤的关键词
FILTER_KEYWORDS = [
    # main_fsm / demo_fsm 相关
    r'\[Boss\]',
    r'\[IDLE\]',
    r'\[TAKEOFF\]',
    r'\[FSM-Boss\]',
    r'\[setOffboardAndArm\]',
    r'任务一',
    r'任务二',
    r'任务三',
    r'任务四',
    r'任务五',
    r'任务六',
    r'任务开始',
    r'任务结束',
    r'起飞',
    r'降落',
    r'返航',
    r'识别区',
    r'投放区',
    r'攻击阵位',
    r'地图载入',
    r'OFFBOARD',
    r'解锁',
    r'armed',
    r'demo_fsm',
    r'main_fsm',
    
    # ego_controller 相关
    r'\[ego_controller\]',
    r'EGO',
    r'ego_planner',
    r'/ego',
    r'规划',
    r'轨迹',
    r'目标点',
    r'到达目标',
    r'飞行中',
    r'已到达',
    r'待机',
    
    # 高度和位置相关
    r'当前高度',
    r'目标高度',
    r'takeoff_height',
    r'position',
    r'height',
    r'z=',
    r'Z 轴',
]

# 编译正则表达式
filter_patterns = [re.compile(keyword, re.IGNORECASE) for keyword in FILTER_KEYWORDS]

def should_show_line(line):
    """判断某行是否应该显示"""
    for pattern in filter_patterns:
        if pattern.search(line):
            return True
    return False

def filter_log(input_stream, output_stream):
    """过滤日志流"""
    for line in input_stream:
        if should_show_line(line):
            output_stream.write(line)

def get_latest_roslog():
    """获取最新的 ROS rosout.log 文件"""
    ros_log_dir = os.path.expanduser("~/.ros/log/latest")
    if os.path.exists(ros_log_dir):
        rosout_log = os.path.join(ros_log_dir, "rosout.log")
        if os.path.exists(rosout_log):
            return rosout_log
    
    # 如果 latest 不存在，找最新的目录
    log_dirs = sorted(glob.glob(os.path.expanduser("~/.ros/log/*/")))
    if log_dirs:
        latest_dir = log_dirs[-1]
        rosout_log = os.path.join(latest_dir, "rosout.log")
        if os.path.exists(rosout_log):
            return rosout_log
    
    return None

def main():
    if len(sys.argv) > 1:
        # 从文件读取
        log_file = sys.argv[1]
        try:
            with open(log_file, 'r', encoding='utf-8') as f:
                filter_log(f, sys.stdout)
        except FileNotFoundError:
            print(f"错误：文件 '{log_file}' 不存在", file=sys.stderr)
            sys.exit(1)
        except UnicodeDecodeError:
            # 尝试其他编码
            with open(log_file, 'r', encoding='latin-1') as f:
                filter_log(f, sys.stdout)
    else:
        # 无参数时，尝试读取最新的 rosout.log
        rosout_file = get_latest_roslog()
        if rosout_file and os.path.exists(rosout_file):
            print(f"过滤日志文件：{rosout_file}", file=sys.stderr)
            with open(rosout_file, 'r', encoding='utf-8', errors='ignore') as f:
                filter_log(f, sys.stdout)
        else:
            # 从标准输入读取（管道模式）
            print("未找到 ROS 日志，从标准输入读取...", file=sys.stderr)
            filter_log(sys.stdin, sys.stdout)

if __name__ == '__main__':
    main()
