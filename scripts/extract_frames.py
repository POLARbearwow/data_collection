#!/usr/bin/env python3
"""
extract_frames.py  —  将视频拆分为图片

用法：
    python3 extract_frames.py --video input.mp4 --out dir [--stride 1] [--start 0] [--end -1]

参数说明：
    --video   输入视频路径
    --out     输出文件夹，自动创建
    --stride  每隔多少帧保存一张（默认1，表示全部帧）
    --start   从第几帧开始截取（含，默认0）
    --end     截取到第几帧（含，默认-1 表示直到视频结束）

输出文件名格式： frame_%06d.jpg
"""

import cv2
import argparse
import os
from pathlib import Path


def extract_frames(video_path: str, out_dir: str, stride: int = 1, start: int = 0, end: int = -1):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"无法打开视频: {video_path}")

    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    end = total - 1 if end < 0 else min(end, total - 1)

    Path(out_dir).mkdir(parents=True, exist_ok=True)

    idx = 0  # 当前帧编号
    saved = 0  # 保存计数
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if idx > end:
            break
        if idx >= start and (idx - start) % stride == 0:
            filename = os.path.join(out_dir, f"frame_{idx:06d}.jpg")
            cv2.imwrite(filename, frame)
            saved += 1
        idx += 1

    cap.release()
    print(f"已保存 {saved} 张图片到 {out_dir}")


def main():
    parser = argparse.ArgumentParser(description="将视频拆分为图片帧")
    parser.add_argument("--video", required=True, help="输入视频路径")
    parser.add_argument("--out", required=True, help="输出文件夹")
    parser.add_argument("--stride", type=int, default=1, help="帧间隔，默认1")
    parser.add_argument("--start", type=int, default=0, help="起始帧，默认0")
    parser.add_argument("--end", type=int, default=-1, help="结束帧，默认-1(到最后)")
    args = parser.parse_args()

    extract_frames(args.video, args.out, args.stride, args.start, args.end)


if __name__ == "__main__":
    main() 