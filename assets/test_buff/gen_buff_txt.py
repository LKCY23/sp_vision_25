#!/usr/bin/env python3
"""
通用姿态文件生成脚本

假设云台始终保持不动，平视前方（单位四元数 q = 1,0,0,0）。
时间戳按视频帧率均匀递增：t_i = i / fps。

默认在 assets/test_buff 目录下运行，使用相对路径指定要处理的视频。

使用示例（在 assets/test_buff/ 目录下）：
  python gen_buff1_txt.py buff1.mp4
  # 将生成同目录下的 buff1.txt

也支持其他相对路径，例如：
  python gen_buff1_txt.py ../other_dir/foo.avi
  # 将生成 ../other_dir/foo.txt

需要：
  - Python3
  - OpenCV (cv2)
"""

import sys
from pathlib import Path

import cv2


def main() -> int:
  # 视频路径：第一个命令行参数，默认为当前目录下的 buff1.mp4
  if len(sys.argv) >= 2:
    video_path = Path(sys.argv[1])
  else:
    video_path = Path("buff1.mp4")

  video_path = video_path.expanduser()
  txt_path = video_path.with_suffix(".txt")

  if not video_path.exists():
    print(f"[ERROR] 视频文件不存在: {video_path}", file=sys.stderr)
    return 1

  cap = cv2.VideoCapture(str(video_path))
  if not cap.isOpened():
    print(f"[ERROR] 无法打开视频: {video_path}", file=sys.stderr)
    return 1

  fps = cap.get(cv2.CAP_PROP_FPS)
  if not fps or fps <= 1e-3:
    # 如果读不到 fps，就退化为 30fps
    fps = 30.0
    print(f"[WARN] 读取视频帧率失败，使用默认 fps={fps}", file=sys.stderr)

  frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
  if frame_count <= 0:
    print(f"[ERROR] 读取帧数失败，frame_count={frame_count}", file=sys.stderr)
    return 1

  print(f"[INFO] 视频: {video_path}")
  print(f"[INFO] 帧率: {fps:.3f} fps，帧数: {frame_count}，时长约 {frame_count / fps:.2f} s")
  print(f"[INFO] 生成姿态文件: {txt_path}")

  with txt_path.open("w", encoding="utf-8") as f:
    for i in range(frame_count):
      t = i / fps  # 秒
      # 恒定四元数 (w,x,y,z) = (1,0,0,0)，表示“云台不动，姿态固定”
      f.write(f"{t} 1 0 0 0\n")

  print("[INFO] 生成完成")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())

