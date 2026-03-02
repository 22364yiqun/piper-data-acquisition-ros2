# -*- coding: utf-8 -*-
"""
HDF5 数据可视化工具
支持：
  - 三路 RGB 图像同步回放
  - 深度图可视化（伪彩色）
  - 关节角度 / 末端位姿时序曲线
  - 键盘交互：Space 暂停/播放，← → 单帧步进，q 退出，s 保存当前帧

用法：
  python visualize_hdf5.py --hdf5 /path/to/episode_0.hdf5
  python visualize_hdf5.py --hdf5 /path/to/episode_0.hdf5 --fps 15 --show_depth
  python visualize_hdf5.py --hdf5 /path/to/episode_0.hdf5 --export_video output.mp4
"""

import argparse
import json
import sys
import os
import time

import cv2
import h5py
import numpy as np
import matplotlib
matplotlib.use('TkAgg')          # 若 TkAgg 不可用，改为 'Qt5Agg' 或 'Agg'
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation


# ─────────────────────────── 工具函数 ────────────────────────────

def colorize_depth(depth_img, min_mm=200, max_mm=2000):
    """将 uint16 深度图转为伪彩色 BGR 图 (uint8)。"""
    d = depth_img.astype(np.float32)
    d = np.clip(d, min_mm, max_mm)
    d = ((d - min_mm) / (max_mm - min_mm) * 255).astype(np.uint8)
    return cv2.applyColorMap(d, cv2.COLORMAP_JET)


def resize_to_height(img, target_h):
    h, w = img.shape[:2]
    scale = target_h / h
    return cv2.resize(img, (int(w * scale), target_h), interpolation=cv2.INTER_AREA)


def put_label(img, text, color=(255, 255, 255)):
    cv2.putText(img, text, (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1, cv2.LINE_AA)
    return img


# ─────────────────────────── 加载数据 ────────────────────────────

def load_hdf5(path):
    data = {}
    with h5py.File(path, 'r') as f:
        obs = f['observations']

        # ── 相机 ──
        data['cameras'] = {}
        for cam in obs.keys():
            if not isinstance(obs[cam], h5py.Group):
                continue
            cam_data = {}
            if 'rgb' in obs[cam]:
                cam_data['rgb'] = obs[cam]['rgb'][:]           # (N, H, W, 3)
            if 'depth' in obs[cam]:
                cam_data['depth'] = obs[cam]['depth'][:]       # (N, H, W)
            if 'intrinsic_cv' in obs[cam]:
                cam_data['intrinsic'] = obs[cam]['intrinsic_cv'][:]
            if 'extrinsic' in obs[cam]:
                cam_data['extrinsic'] = obs[cam]['extrinsic'][:]
            data['cameras'][cam] = cam_data

        # ── 关节 ──
        for key in ('qpos', 'qvel', 'effort'):
            if key in obs:
                data[key] = obs[key][:]

        # ── 末端位姿 ──
        for key in ('end_pose_left', 'end_pose_right'):
            if key in obs:
                data[key] = obs[key][:]

        # ── 时间戳 ──
        if 'time_stamps' in f:
            data['timestamps'] = f['time_stamps'][:]

        # ── 元信息 ──
        if 'meta_data' in f:
            raw = f['meta_data'][()]
            try:
                data['meta'] = json.loads(raw)
            except Exception:
                data['meta'] = {'raw': str(raw)}

    return data


# ─────────────────── 可视化：OpenCV 图像窗口 ─────────────────────

def build_frame(data, idx, show_depth, cam_order, target_h=360):
    """拼接单帧可视化图像（上排：RGB，下排：深度）。"""
    rgb_strips = []
    dep_strips = []
    has_depth = False

    for cam in cam_order:
        if cam not in data['cameras']:
            continue
        cd = data['cameras'][cam]

        # RGB
        if 'rgb' in cd:
            rgb = cd['rgb'][idx]              # (H, W, 3) BGR or RGB
            # 若为 RGB，转 BGR
            if rgb.shape[2] == 3:
                rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            rgb = resize_to_height(rgb, target_h)
            put_label(rgb, cam)
            rgb_strips.append(rgb)

        # Depth
        if show_depth and 'depth' in cd:
            dep = colorize_depth(cd['depth'][idx])
            dep = resize_to_height(dep, target_h)
            put_label(dep, f"{cam} depth", color=(0, 255, 255))
            dep_strips.append(dep)
            has_depth = True

    if not rgb_strips:
        blank = np.zeros((target_h, 640, 3), dtype=np.uint8)
        return blank

    top = np.hstack(rgb_strips)

    if has_depth and dep_strips:
        # 深度图与 RGB 等宽对齐
        bot = np.hstack(dep_strips)
        # 若宽度不同，填充到相同宽度
        tw, bw = top.shape[1], bot.shape[1]
        if tw != bw:
            pad_w = abs(tw - bw)
            pad = np.zeros((target_h, pad_w, 3), dtype=np.uint8)
            if tw < bw:
                top = np.hstack([top, pad])
            else:
                bot = np.hstack([bot, pad])
        frame = np.vstack([top, bot])
    else:
        frame = top

    return frame


def overlay_info(frame, data, idx, total, fps, paused):
    """在画面左上角叠加帧号/时间/状态等信息。"""
    h, w = frame.shape[:2]
    ts_arr = data.get('timestamps', None)
    ts_str = ""
    if ts_arr is not None and idx < len(ts_arr):
        ts_str = f"  t={ts_arr[idx]:.3f}s"

    lines = [
        f"Frame {idx+1}/{total}{ts_str}",
        f"FPS={fps}  {'[PAUSED]' if paused else '[PLAY]'}",
        "Space:pause/play  ←→:step  q:quit  s:save",
    ]
    y = h - 10 - 18 * (len(lines) - 1)
    for line in lines:
        cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (200, 255, 200), 1, cv2.LINE_AA)
        y += 18
    return frame


# ─────────────────── 可视化：Matplotlib 曲线图 ───────────────────

def plot_timeseries(data, save_path=None):
    """绘制关节角度、末端位姿的时序曲线（静态图）。"""
    timestamps = data.get('timestamps', None)

    has_qpos  = 'qpos'  in data
    has_epl   = 'end_pose_left'  in data
    has_epr   = 'end_pose_right' in data

    n_plots = sum([has_qpos, has_epl, has_epr])
    if n_plots == 0:
        print("[WARN] 没有关节/位姿数据，跳过时序图。")
        return

    fig, axes = plt.subplots(n_plots, 1, figsize=(14, 4 * n_plots), sharex=True)
    if n_plots == 1:
        axes = [axes]

    meta = data.get('meta', {})
    title = meta.get('task_name', '') or ''
    if meta.get('uuid'):
        title += f"  [{meta['uuid']}]"
    fig.suptitle(title or 'HDF5 Episode', fontsize=13)

    ax_idx = 0
    N = None

    if has_qpos:
        qpos = data['qpos']       # (N, 14)
        N = qpos.shape[0]
        x = timestamps[1:N+1] - timestamps[1] if timestamps is not None else np.arange(N)
        ax = axes[ax_idx]; ax_idx += 1
        n_dof = qpos.shape[1]
        half = n_dof // 2
        for j in range(half):
            ax.plot(x, qpos[:, j],       label=f'L_j{j+1}', lw=1.2)
        for j in range(half, n_dof):
            ax.plot(x, qpos[:, j], '--', label=f'R_j{j-half+1}', lw=1.2)
        ax.set_ylabel('qpos (rad)')
        ax.legend(ncol=4, fontsize=7, loc='upper right')
        ax.grid(True, alpha=0.3)

    if has_epl:
        epl = data['end_pose_left']   # (N, 7)  x y z qx qy qz qw
        N = epl.shape[0]
        x = timestamps[1:N+1] - timestamps[1] if timestamps is not None else np.arange(N)
        ax = axes[ax_idx]; ax_idx += 1
        labels = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
        for j, lb in enumerate(labels[:3]):
            ax.plot(x, epl[:, j], label=lb, lw=1.2)
        ax.set_ylabel('end_pose_left (m / quat)')
        ax2 = ax.twinx()
        for j, lb in enumerate(labels[3:], start=3):
            ax2.plot(x, epl[:, j], '--', label=lb, lw=1.0, alpha=0.7)
        ax.legend(loc='upper left', fontsize=7)
        ax2.legend(loc='upper right', fontsize=7)
        ax.grid(True, alpha=0.3)

    if has_epr:
        epr = data['end_pose_right']
        N = epr.shape[0]
        x = timestamps[1:N+1] - timestamps[1] if timestamps is not None else np.arange(N)
        ax = axes[ax_idx]; ax_idx += 1
        labels = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
        for j, lb in enumerate(labels[:3]):
            ax.plot(x, epr[:, j], label=lb, lw=1.2)
        ax.set_ylabel('end_pose_right (m / quat)')
        ax2 = ax.twinx()
        for j, lb in enumerate(labels[3:], start=3):
            ax2.plot(x, epr[:, j], '--', label=lb, lw=1.0, alpha=0.7)
        ax.legend(loc='upper left', fontsize=7)
        ax2.legend(loc='upper right', fontsize=7)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('time (s)' if timestamps is not None else 'frame')
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=120)
        print(f"[INFO] 时序图已保存 → {save_path}")
    else:
        plt.show(block=False)
        plt.pause(0.1)


# ──────────────────────── 主播放循环 ─────────────────────────────

def play(data, args):
    cam_order_default = ['camera_middle', 'camera_left', 'camera_right']
    # 取实际有 rgb 数据的相机，按上述顺序排列
    available_cams = [c for c in cam_order_default if c in data['cameras'] and 'rgb' in data['cameras'][c]]
    # 补充不在默认顺序中的相机
    for c in data['cameras']:
        if c not in available_cams and 'rgb' in data['cameras'][c]:
            available_cams.append(c)

    if not available_cams:
        print("[ERROR] HDF5 中没有找到 RGB 图像数据。")
        return

    # 帧总数
    total = data['cameras'][available_cams[0]]['rgb'].shape[0]
    print(f"[INFO] 总帧数: {total}  相机: {available_cams}")

    meta = data.get('meta', {})
    print(f"[INFO] 任务: {meta.get('task_name','')}  指令: {meta.get('instruction','')}")

    win_name = "HDF5 Visualizer  (Space:pause  ←→:step  q:quit  s:save)"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)

    # 导出视频
    writer = None
    if args.export_video:
        first_frame = build_frame(data, 0, args.show_depth, available_cams)
        fh, fw = first_frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(args.export_video, fourcc, args.fps, (fw, fh))
        print(f"[INFO] 导出视频 → {args.export_video}")

    idx = 0
    paused = False
    frame_ms = max(1, int(1000 / args.fps))

    while True:
        frame = build_frame(data, idx, args.show_depth, available_cams)
        overlay_info(frame, data, idx, total, args.fps, paused)

        cv2.imshow(win_name, frame)

        if writer:
            writer.write(frame)

        key = cv2.waitKey(1 if paused else frame_ms) & 0xFF

        if key == ord('q') or key == 27:        # q / ESC
            break
        elif key == ord(' '):                    # Space
            paused = not paused
        elif key == 83 or key == ord('d'):       # → / d
            idx = min(idx + 1, total - 1)
            paused = True
        elif key == 81 or key == ord('a'):       # ← / a
            idx = max(idx - 1, 0)
            paused = True
        elif key == ord('s'):                    # s → 保存帧
            save_path = f"frame_{idx:05d}.png"
            cv2.imwrite(save_path, frame)
            print(f"[INFO] 帧已保存 → {save_path}")
        elif key == ord('r'):                    # r → 重置
            idx = 0

        if not paused:
            idx += 1
            if idx >= total:
                if args.loop:
                    idx = 0
                else:
                    break

    cv2.destroyAllWindows()
    if writer:
        writer.release()
        print(f"[INFO] 视频导出完成 → {args.export_video}")


# ─────────────────────────── 入口 ────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(description='HDF5 机器人数据可视化')
    parser.add_argument('--hdf5', type=str, required=True,
                        help='HDF5 文件路径，例如 episode_0.hdf5')
    parser.add_argument('--fps', type=int, default=10,
                        help='回放帧率（默认 10）')
    parser.add_argument('--show_depth', action='store_true',
                        help='同时显示深度图（伪彩色）')
    parser.add_argument('--plot', action='store_true',
                        help='额外绘制关节角度与末端位姿时序曲线')
    parser.add_argument('--export_video', type=str, default='',
                        help='将回放导出为 mp4 文件（不为空时启用）')
    parser.add_argument('--loop', action='store_true',
                        help='循环播放')
    parser.add_argument('--info_only', action='store_true',
                        help='只打印 HDF5 结构，不播放')
    return parser.parse_args()


def print_info(path):
    """打印 HDF5 文件结构。"""
    print(f"\n{'='*60}")
    print(f"  HDF5 结构: {path}")
    print(f"{'='*60}")
    with h5py.File(path, 'r') as f:
        def _show(name, obj):
            indent = '  ' * name.count('/')
            if isinstance(obj, h5py.Dataset):
                print(f"{indent}{name.split('/')[-1]}: shape={obj.shape}, dtype={obj.dtype}")
            elif isinstance(obj, h5py.Group):
                print(f"{indent}{name.split('/')[-1]}/ (Group)")
        f.visititems(_show)

        if 'meta_data' in f:
            try:
                meta = json.loads(f['meta_data'][()])
                print(f"\n{'─'*40}")
                print("  Meta:")
                for k, v in meta.items():
                    print(f"    {k}: {v}")
            except Exception:
                pass
    print(f"{'='*60}\n")


def main():
    args = parse_args()

    if not os.path.isfile(args.hdf5):
        print(f"[ERROR] 文件不存在: {args.hdf5}")
        sys.exit(1)

    print_info(args.hdf5)

    if args.info_only:
        return

    print("[INFO] 正在加载数据...")
    data = load_hdf5(args.hdf5)
    print("[INFO] 数据加载完成。")

    if args.plot:
        plot_timeseries(data)

    play(data, args)


if __name__ == '__main__':
    main()
