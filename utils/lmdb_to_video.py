# -*- coding: utf-8 -*-
"""
把 LMDB 中的所有 episode 逐个导出为三路视频：
OUT_DIR / EPISODE / {left|middle|right}.avi
"""

import lmdb
import pickle
import cv2
import numpy as np
import os
import re
from collections import defaultdict

# ===== 路径与参数 =====
LMDB_PATH = r"E:\挑战杯\初始数据\lmdb_dataset_place_shoe_2025_08_21\image"
OUT_DIR   = r"E:\挑战杯\初始数据\vision\lmdb_dataset_place_shoe_2025_08_21"
FPS       = 10

os.makedirs(OUT_DIR, exist_ok=True)

# ===== 解码函数（健壮） =====
def decode_image(value_bytes):
    """把 LMDB 的 value 解成 BGR uint8 (H,W,3)。"""
    img = None
    try:
        data = pickle.loads(value_bytes)
        if isinstance(data, np.ndarray):
            img = data
        else:
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    except Exception:
        img = cv2.imdecode(np.frombuffer(value_bytes, np.uint8), cv2.IMREAD_COLOR)

    if img is None:
        raise ValueError("imdecode 返回 None（既不是 pickle ndarray，也不是有效图片编码）")

    if img.dtype != np.uint8:
        img = np.clip(img, 0, 255).astype(np.uint8)
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    elif img.ndim == 3 and img.shape[2] == 4:
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return img

# ===== 打开 VideoWriter（带回退） =====
def open_writer(out_path, size, fps):
    for cc in ("XVID", "MJPG", "MP4V"):
        fourcc = cv2.VideoWriter_fourcc(*cc)
        writer = cv2.VideoWriter(out_path, fourcc, fps, size)
        if writer.isOpened():
            return writer, cc
    return None, None

# ===== ❶ 一次性扫描：收集所有 episode/cam/fid 对应的原始 key（bytes） =====
# 形如： .../episode_xxx/(left|middle|right)/<fid>
pat_all = re.compile(rb'(episode_[^\\/]+)[\\/](left|middle|right)[\\/](\d+)$')

# 保存：frame_keys_all[episode][cam][fid] = original_key_bytes
frame_keys_all = defaultdict(lambda: defaultdict(dict))

print("🔎 正在扫描 LMDB 键并建立索引（仅一次）...")
with lmdb.open(LMDB_PATH, readonly=True, lock=False, readahead=False) as env:
    with env.begin() as txn:
        cur = txn.cursor()
        for k_bytes, _ in cur:
            m = pat_all.search(k_bytes)
            if not m:
                continue
            episode = m.group(1).decode('utf-8')
            cam     = m.group(2).decode('utf-8')      # left/middle/right
            fid     = int(m.group(3).decode('utf-8')) # 帧号
            frame_keys_all[episode][cam][fid] = k_bytes

episode_list = sorted(frame_keys_all.keys())
print(f"📦 共发现 {len(episode_list)} 个 episode。")

if not episode_list:
    raise RuntimeError("没有在 LMDB 中找到任何 episode_* 键，请检查路径与数据。")

# ===== ❷ 逐个 episode 导出三路视频 =====
with lmdb.open(LMDB_PATH, readonly=True, lock=False, readahead=False) as env:
    with env.begin() as txn:
        for EPISODE in episode_list:
            print(f"\n🎯 处理 episode: {EPISODE}")
            ep_dir = os.path.join(OUT_DIR, EPISODE)      # 每个 episode 一个子目录
            os.makedirs(ep_dir, exist_ok=True)

            frame_keys = frame_keys_all[EPISODE]         # dict[cam][fid] -> key_bytes

            for cam in ("left", "middle", "right"):
                if not frame_keys.get(cam):
                    print(f"  ⚠️ 没有找到 {EPISODE} - {cam} 的数据，跳过。")
                    continue

                ids = sorted(frame_keys[cam].keys())
                first_key_bytes = frame_keys[cam][ids[0]]
                first_val = txn.get(first_key_bytes)
                if first_val is None:
                    print(f"  ⚠️ 找不到首帧 {first_key_bytes!r}，跳过 {cam}")
                    continue

                try:
                    first_img = decode_image(first_val)
                except Exception as e:
                    print(f"  ⚠️ 首帧解码失败 {cam}: {e}，跳过。")
                    continue

                H, W = first_img.shape[:2]
                out_path = os.path.join(ep_dir, f"{cam}.avi")
                writer, codec = open_writer(out_path, (W, H), FPS)
                if writer is None:
                    raise RuntimeError("无法打开 VideoWriter（XVID/MJPG/MP4V 都失败）")

                # 写首帧
                writer.write(first_img)

                drop_cnt = 0
                # 写后续帧
                for fid in ids[1:]:
                    k_bytes = frame_keys[cam][fid]
                    v = txn.get(k_bytes)
                    if v is None:
                        drop_cnt += 1
                        continue
                    try:
                        img = decode_image(v)
                    except Exception as e:
                        print(f"  解码失败 {k_bytes!r}: {e}")
                        drop_cnt += 1
                        continue
                    if img.shape[:2] != (H, W):
                        img = cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA)
                    writer.write(img)

                writer.release()
                print(f"  ✅ 生成 {cam}.avi（编码器：{codec}，写入 {len(ids)-drop_cnt} 帧，丢弃 {drop_cnt} 帧）")

print("\n🎉 全部完成。输出根目录：", OUT_DIR)

