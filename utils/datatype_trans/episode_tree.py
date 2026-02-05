#!/usr/bin/env python
# -*- coding: utf-8 -*-
#  绘制 episode 的树状结构，带帧数量和标签

import lmdb, os
from collections import defaultdict

# =============== 工具函数 ===============

def get_one_episode_prefix(lmdb_path):
    """从一个 lmdb (比如 image) 中取出一个 episode 的前缀"""
    env = lmdb.open(lmdb_path, readonly=True, lock=False, readahead=False)
    with env.begin() as txn:
        cursor = txn.cursor()
        for k, _ in cursor:   # 取第一条即可
            key_str = k.decode("utf-8")
            parts = key_str.split("/")
            for i, p in enumerate(parts):
                if p.startswith("episode_"):
                    return "/".join(parts[:i+1])  # 返回完整前缀
            return parts[0]
    raise RuntimeError(f"{lmdb_path} 里没有数据")

def explore_episode(lmdb_path, prefix, tag):
    """
    读取某个 episode 的全部键，生成分支结构，并加上来源标签 (RGB/Depth/Index/Meta)
    """
    if not os.path.exists(lmdb_path):
        return defaultdict(set), {}

    env = lmdb.open(lmdb_path, readonly=True, lock=False, readahead=False)
    structure = defaultdict(set)
    frame_counts = {}
    with env.begin() as txn:
        cursor = txn.cursor()
        for key, _ in cursor:
            key_str = key.decode("utf-8")
            if not key_str.startswith(prefix):
                continue
            parts = key_str[len(prefix):].strip("/").split("/")

            if parts and parts[-1].isdigit():
                path = "/".join(parts[:-1])
                # 区分来源 (把标签写进结构里)
                if tag in ["RGB", "Depth"]:
                    path = f"{path} [{tag}]"
                    parts[:-1] = [f"{p} [{tag}]" if i == len(parts)-2 else p 
                                  for i,p in enumerate(parts[:-1])]
                frame_counts[path] = frame_counts.get(path, 0) + 1
                parts[-1] = "<frame>"

            for j in range(1, len(parts)+1):
                structure["/".join(parts[:j-1])].add(parts[j-1])
    return structure, frame_counts

def merge_structures(structs):
    merged = defaultdict(set)
    for s in structs:
        for k, v in s.items():
            merged[k] |= v
    return merged

def merge_frame_counts(counts_list):
    merged = {}
    for counts in counts_list:
        for k, v in counts.items():
            merged[k] = merged.get(k, 0) + v
    return merged

def print_tree(structure, frame_counts, root="", prefix=""):
    """递归打印树状结构，带帧数量和标签"""
    children = sorted(list(structure.get(root, [])))
    for i, child in enumerate(children):
        branch = "└─ " if i == len(children)-1 else "├─ "
        new_root = (root + "/" + child).strip("/")
        label = child
        if child == "<frame>":
            n = frame_counts.get(root, 0)
            label = f"(frames: {n})"
        print(prefix + branch + label)
        new_prefix = prefix + ("   " if i == len(children)-1 else "│  ")
        print_tree(structure, frame_counts, new_root, new_prefix)

# =============== 主程序 ===============

if __name__ == "__main__":
    dataset_root = "/media/yiqun/1DF8-CB6B/new_dataset/lmdb_dataset_place_shoe_2025_09_11"  # 修改为你的数据集根目录

    # 1. 从 image/ 里拿一个 episode 前缀
    ep_prefix = get_one_episode_prefix(os.path.join(dataset_root, "image"))
    print(f"选择 episode 前缀: {ep_prefix}")

    # 2. 遍历 image/depth/index/meta 四个 LMDB，加上标签
    lmdb_parts = [
        ("image", "RGB"),
        ("depth", "Depth"),
        ("index", "Index"),
        ("meta", "Meta")
    ]

    structs, counts_list = [], []
    for part, tag in lmdb_parts:
        path = os.path.join(dataset_root, part)
        s, c = explore_episode(path, ep_prefix, tag)
        structs.append(s)
        counts_list.append(c)

    # 3. 合并
    merged_struct = merge_structures(structs)
    merged_counts = merge_frame_counts(counts_list)

    # 4. 打印分支树
    print("\nEpisode 的 HDF5 分支树：")
    print_tree(merged_struct, merged_counts)
