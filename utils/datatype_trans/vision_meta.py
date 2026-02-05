import lmdb
import pickle
import json
import numpy as np

# LMDB 数据库路径 (meta 文件夹)
meta_path = r"G:\new_dataset\lmdb_dataset_place_shoe_2025_09_11\meta"

# 要导出的 episode 名称
episode_id = "place_shoe/yuhang.zhou/episode_2025_09_11-17_49_54"

# 输出文件路径
out_file = f"{episode_id.replace('/', '_')}.json"

env = lmdb.open(meta_path, readonly=True, lock=False)

episode_data = {}

with env.begin() as txn:
    cursor = txn.cursor()
    for key, value in cursor:
        k = key.decode("utf-8", errors="ignore")
        if episode_id in k:   # 只筛选目标 episode
            try:
                v = pickle.loads(value)
            except Exception:
                try:
                    v = value.decode("utf-8")
                except Exception:
                    v = value
            episode_data[k] = v

# 定义 numpy -> list 的转换
def json_default(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.float32, np.float64)):
        return float(o)
    if isinstance(o, (np.int32, np.int64)):
        return int(o)
    return str(o)

# 存 JSON
with open(out_file, "w", encoding="utf-8") as f:
    json.dump(episode_data, f, indent=2, ensure_ascii=False, default=json_default)

print(f"✅ 已导出 episode {episode_id} 到 {out_file}")
