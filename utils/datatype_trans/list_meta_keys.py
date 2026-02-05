import lmdb
import pickle
import json
import numpy as np

meta_path = "/home/yiqun/data/lmdb_dataset/meta"  # meta 文件夹路径
out_file = "meta_dump.json"

# 定义递归转换函数
def convert_to_jsonable(obj):
    """递归地把 numpy、bytes、复杂类型转换为 JSON 可序列化对象"""
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, (np.float32, np.float64)):
        return float(obj)
    elif isinstance(obj, (np.int32, np.int64)):
        return int(obj)
    elif isinstance(obj, bytes):
        try:
            return obj.decode("utf-8", errors="ignore")
        except Exception:
            return str(obj)
    elif isinstance(obj, dict):
        return {k: convert_to_jsonable(v) for k, v in obj.items()}
    elif isinstance(obj, (list, tuple, set)):
        return [convert_to_jsonable(v) for v in obj]
    elif isinstance(obj, (bool, int, float, str)) or obj is None:
        return obj
    else:
        # 其他类型统一转字符串，防止 json.dump 出错
        return str(obj)

# 打开 LMDB
env = lmdb.open(meta_path, readonly=True, lock=False)

data_dict = {}
with env.begin() as txn:
    cursor = txn.cursor()
    for k, v in cursor:
        key = k.decode("utf-8", errors="ignore")
        try:
            value = pickle.loads(v)
        except Exception:
            try:
                value = v.decode("utf-8", errors="ignore")
            except Exception:
                value = v
        data_dict[key] = convert_to_jsonable(value)

# 写入 JSON 文件
with open(out_file, "w", encoding="utf-8") as f:
    json.dump(data_dict, f, indent=2, ensure_ascii=False)

print(f"✅ 已导出 {len(data_dict)} 个键值对到 {out_file}")
