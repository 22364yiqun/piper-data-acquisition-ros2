"""
HDF5文件结构查看工具

该脚本用于查看HDF5文件的内部结构，包括：
1. 所有数据集(Dataset)的名称、形状和数据类型
2. 所有组(Group)的层级结构

HDF5文件结构说明：
- Group (组)：类似于文件系统中的文件夹，用于组织数据结构
- Dataset (数据集)：存储实际数据的多维数组

使用示例：
python hdf5_viewer.py

输出格式：
- 数据集：<名称>: shape=<形状>, dtype=<数据类型>
- 组：<名称>/ (Group)

"""

import h5py

# 打开 hdf5 文件 (只读模式)
with h5py.File("/home/yiqun/code/piper-data-acquisition-ros2/episode_3.hdf5", "r") as f:
    # 遍历文件中的所有对象
    def print_hdf5(name, obj):
        if isinstance(obj, h5py.Dataset):
            print(f"{name}: shape={obj.shape}, dtype={obj.dtype}")
        elif isinstance(obj, h5py.Group):
            print(f"{name}/ (Group)")

    f.visititems(print_hdf5)
