import os
import json
import numpy as np
import open3d as o3d
import argparse
from tqdm import tqdm

def load_extrinsic(path):
    """Load 4x4 extrinsic matrix.
    - empty path: identity
    - json: expect key 'matrix' (list of 16 or 4x4)
    - npy: np.load
    - txt: plain numbers (len 16) or 4 lines of 4 numbers (如 result/refine_650.txt)
    """
    if not path:
        return np.eye(4)
    if path.endswith(".npy"):
        mat = np.load(path)
        return np.array(mat).reshape(4, 4)
    if path.endswith(".json"):
        with open(path, "r") as f:
            data = json.load(f)
        if "matrix" in data:
            arr = data["matrix"]
        else:
            raise ValueError("JSON extrinsic should contain key 'matrix'")
        return np.array(arr).reshape(4, 4)
    # txt or others: parse numbers
    with open(path, "r") as f:
        tokens = []
        for line in f:
            tokens.extend([t for t in line.strip().replace(",", " ").split() if t])
    if len(tokens) != 16:
        raise ValueError(f"Extrinsic file {path} should contain 16 numbers, got {len(tokens)}")
    nums = [float(x) for x in tokens]
    return np.array(nums).reshape(4, 4)

def load_poses(pose_file_path):
    """
    从指定的pose文件中加载位姿。

    Args:
        pose_file_path (str): 位姿文件的路径。
                               格式: timestamp r11 r12 r13 t1 r21 r22 r23 t2 r31 r32 r33 t3

    Returns:
        dict: 一个字典，键是时间戳字符串，值是4x4的numpy变换矩阵。
    """
    poses = {}
    print(f"Loading poses from: {pose_file_path}")
    with open(pose_file_path, 'r') as f:
        i = -1
        for line in f:
            i = i + 1
            # if i % 20 != 0:
            #     continue
            parts = line.strip().split()
            if len(parts) != 13:
                print(f"Warning: Skipping malformed line with {len(parts)} parts.")
                continue
            
            timestamp = parts[0]
            
            # 将12个数字的变换矩阵部分转换为浮点数
            matrix_elements = [float(x) for x in parts[1:]]
            
            # 构建4x4的齐次变换矩阵
            transform_matrix = np.identity(4)
            transform_matrix[0:3, :] = np.array(matrix_elements).reshape(3, 4)
            
            poses[timestamp] = transform_matrix
            
    print(f"Successfully loaded {len(poses)} poses.")
    return poses

def main(args):
    """
    主执行函数
    """
    # 1. 加载所有位姿
    try:
        poses_dict = load_poses(args.pose_file)
        if not poses_dict:
            print("Error: No poses were loaded. Please check the pose file.")
            return
    except FileNotFoundError:
        print(f"Error: Pose file not found at '{args.pose_file}'")
        return

    # 2. 初始化一个用于拼接的大点云
    stitched_cloud = o3d.geometry.PointCloud()

    extrinsc = load_extrinsic(args.extrinsc)
        
    
    # 3. 遍历所有位姿，读取对应的PCD文件，进行变换和拼接
    print("Stitching point clouds...")
    # 使用tqdm创建进度条
    for timestamp, transform in tqdm(poses_dict.items(), desc="Processing PCDs"):
        pcd_filename = f"{timestamp}.pcd"
        pcd_path = os.path.join(args.pcd_dir, pcd_filename)
        
        if not os.path.exists(pcd_path):
            # print(f"Warning: PCD file not found for timestamp {timestamp}, skipping.")
            continue
            
        try:
            # 读取单帧点云
            pcd = o3d.io.read_point_cloud(pcd_path)
            
            # （可选）降采样以减少内存消耗和加快处理速度
            if args.voxel_size > 0:
                pcd = pcd.voxel_down_sample(voxel_size=args.voxel_size)

            if not pcd.has_points():
                continue

            # 应用位姿变换
            pcd.transform(extrinsc)
            pcd.transform(transform)
            
            # 将变换后的点云拼接到大点云中
            stitched_cloud += pcd
            
        except Exception as e:
            print(f"Error processing {pcd_filename}: {e}")

    # 4. 保存最终结果（可选终端下采样）
    if not stitched_cloud.has_points():
        print("Error: The final stitched cloud has no points. Please check your data and paths.")
        return

    print("\nStitching complete.")
    print(f"Total points before final downsample: {len(stitched_cloud.points)}")
    pts = np.asarray(stitched_cloud.points)
    mask = np.isfinite(pts).all(axis=1)
    stitched_cloud.points = o3d.utility.Vector3dVector(pts[mask])
    if args.voxel_save > 0:
        stitched_cloud = stitched_cloud.voxel_down_sample(args.voxel_save)
        print(f"Total points after final downsample (voxel={args.voxel_save}): {len(stitched_cloud.points)}")
    
    print(f"Saving the final stitched map to: {args.output_file}")
    o3d.io.write_point_cloud(args.output_file, stitched_cloud, write_ascii=False)
    print("Done.")

if __name__ == "__main__":
    # 设置命令行参数解析
    parser = argparse.ArgumentParser(description="Stitch multiple PCD files into a single map using a pose file.")
    parser.add_argument("pose_file", help="Path to the pose file (format: timestamp r11..t3).")
    parser.add_argument("pcd_dir", help="Path to the directory containing the PCD files.")
    parser.add_argument("output_file", help="Path to save the final stitched PCD file.")
    parser.add_argument("--voxel_size", type=float, default=0.2, 
                        help="Voxel size for downsampling each PCD before stitching. Set to 0 to disable. Default: 0.1 meters.")
    parser.add_argument("--voxel_save", type=float, default=0, 
                        help="Final voxel size when saving stitched map. 0 to disable.")
    parser.add_argument("--extrinsc", type=str, default="", help="extrinsc file.")
    
    args = parser.parse_args()
    main(args)
