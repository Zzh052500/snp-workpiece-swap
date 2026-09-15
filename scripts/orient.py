#!/usr/bin/env python3
"""对比不同 tpp.yaml 生成的刀路「姿态一致性」。

用法: orient.py <tpp配置路径> <标签>
判据：每条路径上刀具轴（姿态矩阵第三列）的标准差。打磨要求它接近 0
（整条路径刀具朝向稳定）；大了就说明姿态在乱翻，机器人必然够不着。
"""
import math
import sys

import rclpy
from rclpy.node import Node

from noether_ros.srv import PlanToolPath

CFG = sys.argv[1]
TAG = sys.argv[2]
MESH = sys.argv[3] if len(sys.argv) > 3 else '/workspace/snp_home/snp/meshes/results_mesh.ply'


def tool_z(q):
    """四元数 -> 旋转矩阵第三列（刀具局部 z 轴在世界系下的方向）"""
    x, y, z, w = q
    return (2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y))


rclpy.init()
node = Node('orient')
cli = node.create_client(PlanToolPath, '/plan_tool_path')
if not cli.wait_for_service(timeout_sec=30):
    print('!! /plan_tool_path 不可用')
    raise SystemExit(1)

req = PlanToolPath.Request()
req.config = CFG
req.mesh_file = MESH
req.mesh_frame = 'base_link'
fut = cli.call_async(req)
rclpy.spin_until_future_complete(node, fut, timeout_sec=300)
res = fut.result()
if res is None or not res.success:
    print(f'!! [{TAG}] 失败:', None if res is None else res.message)
    raise SystemExit(1)

print(f'=== {TAG} ===')
tot_pts = 0
worst = 0.0
allz = []
for gi, group in enumerate(res.tool_paths):
    for i, path in enumerate(group.tool_paths):
        zs = []
        for seg in path.segments:
            for p in seg.poses:
                q = p.orientation
                zs.append(tool_z((q.x, q.y, q.z, q.w)))
        n = len(zs)
        tot_pts += n
        if n == 0:
            continue
        mx = sum(a for a, _, _ in zs) / n
        my = sum(b for _, b, _ in zs) / n
        mz = sum(c for _, _, c in zs) / n
        sd = math.sqrt(sum((a - mx) ** 2 + (b - my) ** 2 + (c - mz) ** 2 for a, b, c in zs) / n)
        allz.extend(zs)
        worst = max(worst, sd)
        flag = '  <<< 姿态乱' if sd > 0.05 else ''
        print(f'  路径{i}: {n:3d}点  刀具轴均值({mx:+.3f},{my:+.3f},{mz:+.3f})  标准差={sd:.4f}{flag}')

if allz:
    n = len(allz)
    mx = sum(a for a, _, _ in allz) / n
    my = sum(b for _, b, _ in allz) / n
    mz = sum(c for _, _, c in allz) / n
    sd = math.sqrt(sum((a - mx) ** 2 + (b - my) ** 2 + (c - mz) ** 2 for a, b, c in allz) / n)
    print(f'  总计 {tot_pts} 点   全局刀具轴({mx:+.3f},{my:+.3f},{mz:+.3f})  全局标准差={sd:.4f}  最差单条={worst:.4f}')
print()
rclpy.shutdown()
