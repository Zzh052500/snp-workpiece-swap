#!/usr/bin/env python3
"""通用刀路测试：tpptest.py <config.yaml> <mesh.ply>
   打印每条路径的点数/包围盒、以及相邻点是否有跳变。"""
import math
import sys

import rclpy
from rclpy.node import Node

from noether_ros.srv import PlanToolPath

cfg = sys.argv[1]
mesh = sys.argv[2]

rclpy.init()
node = Node('tpptest')
cli = node.create_client(PlanToolPath, '/plan_tool_path')
if not cli.wait_for_service(timeout_sec=30):
    print('!! /plan_tool_path 不可用')
    sys.exit(1)

req = PlanToolPath.Request()
req.config = cfg
req.mesh_file = mesh
req.mesh_frame = 'base_link'
fut = cli.call_async(req)
rclpy.spin_until_future_complete(node, fut, timeout_sec=300)
res = fut.result()
if res is None:
    print('!! 超时')
    sys.exit(1)

print('config =', cfg)
print('mesh   =', mesh)
print('success =', res.success, ' message =', repr(res.message))
print()

paths = []
for group in res.tool_paths:
    for path in group.tool_paths:
        pts = []
        for seg in path.segments:
            for p in seg.poses:
                pts.append((p.position.x, p.position.y, p.position.z))
        paths.append(pts)

if not paths:
    print('>>> 0 条路径')
    sys.exit(2)

tot = 0
for i, pts in enumerate(paths):
    n = len(pts)
    tot += n
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    jumps = [math.dist(pts[k], pts[k + 1]) for k in range(n - 1)]
    big = sum(1 for d in jumps if d > 0.05)
    print('路径%d: %3d点  x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]  跳变%d'
          % (i, n, min(xs), max(xs), min(ys), max(ys), min(zs), max(zs), big))
print()
print('合计 %d 条路径 / %d 点' % (len(paths), tot))
rclpy.shutdown()
