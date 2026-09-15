#!/usr/bin/env python3
"""从凳子网格里切出「只有坐面」的工件。

判据：一个面若三个顶点全部在 z >= THR 之上，就属于坐面板。
z=0.279 处有明确断层（坐面底面），腿在 0.087~0.269，坐面 0.279~0.317。
"""
import numpy as np

SRC = '/workspace/snp_home/snp/meshes/results_mesh.ply'
DST = '/workspace/snp_home/seat_only.ply'
THR = 0.27


def read_ply(p):
    with open(p) as f:
        lines = f.read().split('\n')
    i = 0
    nv = nf = 0
    while not lines[i].startswith('end_header'):
        if lines[i].startswith('element vertex'):
            nv = int(lines[i].split()[-1])
        if lines[i].startswith('element face'):
            nf = int(lines[i].split()[-1])
        i += 1
    body = [l for l in lines[i + 1:] if l.strip()]
    V = np.array([[float(x) for x in body[j].split()[:3]] for j in range(nv)])
    F = np.array([[int(x) for x in body[nv + j].split()[1:4]] for j in range(nf)])
    return V, F


V, F = read_ply(SRC)
print('原网格: %d 顶点 / %d 面  z[%.3f,%.3f]' % (len(V), len(F), V[:, 2].min(), V[:, 2].max()))

keep = (V[F][:, :, 2] >= THR).all(axis=1)
F2 = F[keep]
used = np.unique(F2)
remap = -np.ones(len(V), dtype=int)
remap[used] = np.arange(len(used))
V2 = V[used]
F2 = remap[F2]

print('切出:   %d 顶点 / %d 面' % (len(V2), len(F2)))
print('bbox min %s  max %s' % (np.round(V2.min(0), 3), np.round(V2.max(0), 3)))
print('尺寸 %.3f x %.3f x %.3f' % tuple(V2.max(0) - V2.min(0)))

with open(DST, 'w') as f:
    f.write('ply\nformat ascii 1.0\n')
    f.write('element vertex %d\n' % len(V2))
    f.write('property float x\nproperty float y\nproperty float z\n')
    f.write('element face %d\n' % len(F2))
    f.write('property list uchar int vertex_indices\n')
    f.write('end_header\n')
    for v in V2:
        f.write('%.6f %.6f %.6f\n' % (v[0], v[1], v[2]))
    for fc in F2:
        f.write('3 %d %d %d\n' % (fc[0], fc[1], fc[2]))
print('已写出', DST)
