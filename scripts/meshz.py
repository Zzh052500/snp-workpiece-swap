#!/usr/bin/env python3
"""看网格的 z 分布，确定坐面和腿的分界高度，为「只保留坐面」做准备。"""
import numpy as np

PATH = '/workspace/snp_home/snp/meshes/results_mesh.ply'


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


V, F = read_ply(PATH)
print('顶点 %d  面 %d' % (len(V), len(F)))
print('bbox min %s  max %s' % (np.round(V.min(0), 3), np.round(V.max(0), 3)))
print()

h, edges = np.histogram(V[:, 2], bins=24)
for c, e in zip(h, edges):
    bar = '#' * min(int(c / 40), 50)
    print('  z=%.3f  %-50s %d' % (e, bar, c))
print()

for zlo, zhi in [(0.08, 0.10), (0.20, 0.22), (0.27, 0.29), (0.30, 0.322)]:
    m = (V[:, 2] >= zlo) & (V[:, 2] < zhi)
    n = int(m.sum())
    if n:
        dx = V[m, 0].max() - V[m, 0].min()
        dy = V[m, 1].max() - V[m, 1].min()
        print('z in [%.3f,%.3f)  %4d点  x[%.3f,%.3f] y[%.3f,%.3f]  x跨度%.3f y跨度%.3f'
              % (zlo, zhi, n, V[m, 0].min(), V[m, 0].max(),
                 V[m, 1].min(), V[m, 1].max(), dx, dy))
print()

# 逐面统计：面的最大 z，用来判断这个面属于坐面还是腿
fzmax = V[F, 2].max(axis=1)
fzmin = V[F, 2].min(axis=1)
for thr in (0.20, 0.25, 0.27, 0.28, 0.29):
    keep = fzmin >= thr
    nk = int(keep.sum())
    if nk:
        Vk = V[F[keep]].reshape(-1, 3)
        f = F[keep]
        print('只保留全部顶点 z>=%.2f 的面: %d 面  x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'
              % (thr, nk, Vk[:, 0].min(), Vk[:, 0].max(),
                 Vk[:, 1].min(), Vk[:, 1].max(), Vk[:, 2].min(), Vk[:, 2].max()))
    else:
        print('只保留全部顶点 z>=%.2f 的面: 0 面' % thr)
