#!/usr/bin/env bash
# 收敛性判定 —— 剩下的唯一关键未知量。
#
# 问题：运动规划要吃 ~2.1GB。这 2.1GB 是「有界的一次性开销」还是「无界的泄漏」？
#   有界  -> 把容器上限调大就能跑通，换大内存机器即可解决
#   无界  -> 换机器也没用
#
# 做法：把容器上限临时调高（docker update，可逆、不用重建），跑一次规划，看 RSS 曲线
#       末段斜率是否趋近 0。同时打印每段斜率供判断。
#
# 用法: converge.sh [上限]     默认 32g
#
# ⚠️ 2026-09-16 更正：默认值从 16g 提到 32g。
#    本机实测 6g 时节点 8 秒涨到 5.2GB 才被截断、斜率毫无收敛迹象（README §4.12），
#    说明 16g 很可能照样撞墙、白跑一轮。宁可给大，不要给小。
#
# ⚠️ 本机（7.5GB 内存、swap 已用 1.7GB）**不要**用大上限，会把整机拖死。
#    判定实验必须上服务器（内存 ≥32GB）做。
set -uo pipefail

C=snp_automate_2023_sim
LIM=${1:-32g}
SIM="${SNP_SIM_DIR:-$HOME/snp-automate-2023-polishing-simulation}"
H="$SIM/runtime/snp_home"
CFG=/opt/snp_automate_2023/install/snp_automate_2023/share/snp_automate_2023/config/tpp.yaml
MESHC=/workspace/snp_home/snp/meshes/results_mesh.ply
LOG="$H/converge.log"

node_alive() {
  docker exec $C bash -lc 'for d in /proc/[0-9]*; do [ "$(cat $d/comm 2>/dev/null)" = "snp_motion_plan" ] && exit 0; done; exit 1'
}

echo "=== 主机余量检查 ==="
free -h | sed 's/^/  /'
AVAIL_MB=$(free -m | awk '/Mem:/{print $7}')
echo "  可用 ${AVAIL_MB} MiB；请求上限 $LIM"

echo
echo "=== [1] 调高容器上限 -> $LIM ==="
docker update --memory "$LIM" --memory-swap "$LIM" $C >/dev/null && echo "  ok"
docker inspect $C --format '  现在 Memory={{.HostConfig.Memory}} MemorySwap={{.HostConfig.MemorySwap}}'

echo
echo "=== [2] 重启并等节点 ==="
docker restart $C >/dev/null
for i in $(seq 1 60); do
  if node_alive; then echo "  第 $((i*2))s 就绪"; break; fi
  sleep 2
done
node_alive || { echo "  !! 节点起不来"; exit 1; }
sleep 8

echo
echo "=== [3] 起监视器（0.2s 粒度）==="
docker exec $C bash -lc "rm -f /workspace/snp_home/mon.pid; echo '=== converge $LIM ===' > /workspace/snp_home/converge.log"
docker exec -d $C bash -lc "echo \$\$ > /workspace/snp_home/mon.pid; exec bash /workspace/snp_home/mon.sh /workspace/snp_home/converge.log"
sleep 2

echo "=== [4] 开火：7 条路径 / 294 点 ==="
docker exec $C bash -lc "source /opt/ros/jazzy/setup.bash; source /opt/snp/install/setup.bash; export ROS_DOMAIN_ID=42; timeout 900 python3 -u /workspace/snp_home/prechk.py $CFG $MESHC" 2>&1 | grep -vE "运行中|^  File|^    " | tail -22

echo
echo "=== [5] 停监视器 ==="
docker exec $C bash -lc 'kill "$(cat /workspace/snp_home/mon.pid)" 2>/dev/null; echo stopped'

echo
echo "=== [6] 判定 ==="
if node_alive; then echo "  ✓ 节点存活 —— 规划跑通了"; else echo "  ✗ 节点仍然死了（上限 $LIM 也不够，或者无界）"; fi

python3 - "$LOG" <<'PY'
import re, sys
rows = []
for ln in open(sys.argv[1]):
    if '|' not in ln:
        continue
    cur, rest = ln.split('|', 1)
    try:
        cur = int(cur) / 1048576
    except ValueError:
        continue
    d = {}
    for m in re.finditer(r'(\d+)\s+(\d+)\s+(\S+)', rest):
        d[m.group(3)] = int(m.group(1)) / 1024
    rows.append((cur, d))

n = [d['snp_motion_plan'] for _, d in rows if 'snp_motion_plan' in d]
if not n:
    print("  (没采到节点 RSS)")
    raise SystemExit()
print(f"  容器峰值 {max(r[0] for r in rows):.0f} MiB")
print(f"  节点 RSS 起始 {n[0]:.0f} -> 峰值 {max(n):.0f} MiB   增量 +{max(n)-n[0]:.0f} MiB")

# 取单调递增段作为爬升
up = []
for v in n:
    if not up or v > up[-1]:
        up.append(v)
    else:
        break
if len(up) < 6:
    print("  爬升段太短，无法判断收敛性")
    raise SystemExit()
k = len(up) // 3
def slope(seg):
    return (seg[-1] - seg[0]) / max(1, len(seg) - 1) if len(seg) > 1 else 0.0
s1, s2, s3 = slope(up[:k]), slope(up[k:2*k]), slope(up[2*k:])
print(f"  爬升段 {len(up)} 个采样点（0.2s 一个）")
print(f"    前 1/3 斜率 = {s1:+7.1f} MiB/采样")
print(f"    中 1/3 斜率 = {s2:+7.1f} MiB/采样")
print(f"    后 1/3 斜率 = {s3:+7.1f} MiB/采样")
print("  曲线: " + " ".join(f"{v:.0f}" for v in up[-18:]))
print()
if s3 < 0.25 * max(s1, 1e-9):
    print("  >>> 末段斜率已坍缩到首段的 1/4 以下 —— 强烈提示【有界】，换大内存机器可解。")
elif s3 < 0.7 * max(s1, 1e-9):
    print("  >>> 末段斜率在下降但未坍缩 —— 疑似【有界但很大】，继续调大上限再试。")
else:
    print("  >>> 斜率基本没降 —— 更像【无界】。换机器也救不了，需要改流水线（摘 DiscreteContactCheckTask）。")
PY

echo
echo "=== [7] 还原上限 ==="
# 还原到什么由 SNP_MEM_LIMIT 决定；不设则按小内存机器的安全值 3g。
# 在大内存机器上想还原成「不限制」，先 export SNP_MEM_LIMIT=0。
RESTORE="${SNP_MEM_LIMIT:-3g}"
# docker 的 0 表示不限制
docker update --memory "$RESTORE" --memory-swap "$RESTORE" $C >/dev/null \
  && echo "  已还原为 $RESTORE"$([ "$RESTORE" = 0 ] && echo "（= 不限制）")
docker inspect $C --format '  现在 Memory={{.HostConfig.Memory}} MemorySwap={{.HostConfig.MemorySwap}}'
