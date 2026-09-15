#!/bin/bash
# 重启 -> 起 prof.py（抓 maps）-> 开火 -> 停 -> 报告
set -u
C=snp_automate_2023_sim
H="${SNP_SIM_DIR:-$HOME/snp-automate-2023-polishing-simulation}/runtime/snp_home"
CFG=/opt/snp_automate_2023/install/snp_automate_2023/share/snp_automate_2023/config/tpp.yaml
MESHC=/workspace/snp_home/snp/meshes/results_mesh.ply
LOG="$H/prof.log"

node_alive() {
  docker exec $C bash -lc 'for d in /proc/[0-9]*; do [ "$(cat $d/comm 2>/dev/null)" = "snp_motion_plan" ] && exit 0; done; exit 1'
}

echo "=== [1] 重启 ==="
docker restart $C >/dev/null
for i in $(seq 1 40); do
  if node_alive; then echo "  第 $((i*2))s 就绪"; break; fi
  sleep 2
done
sleep 8

echo "=== [2] 起剖析器 ==="
rm -f "$LOG" "$H/prof.pid"
docker exec -d $C bash -lc "echo \$\$ > /workspace/snp_home/prof.pid; exec python3 -u /workspace/snp_home/prof.py /workspace/snp_home/prof.log"
sleep 2

echo "=== [3] 开火 ==="
docker exec $C bash -lc "source /opt/ros/jazzy/setup.bash; source /opt/snp/install/setup.bash; export ROS_DOMAIN_ID=42; timeout 300 python3 -u /workspace/snp_home/prechk.py $CFG $MESHC" 2>&1 | grep -vE "运行中|^  File|^    " | tail -14

echo "=== [4] 停剖析器 ==="
docker exec $C bash -lc 'kill "$(cat /workspace/snp_home/prof.pid)" 2>/dev/null; echo stopped'
sleep 1

echo "=== [5] 结果 ==="
if node_alive; then echo "  ✓ 节点存活"; else echo "  ✗ 节点死了"; fi
echo
python3 - "$LOG" <<'PY'
import sys
ls = [l.rstrip() for l in open(sys.argv[1]) if l.startswith('RSS=')]
print(f"  采样 {len(ls)} 行")
if not ls: sys.exit()
def rss(l): return float(l.split()[0].split('=')[1].rstrip('M'))
peak = max(ls, key=rss)
print(f"  峰值行: {peak}\n")
print("  爬升过程（每 3 行取 1 行）:")
start = next((i for i,l in enumerate(ls) if rss(l) > 200), 0)
for l in ls[start:][::3][:16]:
    print("   ", l[:170])
PY
