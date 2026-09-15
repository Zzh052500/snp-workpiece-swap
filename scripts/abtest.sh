#!/bin/bash
# A/B 对照：换 results_mesh.ply（碰撞环境用的网格）-> 重启 -> 测运动规划内存。
# 用法: abtest.sh <网格文件(主机路径)> <标签>
#
# runtime/snp_home 就是容器的 /workspace/snp_home（bind mount），所以换文件不用 docker cp。
set -u
C=snp_automate_2023_sim
H="${SNP_SIM_DIR:-$HOME/snp-automate-2023-polishing-simulation}/runtime/snp_home"
MESH=$H/snp/meshes/results_mesh.ply
MESHC=/workspace/snp_home/snp/meshes/results_mesh.ply
PJ=/opt/snp/install/snp_motion_planning/share/snp_motion_planning/config/task_composer_plugins.yaml
LAUNCH=/opt/snp/install/snp_motion_planning/share/snp_motion_planning/launch/planning_server.launch.xml
SRC="$1"
TAG="$2"
LOG="$H/ab_${TAG}.log"
CFG=/opt/snp_automate_2023/install/snp_automate_2023/share/snp_automate_2023/config/tpp.yaml

node_alive() {
  docker exec $C bash -lc 'for d in /proc/[0-9]*; do [ "$(cat $d/comm 2>/dev/null)" = "snp_motion_plan" ] && exit 0; done; exit 1'
}

echo "=== [0] 还原前几次实验留下的改动 ==="
[ -f "$H/task_composer_plugins.yaml.orig" ] && docker cp "$H/task_composer_plugins.yaml.orig" "$C:$PJ" && echo "  threads -> 8 (还原)"
[ -f "$H/planning_server.launch.xml.orig" ] && docker cp "$H/planning_server.launch.xml.orig" "$C:$LAUNCH" && echo "  lvs -> 0.05 (还原)"

echo "=== [1] 换网格: $SRC ==="
cp "$H/snp/meshes/results_mesh.ply.bak-stool" "$H/snp/meshes/_stool.ply"
cp "$H/seat_only.ply" "$H/snp/meshes/_seat.ply"
cp "$SRC" "$MESH"
md5sum "$MESH" | sed 's/^/  /'
python3 - "$MESH" <<'PY'
import sys
n=v=f=0
for i,ln in enumerate(open(sys.argv[1],errors='ignore')):
    if i==1: print(f"  头部: {ln.strip()[:70]}")
    if ln.startswith('element vertex'): v=int(ln.split()[-1])
    elif ln.startswith('element face'): f=int(ln.split()[-1])
    if ln.strip()=='end_header': n=i; break
print(f"  顶点={v} 面={f} header={n+1}行")
PY

echo "=== [2] 重启容器 ==="
docker restart $C >/dev/null
for i in $(seq 1 40); do
  if node_alive; then echo "  第 $((i*2))s：节点就绪"; break; fi
  sleep 2
done
if ! node_alive; then echo "  !! 节点起不来，放弃"; exit 1; fi
sleep 8

echo "=== [3] 监视器 ==="
docker exec $C bash -lc "rm -f /workspace/snp_home/mon.pid; echo '=== $TAG ===' > /workspace/snp_home/ab_${TAG}.log"
docker exec -d $C bash -lc "echo \$\$ > /workspace/snp_home/mon.pid; exec bash /workspace/snp_home/mon.sh /workspace/snp_home/ab_${TAG}.log"
sleep 2

echo "=== [4] 开火 ==="
docker exec $C bash -lc "source /opt/ros/jazzy/setup.bash; source /opt/snp/install/setup.bash; export ROS_DOMAIN_ID=42; timeout 300 python3 -u /workspace/snp_home/prechk.py $CFG $MESHC" 2>&1 | grep -vE "运行中|^  File|^    " | tail -22

echo "=== [5] 停监视器 ==="
docker exec $C bash -lc 'kill "$(cat /workspace/snp_home/mon.pid)" 2>/dev/null; echo stopped'

echo "=== [6] 结果 ==="
if node_alive; then echo "  ✓ 节点存活 —— 成功！"; else echo "  ✗ 节点死了"; fi
python3 - "$LOG" <<'PY'
import re,sys
rows=[]
for ln in open(sys.argv[1]):
    if '|' not in ln: continue
    cur,rest=ln.split('|',1)
    try: cur=int(cur)/1048576
    except: continue
    d={}
    for m in re.finditer(r'(\d+)\s+(\d+)\s+(\S+)',rest): d[m.group(3)]=int(m.group(1))/1024
    rows.append((cur,d))
if not rows: print("  (无数据)"); sys.exit()
print(f"  容器峰值 = {max(r[0] for r in rows):.0f} MiB  (上限 3072)")
n=[d['snp_motion_plan'] for _,d in rows if 'snp_motion_plan' in d]
if n:
    print(f"  节点 RSS 起始 {n[0]:.0f} -> 峰值 {max(n):.0f} MiB   增量 +{max(n)-n[0]:.0f} MiB")
    up=[]
    for v in n:
        if not up or v>max(up): up.append(v)
    print("  上升段: " + " ".join(f"{v:.0f}" for v in up[:22]))
PY
