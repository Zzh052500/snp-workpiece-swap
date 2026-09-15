#!/bin/bash
# 改 planning_server.launch.xml 里的一个 arg 默认值 -> docker cp -> 重启 -> 测内存。
# 用法: cfgtest.sh '<sed表达式>' <标签>
# 例:   cfgtest.sh 's|\(name="collision_object_type" default="\)[^"]*|\1mesh|' mesh
set -u
C=snp_automate_2023_sim
H="${SNP_SIM_DIR:-$HOME/snp-automate-2023-polishing-simulation}/runtime/snp_home"
PJ=/opt/snp/install/snp_motion_planning/share/snp_motion_planning/config/task_composer_plugins.yaml
LAUNCH=/opt/snp/install/snp_motion_planning/share/snp_motion_planning/launch/planning_server.launch.xml
CFG=/opt/snp_automate_2023/install/snp_automate_2023/share/snp_automate_2023/config/tpp.yaml
MESHC=/workspace/snp_home/snp/meshes/results_mesh.ply
SEDEXPR="$1"
TAG="$2"
LOG="$H/cfg_${TAG}.log"

node_alive() {
  docker exec $C bash -lc 'for d in /proc/[0-9]*; do [ "$(cat $d/comm 2>/dev/null)" = "snp_motion_plan" ] && exit 0; done; exit 1'
}

echo "=== [0] 还原 task composer 配置 ==="
[ -f "$H/task_composer_plugins.yaml.orig" ] && docker cp "$H/task_composer_plugins.yaml.orig" "$C:$PJ" >/dev/null && echo "  threads -> 8"

echo "=== [1] 取原始 launch（已备份则直接用）==="
[ -f "$H/planning_server.launch.xml.orig" ] || docker cp "$C:$LAUNCH" "$H/planning_server.launch.xml.orig"
cp "$H/planning_server.launch.xml.orig" "$H/planning_server.launch.xml.new"

echo "=== [2] 应用: $SEDEXPR ==="
sed -i "$SEDEXPR" "$H/planning_server.launch.xml.new"
if diff -q "$H/planning_server.launch.xml.orig" "$H/planning_server.launch.xml.new" >/dev/null; then
  echo "  !! sed 没匹配上，中止"; exit 1
fi
diff "$H/planning_server.launch.xml.orig" "$H/planning_server.launch.xml.new" | sed 's/^/    /'

echo "=== [3] 放进容器 ==="
docker cp "$H/planning_server.launch.xml.new" "$C:$LAUNCH"

echo "=== [4] 重启 ==="
docker restart $C >/dev/null
for i in $(seq 1 40); do
  if node_alive; then echo "  第 $((i*2))s 就绪"; break; fi
  sleep 2
done
if ! node_alive; then echo "  !! 节点起不来，中止"; exit 1; fi
sleep 8

echo "=== [5] 确认参数生效 ==="
docker exec $C bash -lc "source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=42; for p in collision_object_type max_convex_hulls octree_resolution; do printf '  %-24s = ' \$p; timeout 12 ros2 param get /snp_planning_server \$p 2>/dev/null | tail -1; done"

echo "=== [6] 监视器 ==="
docker exec $C bash -lc "rm -f /workspace/snp_home/mon.pid; echo '=== $TAG ===' > /workspace/snp_home/cfg_${TAG}.log"
docker exec -d $C bash -lc "echo \$\$ > /workspace/snp_home/mon.pid; exec bash /workspace/snp_home/mon.sh /workspace/snp_home/cfg_${TAG}.log"
sleep 2

echo "=== [7] 开火 ==="
docker exec $C bash -lc "source /opt/ros/jazzy/setup.bash; source /opt/snp/install/setup.bash; export ROS_DOMAIN_ID=42; timeout 300 python3 -u /workspace/snp_home/prechk.py $CFG $MESHC" 2>&1 | grep -vE "运行中|^  File|^    " | tail -16

echo "=== [8] 停监视器 ==="
docker exec $C bash -lc 'kill "$(cat /workspace/snp_home/mon.pid)" 2>/dev/null; echo stopped'

echo "=== [9] 结果 ==="
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
