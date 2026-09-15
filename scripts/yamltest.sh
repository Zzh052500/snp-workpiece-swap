#!/bin/bash
# 通用：改 task_composer_plugins.yaml 里的一个设置 -> docker cp 进容器 -> 重启 -> 测运动规划内存。
# 用法: yamltest.sh <sed表达式> <标签>
# 例:   yamltest.sh 's/threads: 8/threads: 1/' threads1
set -u
C=snp_automate_2023_sim
CFG=/opt/snp_automate_2023/install/snp_automate_2023/share/snp_automate_2023/config/tpp.yaml
MESH=/workspace/snp_home/snp/meshes/results_mesh.ply
HOSTDIR=/home/zzh/snp-automate-2023-polishing-simulation/runtime/snp_home
PJ=/opt/snp/install/snp_motion_planning/share/snp_motion_planning/config/task_composer_plugins.yaml
LAUNCH=/opt/snp/install/snp_motion_planning/share/snp_motion_planning/launch/planning_server.launch.xml
SEDEXPR="$1"
TAG="$2"
LOG="/workspace/snp_home/tc_${TAG}.log"

node_alive() {
  docker exec $C bash -lc 'for d in /proc/[0-9]*; do [ "$(cat $d/comm 2>/dev/null)" = "snp_motion_plan" ] && exit 0; done; exit 1'
}

echo "=== [0] 还原 launch 文件的默认值（清掉上次测试的改动）==="
if [ -f "$HOSTDIR/planning_server.launch.xml.orig" ]; then
  docker cp "$HOSTDIR/planning_server.launch.xml.orig" "$C:$LAUNCH" && echo "  已还原 contact_check_lvs_distance"
fi

echo "=== [1] 取出 task composer 配置（原始备份只做一次）==="
[ -f "$HOSTDIR/task_composer_plugins.yaml.orig" ] || docker cp "$C:$PJ" "$HOSTDIR/task_composer_plugins.yaml.orig"
cp "$HOSTDIR/task_composer_plugins.yaml.orig" "$HOSTDIR/task_composer_plugins.yaml.new"
echo "  相关原始设置:"
grep -nE "threads:|max_convex_hulls|conditional:" "$HOSTDIR/task_composer_plugins.yaml.new" | head -5 | sed 's/^/    /'

echo "=== [2] 应用: $SEDEXPR ==="
sed -i "$SEDEXPR" "$HOSTDIR/task_composer_plugins.yaml.new"
if diff -q "$HOSTDIR/task_composer_plugins.yaml.orig" "$HOSTDIR/task_composer_plugins.yaml.new" >/dev/null; then
  echo "  !! 文件没有变化，sed 表达式没匹配上 —— 中止"
  exit 1
fi
diff "$HOSTDIR/task_composer_plugins.yaml.orig" "$HOSTDIR/task_composer_plugins.yaml.new" | sed 's/^/    /'

echo "=== [3] 放进容器 ==="
docker cp "$HOSTDIR/task_composer_plugins.yaml.new" "$C:$PJ"

echo "=== [4] 重启 ==="
docker restart $C >/dev/null
for i in $(seq 1 30); do
  if node_alive; then echo "  第 $((i*2))s：节点就绪"; break; fi
  sleep 2
done
sleep 8

echo "=== [5] 监视器 ==="
docker exec $C bash -lc "rm -f /workspace/snp_home/mon.pid; echo '=== $TAG ===' > $LOG"
docker exec -d $C bash -lc "echo \$\$ > /workspace/snp_home/mon.pid; exec bash /workspace/snp_home/mon.sh $LOG"
sleep 2

echo "=== [6] 开火 ==="
docker exec $C bash -lc "source /opt/ros/jazzy/setup.bash; source /opt/snp/install/setup.bash; export ROS_DOMAIN_ID=42; timeout 240 python3 -u /workspace/snp_home/prechk.py $CFG $MESH" 2>&1 | grep -vE "运行中|^  File|^    " | tail -20

echo "=== [7] 停监视器 ==="
docker exec $C bash -lc 'kill "$(cat /workspace/snp_home/mon.pid)" 2>/dev/null; echo stopped'

echo "=== [8] 结果 ==="
if node_alive; then echo "  ✓ 节点存活 —— 有戏！"; else echo "  ✗ 节点还是死了"; fi
python3 - "$HOSTDIR/tc_${TAG}.log" <<'PY'
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
if rows:
    print(f"  容器峰值 = {max(r[0] for r in rows):.0f} MiB")
    n=[d['snp_motion_plan'] for _,d in rows if 'snp_motion_plan' in d]
    if n:
        print(f"  节点 RSS 起始 {n[0]:.0f} -> 峰值 {max(n):.0f} MiB  (共 {len(n)} 个采样点)")
        up=[]
        for v in n:
            if not up or v>max(up): up.append(v)
        print("  上升段: " + " ".join(f"{v:.0f}" for v in up[-14:]))
PY
