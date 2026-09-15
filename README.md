# SNP 打磨仿真：替换工件为「坐面」

在 SNP Automate 2023 打磨仿真（ROS 2 Jazzy + Tesseract）里，把默认工件换成一张**凳子坐面板**，
并让刀路生成变得**干净、自动、可复现**；同时记录一个**至今未解的运动规划 OOM 故障**。

> **一句话结论**：刀路那半边已经做到了「7 条纯坐面光栅线 / 294 点 / 零跳变 / 逐点可复现」；
> 但运动规划（`/generate_motion_plan`）目前是**无条件硬故障**——任何请求都会在 6 秒内把
> `snp_motion_planning_node` 撑到 ~2.3 GB 撞上容器内存上限被 SIGKILL，**与工件和参数都无关**。

---

## 目录

- [1. 背景](#1-背景)
- [2. 工件替换](#2-工件替换)
- [3. tpp.yaml 的两处修改](#3-tppyaml-的两处修改)
- [4. 未解故障：运动规划必然 OOM](#4-未解故障运动规划必然-oom)
- [5. 容器加固](#5-容器加固)
- [6. 复现步骤](#6-复现步骤)
- [7. 文件清单](#7-文件清单)
- [8. 环境](#8-环境)

---

## 1. 背景

仿真跑在一个 Docker 容器里（`snp_automate_2023_sim`，`ROS_DOMAIN_ID=42`）。
整条链路是：

```
点云扫描 → industrial_reconstruction 重建网格 → /plan_tool_path 出刀路 → /generate_motion_plan 做运动规划
   ↑ 仿真模式（SNP_SIM_VISION=true）直接给网格，不需要真相机
```

目标（用户原话）：**「先把座面能够稳定打磨并且可以复现」**。
「可复现」是硬要求，这直接决定了下面两处配置修改的取舍。

---

## 2. 工件替换

### 2.1 为什么原来的工件不行

原工件是一个架在桌面上的**球拱面**。换成凳子后，橙色的 `ROISelection` 圈选失效了：

`ROISelection`（`snp_tpp/roi_selection_mesh_modifier.h`）读 RViz 多边形选择工具的圈选结果，
交给 noether 的 `ExtrudedPolygonSubsetExtractor`。后者的算法是
**拟合平面 → 投影边界 → 沿拟合平面法向把多边形拉成棱柱 → 取棱柱内顶点**。

也就是说它取的是**柱体**，不是你在屏幕上圈出来的那块曲面。
对球拱面没问题（竖直柱体切出来的正好是一块干净曲面），但对凳子：

> 坐面下方的竖直柱体**必然把四条腿一起包进来**。
> 实测圈选坐面时柱内顶点 z 从 0.087 贯穿到 0.317，坐面 + 四条腿全在里面。

栅格规划器在「带腿柱体」上生成不出刀路 → 返回空 `tool_paths` → 行为树下游对空 vector 取 `at(0)`
→ `std::vector::_M_range_check` → `Behavior tree did not complete successfully`。

只要腿在坐面下方，**这是几何上的死结**，调 `min_cluster_size` / `plane_distance_threshold` 都无解。

### 2.2 切出坐面

用 z 阈值把腿切掉（`scripts/seatcut.py`）：

```python
SRC = '/workspace/snp_home/snp/meshes/results_mesh.ply'
DST = '/workspace/snp_home/seat_only.ply'
THR = 0.27
keep = (V[F][:, :, 2] >= THR).all(axis=1)   # 三个顶点都在阈值之上的面才保留
```

阈值是看 z 分布定的（`scripts/meshz.py`）：

| z 区间 | 内容 |
|---|---|
| 0.087 | 桌面接触点（360 个点） |
| 0.087 – 0.269 | 四条腿 |
| 0.279 – 0.317 | 坐面（0.298 处峰值，945 个点） |

不同阈值下的面数：

| 阈值 | 0.20 | 0.25 | **0.27** | 0.28 | 0.29 |
|---|---|---|---|---|---|
| 面数 | 7262 | 4724 | **4082** | 3242 | 2702 |

**切割结果（实测）**：

```
原网格  6463 顶点 / 12032 面
坐面板  2256 顶点 /  4082 面     文件 125494 字节  md5 9af94bbe191e3278890a6422b573e047
bbox    x[0.658, 0.938]  y[-0.095, 0.115]  z[0.270, 0.317]
尺寸    0.280 × 0.210 × 0.047 m
z < 0.26 的顶点数：0        ← 确认没有腿
```

### 2.3 装进仿真

坐面网格要同时放到**两个**位置，并各自备份原工件：

```bash
meshes/part_scan.ply                                  # 原始工件（刀路生成的输入）
runtime/snp_home/snp/meshes/results_mesh.ply          # 重建输出的落点
```

```bash
cp meshes/part_scan.ply                       meshes/part_scan.ply.bak-stool            # 376780 字节
cp runtime/snp_home/snp/meshes/results_mesh.ply runtime/.../results_mesh.ply.bak-stool
```

改完**必须重启容器**（`docker restart snp_automate_2023_sim`）。

### 2.4 验证 RViz 里推的确实是坐面

调用 `/stop_reconstruction` 把网格推到 RViz，然后用 `scripts/meshcheck.py` 订阅
`/industrial_reconstruction_mesh` 检查。

> **踩坑**：这个话题的类型是 `visualization_msgs/msg/Marker`（`TRIANGLE_LIST`），
> **不是** `shape_msgs/Mesh`。这是 `industrial_reconstruction` 的约定——RViz 用 Marker 显示网格。
> 用 `shape_msgs/Mesh` 订阅会静默收不到任何消息。

实测输出：

```
收到 Marker: type=TRIANGLE_LIST  ns=''  id=1
  frame_id = 'base_link'
  点数 = 12246  ->  三角面 4082
  bbox x[0.658,0.938] y[-0.095,0.115] z[0.270,0.317]
  >>> 是坐面（z 全部 > 0.26，没有腿）
```

12246 / 3 = 4082，与 `seat_only.ply` 的面数、bbox 完全一致。

---

## 3. tpp.yaml 的两处修改

`config/tpp.yaml` 是刀路生成的配置。改了**两处**，**两个修改是独立的**，各自解决一个不同的问题：

### 3.1 停用 `ROISelection`

去掉「只打磨圈选区域」的柱体提取（理由见 2.1）。因为腿已经切掉了，
「竖直柱体必然包住腿」的死结不存在了。

保留的 modifier：`NormalsFromMeshFaces`。规划器参数未动：
`line_spacing` / `point_spacing` = 0.03，`min_hole_size` / `min_segment_size` = 0.1，
`bidirectional: true`。

### 3.2 `LocatedVectorDirection` → `FixedDirection`

**这是「可复现」的关键。** 原配置用 `LocatedVectorDirection`，它的坐标来自 RViz 的
`LocatedVector` 工具，而 `config/app.rviz` 里只保存了线颜色和服务名，**没有保存起点终点**：

```yaml
- Class: noether_ros/LocatedVector
  Line color: 128; 128; 0
  Render as overlay: true
  Service name: get_located_vector
```

后果：**容器/RViz 每次重启，located vector 就丢了**，必须手工再拖一次；没拖之前
`/plan_tool_path` 一律报：

```
Error invoking tool path planner ... Start point has not been set
```

这是整条链路里**唯一的纯手工、不可复现环节**，所以换成固定方向：

```yaml
direction_generator:
  name: FixedDirection
  direction:
    x: 1.0
    y: 0.0
    z: 0.0
```

> **YAML 形状陷阱**：`direction` 必须是 `{x, y, z}` **映射**。
> 写成 `[1.0, 0.0, 0.0]` 会报 `Required member 'x' not found in YAML node`。

`x: 1.0` 与原来手拖出来的方向一致（手拖时 source→target 实测约 `(+0.126, -0.013, -0.001)`）。

### 3.3 验证结果

`config/` 是**只读挂载**到容器里的，noether 服务器每次 `plan_tool_path` 都会重读配置，
所以**改完不需要重启**。

```
config = .../snp_automate_2023/config/tpp.yaml
mesh   = /workspace/snp_home/snp/meshes/results_mesh.ply
success = True  message = ''

路径0:  39点  x[0.659,0.936] y[0.099,0.099] z[0.274,0.319]  跳变0
路径1:  43点  x[0.658,0.934] y[0.069,0.069] z[0.282,0.319]  跳变0
路径2:  44点  x[0.658,0.933] y[0.039,0.039] z[0.281,0.319]  跳变0
路径3:  43点  x[0.658,0.934] y[0.009,0.009] z[0.282,0.319]  跳变0
路径4:  43点  x[0.658,0.934] y[-0.021,-0.021] z[0.282,0.319]  跳变0
路径5:  43点  x[0.658,0.934] y[-0.051,-0.051] z[0.281,0.319]  跳变0
路径6:  39点  x[0.659,0.937] y[-0.081,-0.081] z[0.275,0.318]  跳变0

合计 7 条路径 / 294 点
```

特征：沿 **+X 平行**、每条路径 **y 恒定**、z 只在 **0.274–0.319**（纯坐面高度）、
**零跳变**。跑两遍逐点相同 ⟹ **可复现**。

---

## 4. 未解故障：运动规划必然 OOM

> **状态：已定位到范围，根因未确认，未修复。** 下面全部是实测数据。

### 4.1 现象

任何 `/generate_motion_plan` 请求都会让 `snp_motion_planning_node` 在约 6 秒内从 ~130 MiB
涨到 **2.3 GB**，撞上容器 3 GiB 上限，被 OOM kill（`exit code -9`）。

**截至记录时：9 次请求、9 次死亡，无一幸免。**

```
Received motion planning request   9 次
Motion Planner process succeeded   9 次
process has died (exit -9)        10 次   ← 多出的一次是启动早期的另一次崩溃
```

即每一次规划请求都**确实规划成功了**，然后在返回结果的阶段把内存吃爆被杀掉——
不是随机故障，是确定性故障。

容器本身活着，但**节点死了 ⟹ 它提供的所有服务一起消失**，RViz 侧表现为：

```
[ Remove Scan Link ]        ->  FAILED  Service 'remove_scan_link' is unreachable
[ GenerateMotionPlanService ]-> FAILED  Service 'generate_motion_plan' is unreachable
```

### 4.2 关键证据一：与刀路点数**完全无关**

这是最能说明问题的一条。用 `scripts/prechk.py` 的缩放对照（第 3 个参数 = 只取前 N 条路径）：

| 输入 | 节点 RSS 起始 → 峰值 | 最陡上升 | 容器峰值 |
|---|---|---|---|
| **7 条路径 / 294 点** | 132 → **2358 MiB** | +2230 MiB / **5.8 s** | 3072（撞顶） |
| **1 条路径 / 39 点** | 130 → **2304 MiB** | +2174 MiB / **6.0 s** | 3072（撞顶） |

输入点数差了 **7.5 倍**，内存曲线几乎一模一样。

**结论：调 `point_spacing`、砍路径数、切网格都救不了。** 这条思路到此为止。

原始曲线在 `evidence/memwatch-7paths-294pts.log` 和 `evidence/memwatch-1path-39pts.log`
（0.2 秒粒度，逐进程 RSS，容器内每 0.2 s 采样一次 `memory.current` + `ps` 前 4 名）。

容器内存撞顶时的进程构成（`evidence/memwatch-7paths-294pts.log` 峰值那一行）：

```
容器 memory.current = 3072 MiB
   snp_motion_plan    pid=76     2331 MiB   ← 元凶
   python3            pid=74      278 MiB   ← 测试客户端，清白
   rviz2              pid=73      131 MiB
   joint_state_pub    pid=67       96 MiB
```

### 4.3 关键证据二：爆炸发生在**规划成功之后**

日志顺序是确定的（原始摘录见 `evidence/event-order.txt`）：

```
Received motion planning request
  → Failed to find or load library ... (×9)        ← 见 4.4，无害
  → KDL LMA Failed to calculate IK ... (×30)
  → Motion Planner process succeeded               ← 规划已经成功了
  → cache hit! ×664                                ← 内存在这里失控
  → process has died [pid 102, exit code -9]
```

字符串定位：

| 消息 | 出处 |
|---|---|
| `Environment, getKinematicGroup(manipulator, ) cache hit!` | `libtesseract_environment.so` |
| `Motion Planner process succeeded` | `tesseract_task_composer/planning/nodes/motion_planner_task.hpp` |
| `KDL LMA Failed to calculate IK, increment joints are tool small` | Tesseract KDL 求解器 |

### 4.4 已排除的干扰项

- ❌ **插件加载失败不是根因。**
  ```
  Failed to find or load library:
    /opt/tesseract_planning/install/tesseract_task_composer/lib/libsnp_motion_planning_tasks.so
    with error: Bad file descriptor
  ```
  库实际装在 `/opt/snp/install/snp_motion_planning/lib/`。这是 `boost_plugin_loader`
  **探测备用路径失败后继续**，插件最终正常加载，规划确实 `succeeded`。**红鲱鱼。**
- ❌ **不是测试客户端**：python3 全程 278–445 MiB。
- ❌ **不是坐面网格 / `FixedDirection`**：它们解决的是「刀路干净可复现」，与 OOM 无关。
  （这一点我一开始判断错了——曾以为换网格能顺带解决 OOM，实测证明不能。）

### 4.5 嫌疑范围

流水线定义在
`/opt/snp/install/snp_motion_planning/share/snp_motion_planning/config/task_composer_plugins.yaml`
（**在镜像里，不在我们挂载的 `config/` 下**）：

```
SNPPipeline:
  FormatInputTask → SimpleMotionPlannerTask → DescartesMotionPlannerTask
                  → RasterMotionTask → TCPSpeedLimiterTask → Done
```

`RasterMotionTask` 内部对每段再跑 `SNPCartesianPipeline` / `SNPFreespacePipeline`，
而这两个子流水线里**每个 planner 任务后面都紧跟一个 `DiscreteContactCheckTask`**：

```
TrajOptMotionPlannerTask → [ErrorTask, DiscreteContactCheckTask]
DiscreteContactCheckTask → [ErrorTask, ConstantTCPSpeedTimeParameterizationTask]
```

planner 一报成功、紧接着就狂调 `getKinematicGroup` 并失控涨内存 ⟹
**首要嫌疑是 `DiscreteContactCheckTask`**（轨迹逐点接触检查）在对轨迹做离散碰撞检查时，
反复取运动学组且不释放。

### 4.6 建议的下一步（未执行）

1. **验证猜想**：把 `DiscreteContactCheckTask` 从流水线里摘掉，看内存是否保持平稳。
   因为该 yaml 在镜像里，需要加一个挂载覆盖它，或找到能指到别处的 launch 参数。
2. 若确认，退一步的做法是把 `RasterMotionTask` 的 `transition` / `freespace` 子流水线
   换成不含接触检查的版本。

### 4.7 诊断手法的坑（供参考）

容器里**没有** `gdb` / `eu-stack` / `pstack` / `perf`，抓不了栈。可用的手段只有：
cgroup 内存采样 + `ps` 逐进程 RSS + 日志字符串定位。

另外有一个**我犯过的错**值得记下来：用 `docker exec ... | tail -30` 看输出时，
前面的心跳行会被截掉，导致我一度把「节点已经被杀之后的回落期」当成「内存平稳」，
**误报了一次「OOM 没复现」**。看长输出不要用 `tail` 截断。

---

## 5. 容器加固

`docker/compose.sim.yml` 的改动（加固前备份为 `docker/compose.sim.yml.bak-before-hardening`）：

```yaml
shm_size: 512m            # 默认 64MB，RViz/DDS/Qt 都走 shm，加载网格和轨迹时会顶到上限
mem_limit: 3g             # 容器内存上限
memswap_limit: 3g         # == mem_limit 即为禁用容器 swap
logging:
  driver: json-file
  options: { max-size: "10m", max-file: "3" }   # 节点会狂刷 cache hit，默认无上限会写满磁盘
```

**主机只有 5.8 GB 内存。** 容器默认 `mem_limit=0`（无上限）：规划一旦爆内存，内核会拿
**整个主机**去 swap，桌面连同所有窗口一起僵死——这就是「电脑直接卡死」的机制
（当时容器 `OOMKilled=false`、`RestartCount=0`，说明容器自己没被杀，是**主机被拖死了**）。

设上限之后，越界的是**容器内某个进程被 kill**（`docker start` 几秒就回来），
而不是整台机器失去响应。

> **这个加固是对的**，但它不解决 OOM——它只是把「整机冻死」变成了「节点被杀」。
> 而节点被杀正是 `remove_scan_link is unreachable` 的直接原因。

已验证生效：

```
ShmSize=536870912 (512M)   Memory=3221225472 (3g)   MemSwap=3221225472
LogOpts={max-file:3 max-size:10m}   容器内 /dev/shm = 512M
memory.max = 3221225472   memory.swap.max = 0
```

---

## 6. 复现步骤

```bash
C=snp_automate_2023_sim
CFG=/opt/snp_automate_2023/install/snp_automate_2023/share/snp_automate_2023/config/tpp.yaml
MESH=/workspace/snp_home/snp/meshes/results_mesh.ply
```

### 6.1 看刀路（安全，0.1 秒，不会 OOM）

```bash
docker exec $C bash -lc "source /opt/ros/jazzy/setup.bash; source /opt/snp/install/setup.bash; \
  export ROS_DOMAIN_ID=42; timeout 120 python3 -u /workspace/snp_home/tpptest.py $CFG $MESH"
```

### 6.2 复现 OOM（**会让节点死掉，之后需要 `docker restart`**）

```bash
docker exec $C bash -lc "source /opt/ros/jazzy/setup.bash; source /opt/snp/install/setup.bash; \
  export ROS_DOMAIN_ID=42; timeout 150 python3 -u /workspace/snp_home/prechk.py $CFG $MESH"
```

缩放对照（只取前 1 条路径）：

```bash
docker exec $C bash -lc "... python3 -u /workspace/snp_home/prechk.py $CFG $MESH 1"
```

一键复现 + 录内存曲线：`scripts/oomrepro.sh`（会先 `docker restart`，
**轮询**等节点就绪而不是硬等）。

### 6.3 恢复仿真

```bash
docker restart snp_automate_2023_sim
# 轮询等节点（每 2s 一次）
docker exec $C bash -lc 'for d in /proc/[0-9]*; do [ "$(cat $d/comm 2>/dev/null)" = "snp_motion_plan" ] && exit 0; done; exit 1'
```

> **判断节点死活有两个陷阱，方向正好相反：**
>
> 1. **假阳性**：`pgrep -f snp_motion_planning_node` 会匹配到自己的 bash 包装脚本。
>    必须用 `/proc/*/comm` 精确比对（`comm` 被截断到 15 字符，正好是 `snp_motion_plan`）。
> 2. **假阴性以外的另一种假象**：**`ros2 service list` 在节点死后会继续列出它的服务**——
>    那是 ROS 2 daemon 的发现缓存，不是真相。实测节点根本没起来，
>    `/generate_motion_plan` 却还在列表里，害得人以为服务正常。
>    **别用服务列表判断存活**，只看 `/proc`。

---

## 7. 文件清单

| 路径 | 说明 |
|---|---|
| `scripts/seatcut.py` | 按 z 阈值切出坐面 |
| `scripts/meshz.py` | z 分布分析，用来定阈值 |
| `scripts/meshcheck.py` | 验证 RViz 里推的是坐面（订阅 `/industrial_reconstruction_mesh`） |
| `scripts/tpptest.py` | 通用刀路测试：`tpptest.py <config.yaml> <mesh.ply>`，按路径打印点数/bbox/跳变 |
| `scripts/pathdump.py` | 把刀路导出成 JSON |
| `scripts/prechk.py` | 运动规划预检，带内存心跳；第 3 个参数可只取前 N 条路径做缩放对照 |
| `scripts/oomrepro.sh` | OOM 复现 + 0.2s 粒度内存记录 |
| `config/tpp.yaml` | 改好的配置（`FixedDirection`，ROISelection 已停用，含详细中文注释） |
| `config/tpp.yaml.bak-before-roiselection-removal` | 停用 ROISelection 之前的备份 |
| `config/tpp.yaml.bak-before-fixed-direction` | 换 FixedDirection 之前的备份 |
| `artifacts/seat_only.ply` | 坐面板网格（2256 顶点 / 4082 面，ASCII PLY） |
| `evidence/memwatch-7paths-294pts.log` | 7 条路径的逐进程内存曲线（0.2s 粒度） |
| `evidence/memwatch-1path-39pts.log` | 1 条路径的同上（缩放对照） |
| `evidence/event-order.txt` | 崩溃前的事件顺序原始摘录 |

---

## 8. 环境

| 项 | 值 |
|---|---|
| 容器 | `snp_automate_2023_sim` |
| ROS | ROS 2 Jazzy |
| `ROS_DOMAIN_ID` | 42 |
| 网络 | `network_mode: host` |
| 机器人 | Motoman HC10 |
| 仿真开关 | `SNP_SIM_ROBOT=true` `SNP_SIM_VISION=true` `SNP_BYPASS_EXECUTION=true` |
| 运动组 / TCP | `manipulator` / `sand_tcp` |
| mesh frame | `base_link` |
| 主机 | HP Zhan 66 Pro A G1 R MT，5.8 GB 内存 |
| 容器内调试器 | **无** `gdb` / `eu-stack` / `pstack` / `perf` |
