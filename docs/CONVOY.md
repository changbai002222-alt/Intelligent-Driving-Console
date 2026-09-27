# 两车协同（跟驰）避碰

> 目的：**两台车在同一个闭环路径上跑的时候，后车不会追尾前车。**
>
> 一句话原理：车端把「我在哪、多快、这是哪张地图/哪条路径」推到本机管理端；
> 管理端把两台车都投影到同一条路径上，算出「谁在前谁在后、间距多少、是否在接近」，
> 给出跟驰建议；**打开 arm 之后**，建议「停车」时会对后车下发急停。

⚠️ **它只做协同避碰，不做别的事**：不接管循迹、不换路径、不改地图。

---

## 一、数据怎么流

```
 车端（每台车）
 ┌──────────────────────────────────────────────┐
 │ convoy_observer.py  （Python 2.7 + ROS）      │
 │  · 只订阅 /current_pose 和 /ctrl_fb          │
 │  · 没有任何 publisher —— 不发任何指令          │
 │  · 10Hz POST 位姿/速度/年龄/地图与路径哈希     │
 └───────────────────┬──────────────────────────┘
                     │ POST /api/telemetry
                     │ 头：X-Observer-Token: <令牌>
                     ▼
 本机管理端（你的 PC）
 ┌──────────────────────────────────────────────┐
 │ fleet.py  ──►  convoy.py（协同调度器）        │
 │  · 校验令牌（不走同源限制，车端不是本机）      │
 │  · 从车端拉一次路径 -> convoy_policy 投影      │
 │  · 算每车 s（沿路径里程）/ lateral（横向偏差） │
 │  · 两两成对算间距、接近速度、跟驰建议          │
 └───────────────────┬──────────────────────────┘
                     │ GET /api/convoy   （只读，前端/调试可查）
                     │
                     └─ arm=true 时：建议「停车」-> 后车 emergency_stop
```

**注意**：位姿是**推**上来的（车端 → PC），路径是**拉**回来的（PC → 车端 `/api/route`）。

---

## 二、本机管理端配置：`runtime/convoy.json`

`runtime/convoy.json` **已被 `.gitignore` 忽略**（里面有令牌），所以每份发布包都现场生成一份、令牌各不相同。模板见 `runtime/convoy.example.json`。

| 键 | 默认 | 说明 |
|---|---|---|
| `observer_token` | 随机 | 车端 `convoy-observer.env` 的 `CONVOY_TOKEN` **必须和它一致** |
| `fresh_seconds` | 1.5 | 遥测超过这个秒数算过期，该车退出判断 |
| `hold_seconds` | 0 | 建议停车后至少保持几秒（0 = 靠自带迟滞） |
| **`arm`** | **false** | ⚠️ **false = 只算建议、绝不下发**；true = 真的会对车下发急停 |
| `stop_base_m` | 5.0 | 停车基础间距（车身长 + 静止时想留的距离） |
| `estop_lag_s` | 7.0 | 实测急停耗时（秒），用来预算制动距离 |
| `stop_margin_m` | 1.0 | 额外安全余量 |

### 两个接口

| 接口 | 谁调 | 认证 |
|---|---|---|
| `POST /api/telemetry` | **车端 observer** | `X-Observer-Token`（**独立令牌**，不走同源限制 —— 因为车端的 Host 不可能是本机） |
| `GET /api/convoy` | 前端 / 你自己 | 本机同源（`127.0.0.1` / `localhost`） |

`/api/convoy` 返回：

```json
{
  "timestamp": 1790519902.47,
  "route": {"name": "421.csv", "length": 57.14, "error": ""},
  "vehicles": [
    {"vehicle": "实训车01", "s": 11.68, "lateral": 0.03, "speed": 0.22,
     "age": 0.09, "pose_age": 0.11, "healthy": true, "reason": ""}
  ],
  "advice": [
    {"a": "实训车01", "b": "实训车03", "state": "clear",
     "ahead": "实训车03", "behind": "实训车01",
     "gap": 18.4, "closing_speed": 0.01, "stop_gap": 5.31, "factor": 2.0, "reason": "…"}
  ],
  "armed": false
}
```

---

## 三、车端部署

### 前置条件（⚠️ 少一个协同就是空的）

1. 车端**已选好地图和路径** —— observer 每 5 秒从车端 `runtime/config.json` 读 `selected_map` / `selected_route`；**没选就会报 `metadata_error`，位姿推上来但后端拿不到路径名**，无法投影。
2. 车端**和 PC 在同一个局域网**。
3. 车端 `convoy-observer.env` 的令牌与 PC 的 `runtime/convoy.json` **一致**。

### 一条命令部署（推荐）

```bash
./deploy-to-car.sh 192.168.31.134 192.168.31.232
```

它会自动：拷 `convoy_observer.py` → 拷启动脚本与 systemd 单元 → **按 PC 上的令牌和 `fleet.json` 里的车名生成车端配置** → 安装并重启 `bigcar-convoy` 服务。

如果你的 PC 有多张网卡、自动探测的 IP 不对，指定一下：

```bash
FLEET_HOST=192.168.31.74 ./deploy-to-car.sh 192.168.31.134
```

### 手动部署

```bash
# 1) 拷文件
scp backend/convoy_observer.py nvidia@<车IP>:/home/nvidia/Desktop/bigcar-console/backend/
scp deploy/convoy-run.sh deploy/bigcar-convoy.service nvidia@<车IP>:/home/nvidia/Desktop/bigcar-console/deploy/

# 2) 在车上写配置
#    /home/nvidia/Desktop/bigcar-console/runtime/convoy-observer.env
CONVOY_URL=http://192.168.31.74:8870     # 你的 PC，别写 127.0.0.1
CONVOY_TOKEN=<与 PC 的 observer_token 一致>
CONVOY_VEHICLE=实训车01                   # 与平台上登记的车辆名一致

# 3) 安装并启动（自带 Restart=always，掉了会自动拉起）
chmod 755 /home/nvidia/Desktop/bigcar-console/deploy/convoy-run.sh
sudo cp /home/nvidia/Desktop/bigcar-console/deploy/bigcar-convoy.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now bigcar-convoy
systemctl is-active bigcar-convoy          # 应输出 active
```

启动脚本 `deploy/convoy-run.sh` 会**用 env 的值现场生成** `runtime/convoy-observer.json`（observer 自己读的是这个固定路径），所以 **不需要改 `convoy_observer.py`**。observer 与 `person_camera_observer` 一样跑在容器里（`docker exec autoware_ai_orin`，`python2` + ROS）。

---

## 四、怎么判断它工作正常

看 `GET /api/convoy`：

| 现象 | 含义 | 怎么办 |
|---|---|---|
| `route.length` 有数 | 路径已加载 ✅ | —— |
| `route.error` 非空 | 路径不是闭环 / 点太少 | 检查车端选的路径 |
| `vehicles` 为空 | 一条遥测都没收到 | ①令牌对不对 ②`CONVOY_URL` 是不是 PC 的 IP ③车端服务 `systemctl is-active bigcar-convoy` |
| `vehicles[].healthy=false` | 这车位姿不可信，**不参与判断** | 看 `reason` |
| `vehicles[].reason="后端还没有可用路径（尚未加载）"` | 车端没选地图/路径 | 在车端控制台选好地图与路径 |
| `reason="off_route_or_heading"` | 车偏离路径超 1.5m，或朝向与路径差太多 | 正常现象（车不在路径上） |
| `advice[].state="suspended"` | **数据不可信，明确挂起** | ⚠️ **不是「没危险」**，是「看不清」 |
| `advice[].state="stop"` | 后车该停 | gap ≤ stop_gap |
| `advice[].state="clear"` | 安全 | —— |

**`advice` 里没有某一对车，等于「只有一台车或低于两台」** —— 只要有两台车，任何一台不可信都会以 `suspended` 明确列出来，**不会静默消失**。这是刻意设计的：**不能把「看不清」误读成「没危险」**。

> 另一条同源的设计：`main_sync_loop.py` 的位姿闸门（横向偏差超限就拒绝参与）也是这个原则。

---

## 五、参数与已知的坑

### 停车阈值怎么来的

```
stop_gap = stop_base_m + estop_lag_s × closing_speed + stop_margin_m
```

代入实测值（基础 5、急停 7 秒、余量 1）：

| 场景 | closing | 阈值 |
|---|---|---|
| 两车同速（都 0.2 m/s） | ~0 | **5.2 m** |
| 速度差 0.2 | 0.2 | 6.4 m |
| 速度差 0.4 | 0.4 | 7.8 m |

### ⚠️ 已知问题 1：同速跟驰可能被误判「停车」

`stop_gap` 里那 **5 米基础值**（车身 2m + 静止余量 3m）对 57.14 米的环来说偏奢侈。**实测两车正常间距约 5.1~5.5 m，正好压在 5.2 m 这个临界点上** —— 开了 `arm` 之后后车可能会停一次。

**缓解**：`FollowingAdvice` 自带迟滞 —— 一旦触发停车，要等间距拉回约 **8.2 m** 才恢复，所以不会来回抖。如果你觉得太敏感，把 `stop_base_m` 从 `5.0` 调到 `3.5` 即可（一个数的事，改完重启后端生效）。

### ⚠️ 已知问题 2：高速追及余量不足

原来的判据用的是**后车绝对速度**而不是**接近速度**，`v²` 那一项在 0~2 m/s 内永远追不上真实需要的 `7v`。实测速度差小的场景够用，**但如果速度差超过 ~1 m/s，余量会不够**。速度差保持在小范围（0.2~0.3）就没有这个问题。

### 环长的硬约束

```
closing_speed × 急停耗时  <  可用间距（环长 / 车数 - 余量）
57.14m 上两台车 -> 每台 28.57m 可用，2 m/s 追 0.2 也要 12.6m 制动距离
```
**⇒ 环上协同的真正约束是「速度差」，不是「设定速度」。**

---

## 六、安全须知

- ⚠️ **`arm` 默认 false**。默认状态下本模块**只算、只显示，不下发任何车端动作**。这是刻意的。
- ⚠️ **`arm=true` 会真的让车紧急停车**，而车端急停是**终止性**的 —— 之后需要在车端控制台重新点「开始自主巡航」。
- ⚠️ **`convoy_observer.py` 没有任何 publisher**，它不会动车。所有动作都只可能来自 PC 的 `convoy_enforce()`。
- ⚠️ **网页急停不能替代物理急停或遥控接管。** 第一次开 `arm` 必须在封闭场地、低速、有人随时能接管的前提下试。
- ⚠️ **`runtime/convoy.json` 和 `convoy-observer.env` 里有令牌，别提交、别外传**（两个文件都已被 `.gitignore` 忽略）。

---

## 七、相关文件

| 文件 | 作用 |
|---|---|
| `backend/convoy_observer.py` | 车端观察器（Python 2.7 + ROS，只读订阅） |
| `backend/convoy.py` | 本机管理端协同调度器 |
| `backend/convoy_policy.py` | 路径投影与跟驰建议（**原有文件，未改动**） |
| `backend/fleet.py` | 加了 `/api/telemetry`、`/api/convoy` 和执行线程 |
| `deploy/convoy-run.sh` | 车端启动脚本 |
| `deploy/bigcar-convoy.service` | 车端 systemd 单元（`Restart=always`） |
| `runtime/convoy.example.json` | 本机管理端配置模板 |
| `runtime/convoy-observer.example.env` | 车端配置模板 |
