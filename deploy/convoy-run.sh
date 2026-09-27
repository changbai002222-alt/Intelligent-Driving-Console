#!/bin/bash
# 启动车端「协同观察器」（convoy_observer.py）。
#
# 它做的事：只订阅 /current_pose 和 /ctrl_fb，把位姿+速度+年龄+地图/路径哈希
# 以 10Hz POST 到本机管理平台的 /api/telemetry。⚠️ 它没有任何 publisher，
# 不会对车发任何指令 —— 这一点由 convoy_observer.py 自己保证。
#
# 配置来源：runtime/convoy-observer.env
#     CONVOY_URL      = 本机管理平台地址，例如 http://192.168.31.74:8870
#     CONVOY_TOKEN    = 与后端 runtime/convoy.json 的 observer_token 【必须一致】
#     CONVOY_VEHICLE  = 这台车的名字，必须与管理平台上登记的车辆名一致（例：实训车01）
#
# observer 自己读的是 runtime/convoy-observer.json（硬编码路径），
# 所以这里先用 env 的值把那个 JSON 生成出来，再去启动它 —— 这样不必改 observer 本身。
set -e

root=/home/nvidia/Desktop/bigcar-console
env_file="$root/runtime/convoy-observer.env"
if [ -f "$env_file" ]; then
  set -a
  . "$env_file"
  set +a
fi

if [ -z "${CONVOY_URL:-}" ] || [ -z "${CONVOY_TOKEN:-}" ] || [ -z "${CONVOY_VEHICLE:-}" ]; then
  echo "convoy-observer.env 缺少配置（需要 CONVOY_URL / CONVOY_TOKEN / CONVOY_VEHICLE）" >&2
  exit 2
fi

# 生成 observer 要读的配置（宿主机路径；容器内是 /from_host/bigcar-console/runtime/...）
/usr/bin/python3 - "$root/runtime/convoy-observer.json" "$CONVOY_URL" "$CONVOY_TOKEN" "$CONVOY_VEHICLE" <<'PY'
import json, sys
path, url, token, vehicle = sys.argv[1:5]
with open(path, 'w') as handle:
    json.dump({'url': url.rstrip('/'), 'token': token, 'vehicle': vehicle},
              handle, ensure_ascii=False, indent=2)
PY

# python2 + ROS 在容器里；和 person_camera_observer 一样走 docker exec
exec docker exec autoware_ai_orin bash -c '
  source /opt/ros/melodic/setup.bash
  exec python2 /from_host/bigcar-console/backend/convoy_observer.py'
