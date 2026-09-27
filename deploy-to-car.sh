#!/usr/bin/env bash
# 同步车端控制台（后端 + 前端）到一辆或多辆车，并重启服务。
#
#   ./deploy-to-car.sh 192.168.31.134 192.168.31.232
#
# 约定：
#   * 车端 SSH 密码固定为 nvidia（脚本内部使用，不会打印）。
#   * dist/ 必须和后端一起同步：只更新 backend/ 会出现「后端新版、页面旧包」的错位。
#   * 每次部署前自动备份被覆盖的文件到 runtime/../backups/deploy-<时间戳>/。
set -euo pipefail

[[ $# -ge 1 ]] || { echo "用法: $0 <车端IP> [更多IP...]" >&2; exit 1; }

HERE=$(cd "$(dirname "$0")" && pwd)
DEST=/home/nvidia/Desktop/bigcar-console
PW=nvidia

remote() {  # remote <ip> <命令>
  ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=no -o LogLevel=ERROR "nvidia@$1" 2>/dev/null \
    "echo $PW > /tmp/.dshpw; chmod 600 /tmp/.dshpw; sudo -S -p '' bash -c $(printf '%q' "$2") < /tmp/.dshpw 2>/dev/null; rm -f /tmp/.dshpw"
}

for IP in "$@"; do
  echo "=== $IP ==="
  if ! nc -z -G 3 "$IP" 22 2>/dev/null; then
    echo "  跳过：SSH 不可达（车辆离线？）"; continue
  fi
  STAMP=$(date +%Y%m%d-%H%M%S)
  remote "$IP" "mkdir -p $DEST/backups/deploy-$STAMP && cp -a $DEST/backend/app.py $DEST/backend/controller.py $DEST/backend/ros_probe.py $DEST/backups/deploy-$STAMP/ 2>/dev/null; cp -a $DEST/dist $DEST/backups/deploy-$STAMP/dist 2>/dev/null; true"
  echo "  已备份到 backups/deploy-$STAMP"

  scp -q -o ConnectTimeout=10 \
    "$HERE/backend/app.py" "$HERE/backend/controller.py" \
    "$HERE/backend/ros_bridge.py" "$HERE/backend/ros_probe.py" \
    "$HERE/backend/convoy_observer.py" \
    "nvidia@$IP:$DEST/backend/"
  rsync -a --delete --timeout=60 "$HERE/dist/" "nvidia@$IP:$DEST/dist/"

  # ⭐ 协同观察器（convoy_observer.py）：只读订阅位姿，10Hz 推给本机管理平台。
  #    token / 车名都从【本机管理端】现读，避免两边写叉；车名按 IP 反查 fleet.json。
  echo "  配置协同观察器"
  scp -q -o ConnectTimeout=10 "$HERE/deploy/convoy-run.sh" "$HERE/deploy/bigcar-convoy.service" \
    "nvidia@$IP:$DEST/deploy/"
  CONVOY_TOKEN=$(python3 -c "
import json
try:
    print(json.load(open('$HERE/runtime/convoy.json'))['observer_token'])
except Exception:
    print('')
" 2>/dev/null || echo '')
  CAR_NAME=$(python3 -c "
import json
try:
    for v in json.load(open('$HERE/runtime/fleet.json'))['vehicles'].values():
        if v.get('ip') == '$IP':
            print(v.get('name', '')); break
except Exception:
    pass
" 2>/dev/null || echo '')
  PC_IP=${FLEET_HOST:-$(python3 -c "
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(('$IP', 80)); print(s.getsockname()[0]); s.close()
" 2>/dev/null || echo '')}
  if [ -z "$CONVOY_TOKEN" ] || [ -z "$CAR_NAME" ] || [ -z "$PC_IP" ]; then
    echo "  ⚠️ 跳过 observer 配置：token='$CONVOY_TOKEN' 车名='$CAR_NAME' 本机IP='$PC_IP'"
    echo "     （先在本机跑一次 start.bat 生成 runtime/convoy.json，或手动设 FLEET_HOST=<你的IP>）"
  else
    cat > /tmp/convoy-observer.env.$$ <<ENV
# 由 deploy-to-car.sh 生成 $(date +%Y-%m-%d\ %H:%M:%S)
CONVOY_URL=http://$PC_IP:8870
CONVOY_TOKEN=$CONVOY_TOKEN
CONVOY_VEHICLE=$CAR_NAME
ENV
    scp -q -o ConnectTimeout=10 /tmp/convoy-observer.env.$$ "nvidia@$IP:$DEST/runtime/convoy-observer.env"
    rm -f /tmp/convoy-observer.env.$$
    echo "  observer 配置已推送到车（车名=$CAR_NAME，上报到 $PC_IP:8870）"
    remote "$IP" "chmod 755 $DEST/deploy/convoy-run.sh; cp $DEST/deploy/bigcar-convoy.service /etc/systemd/system/; systemctl daemon-reload; systemctl enable bigcar-convoy >/dev/null 2>&1; systemctl restart bigcar-convoy; sleep 2; systemctl is-active bigcar-convoy"
  fi

  remote "$IP" "chmod 644 $DEST/backend/ros_bridge.py; systemctl restart bigcar-console; sleep 3; systemctl is-active bigcar-console"
  remote "$IP" "grep -o 'index-[A-Za-z0-9_-]*\.js' $DEST/dist/index.html"
done
