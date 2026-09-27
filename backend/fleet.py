#!/usr/bin/env python3
"""Local fleet coordinator. Vehicle actions remain owned by the vehicle API."""
from __future__ import annotations

import argparse
import concurrent.futures
import copy
import http.client
import ipaddress
import json
import os
import select
import socket
import threading
import time
import uuid
from collections import Counter
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import quote, urlparse

from convoy import Convoy, ConvoyError

ROOT = Path(__file__).resolve().parents[1]
ACTIONS = ['environment_check', 'start_hardware', 'start_autoware', 'start_localization', 'load_route', 'start_tracking']
TITLES = ['环境检查', '底盘与雷达', 'Autoware', '地图与标定', '路径配置', '循迹运行']
TERMINAL = {'completed', 'failed', 'cancelled', 'interrupted'}
STEP_TERMINAL = {'ok', 'skipped', 'failed', 'cancelled'}


class FleetError(ValueError):
    pass


def blank_step(index):
    return {'index': index, 'title': TITLES[index - 1], 'status': 'pending', 'started': None, 'finished': None,
            'duration': None, 'detail': '', 'error': '', 'skipped': False, 'stage_before': None, 'stage_after': None}


def blank_row(vehicle, map_name, route):
    return {'vehicle_id': vehicle['id'], 'name': vehicle['name'], 'ip': vehicle['ip'], 'status': 'queued', 'step': 0,
            'message': '等待执行', 'map': map_name, 'route': route, 'manual_confirmed': False,
            'safety_confirmed': False, 'steps': [], 'events': [], 'started': None, 'finished': None,
            'attempts': 0, 'waiting_since': None, 'retried': False}


class Fleet:
    def __init__(self, path):
        self.path = Path(path)
        self.lock = threading.RLock()
        self.dispatch_lock = threading.RLock()
        self.settings = {'vehicle_port': 8765, 'poll_seconds': 3, 'step_timeout': 120, 'localization_timeout': 1800}
        self.vehicles = {}
        self.jobs = []
        self.states = {}
        # ⭐ 协同（跟驰）模块。配置故意放在【单独文件】而不是 settings ——
        #    因为 /api/fleet/settings 只接受固定的四个键，会把别的键丢掉。
        self.convoy_config = self.load_convoy_config()
        self.convoy = Convoy(self.convoy_config)
        self.convoy_route_attempt = 0.0
        self.convoy_armed = bool(self.convoy_config.get('arm'))
        if self.path.exists():
            saved = json.loads(self.path.read_text())
            self.settings.update(saved['settings'])
            self.vehicles = saved['vehicles']
            self.jobs = saved['jobs']
        for job in self.jobs:
            job.setdefault('phase', 0)
            job.setdefault('phase_state', 'done')
            for row in job['rows']:
                row.setdefault('steps', [])
                row.setdefault('events', [])
                row.setdefault('started', None)
                row.setdefault('finished', None)
                row.setdefault('attempts', 0)
                row.setdefault('waiting_since', None)
                row.setdefault('retried', False)
                if row['status'] not in TERMINAL:
                    row.update(status='interrupted', message='管理服务已重启，请核对车端状态后重试',
                               finished=time.time())
                    self.close_open_steps(row, 'cancelled', '管理服务重启')
        self.save()

    def save(self):
        with self.lock:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.path.with_suffix('.tmp')
            fd = os.open(str(temporary), os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
            with os.fdopen(fd, 'w') as handle:
                json.dump({'settings': self.settings, 'vehicles': self.vehicles, 'jobs': self.jobs}, handle, ensure_ascii=False)
            os.replace(temporary, self.path)
            os.chmod(self.path, 0o600)

    def vehicle(self, identifier):
        with self.lock:
            if identifier not in self.vehicles:
                raise FleetError('车辆不存在')
            return dict(self.vehicles[identifier])

    # ---------------------------------------------------------------- 协同（跟驰）

    def load_convoy_config(self):
        """协同模块的配置：单独文件 runtime/convoy.json。

        ⚠️ 不放进 self.settings —— /api/fleet/settings 只接受四个固定键，
           塞进去的别的键会在下次保存时被丢掉。
        """
        path = self.path.parent / 'convoy.json'
        try:
            if path.exists():
                data = json.loads(path.read_text())
                if isinstance(data, dict):
                    return data
        except Exception as exc:
            print(f'[convoy] 配置读取失败，改用默认值：{exc}', flush=True)
        return {}

    def telemetry(self, payload):
        """接收一条车端 observer 推来的遥测（只读，绝不下发车端动作）。

        路径是【懒加载】的：第一次拿到遥测时，从车端拉一次 /api/route，
        交给 convoy_policy 做投影准备。失败按 15 秒限流重试，不会每帧都打。
        """
        self.ensure_convoy_route(payload)
        return self.convoy.ingest(payload)

    def ensure_convoy_route(self, payload):
        if self.convoy.route is not None:
            return
        name = str((payload or {}).get('route') or '').strip()
        if not name:
            return
        now = time.time()
        if now - self.convoy_route_attempt < 15:
            return
        self.convoy_route_attempt = now
        with self.lock:
            vehicles = list(self.vehicles.values())
        for vehicle in vehicles:
            try:
                data = self.request(vehicle, f'/api/route?file={quote(name)}')
                points = [(float(p['x']), float(p['y'])) for p in data.get('points', [])]
                if len(points) < 4:
                    continue
                length = self.convoy.load_route(points, name)
                print(f'[convoy] 已加载路径 {name}：{len(points)} 点 / {length:.2f} m', flush=True)
                return
            except Exception:
                continue

    def convoy_view(self):
        """协同状态：每车的位置/健康度 + 两两跟驰建议。"""
        view = self.convoy.status()
        view['armed'] = self.convoy_armed
        return view

    def find_vehicle(self, name):
        """按车辆名称找 id。车端 observer 上报的是名字（不是平台的内部 id）。"""
        with self.lock:
            for identifier, vehicle in self.vehicles.items():
                if vehicle.get('name') == name:
                    return identifier
        return None

    def convoy_enforce(self):
        """执行一次协同建议。⚠️ 只在 arm=True 时由后台线程调用。

        ⭐ 目前【只实现 stop】：
             stop  -> self.emergency()，即车端 emergency_stop —— 这条已经实测验证有效。
        ⚠️  deliberately 不实现 slow：
             slow 需要"往下压 velocity_max"，而那条路（运行时降速）我们还没在真车上
             验证过到底生不生效。宁可少做，也不要在没验证的通道上下发。
             想开的时候再补，代码里留了位置。

        返回本次真正发出的动作列表（空列表 = 什么都没做），便于审计。
        """
        if not self.convoy_armed:
            return []
        done = []
        for advice in self.convoy_view()['advice']:
            if advice.get('state') != 'stop':
                continue
            name = advice.get('behind')
            identifier = self.find_vehicle(name)
            if not identifier:
                done.append({'behind': name, 'result': 'vehicle_not_registered'})
                continue
            try:
                self.emergency(identifier)
                done.append({'behind': name, 'result': 'emergency_stop_sent',
                             'reason': advice.get('reason')})
                print(f'[convoy] 执行停车避让：{name} —— {advice.get("reason")}', flush=True)
            except Exception as exc:
                done.append({'behind': name, 'result': f'failed: {exc}'})
                print(f'[convoy] 停车避让下发失败：{name} —— {exc}', flush=True)
        return done

    def request(self, vehicle, path='/api/state', body=None):
        connection = http.client.HTTPConnection(vehicle['ip'], vehicle['port'], timeout=12)
        try:
            headers = {'X-Control-Token': vehicle['token'], 'Content-Type': 'application/json'}
            connection.request('POST' if body is not None else 'GET', path,
                               json.dumps(body).encode() if body is not None else None, headers)
            response = connection.getresponse()
            data = json.loads(response.read(4 * 1024 * 1024))
            if response.status >= 400:
                raise FleetError(data.get('error', f'车端返回 HTTP {response.status}'))
            return data
        except (OSError, http.client.HTTPException, ValueError) as exc:
            raise FleetError(str(exc)) from exc
        finally:
            connection.close()

    @staticmethod
    def validate_state(state):
        if not isinstance(state, dict) or not isinstance(state.get('current_stage'), int) or not isinstance(state.get('workflow'), list) or not isinstance(state.get('telemetry'), dict):
            raise FleetError('目标不是兼容的智能驾驶控制台')

    def add(self, data):
        name = str(data.get('name', '')).strip()
        try:
            address = ipaddress.ip_address(str(data.get('ip', '')).strip())
        except ValueError as exc:
            raise FleetError('请输入有效 IP，无需端口') from exc
        if address.is_unspecified or address.is_multicast or address.is_link_local:
            raise FleetError('该 IP 不可用')
        if not name or len(name) > 60:
            raise FleetError('车辆名称须为 1–60 个字符')
        vehicle = {'id': uuid.uuid4().hex, 'name': name, 'ip': str(address), 'port': self.settings['vehicle_port'], 'token': str(data.get('token', ''))}
        state = self.request(vehicle)
        self.validate_state(state)
        with self.lock:
            if any(v['ip'] == vehicle['ip'] for v in self.vehicles.values()):
                raise FleetError('该 IP 已添加')
            self.vehicles[vehicle['id']] = vehicle
            self.states[vehicle['id']] = {'online': True, 'state': state, 'updated': time.time(), 'error': ''}
            self.save()
        return {'ok': True}

    def refresh(self, identifier):
        try:
            state = self.request(self.vehicle(identifier))
            self.validate_state(state)
            result = {'online': True, 'state': state, 'updated': time.time(), 'error': ''}
        except Exception as exc:
            result = {'online': False, 'state': None, 'updated': time.time(), 'error': str(exc)}
        with self.lock:
            if identifier in self.vehicles:
                self.states[identifier] = result
        return result

    def poll(self):
        with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
            while True:
                with self.lock:
                    identifiers = list(self.vehicles)
                list(pool.map(self.refresh, identifiers))
                time.sleep(self.settings['poll_seconds'])

    @staticmethod
    def summarise(job):
        counts = Counter(row['status'] for row in job['rows'])
        finished = [row['finished'] for row in job['rows'] if row.get('finished')]
        started = [row['started'] for row in job['rows'] if row.get('started')]
        live = [row for row in job['rows'] if row['status'] not in TERMINAL]
        return {
            'total': len(job['rows']),
            'completed': counts.get('completed', 0),
            'failed': counts.get('failed', 0) + counts.get('interrupted', 0),
            'cancelled': counts.get('cancelled', 0),
            'running': sum(1 for row in live if row['status'] in ('queued', 'running')),
            'waiting': sum(1 for row in live if row['status'] in ('awaiting_localization', 'awaiting_start')),
            'active': len(live),
            'started': min(started) if started else job['created'],
            'finished': max(finished) if finished and not live else None,
        }

    @staticmethod
    def phase_state(job):
        live = [row for row in job['rows'] if row['status'] not in TERMINAL]
        if not live:
            return 'done' if job.get('phase_state') == 'done' else 'idle'
        if any(row['status'] == 'awaiting_localization' for row in live):
            return 'localization'
        if any(row['status'] == 'awaiting_start' for row in live):
            return 'start'
        return 'running'

    def job_view(self, job):
        view = copy.deepcopy(job)
        view['summary'] = self.summarise(job)
        view['target_title'] = TITLES[job['target'] - 1]
        view['phase'] = job.get('phase', 0)
        view['phase_state'] = self.phase_state(job)
        view['phase_title'] = TITLES[view['phase'] - 1] if 1 <= view['phase'] <= len(TITLES) else ''
        return view

    def snapshot(self):
        with self.lock:
            vehicles = [{**{k: v for k, v in vehicle.items() if k != 'token'}, **self.states.get(identifier, {'online': False, 'state': None, 'error': '等待连接'})} for identifier, vehicle in self.vehicles.items()]
            return copy.deepcopy({'vehicles': vehicles, 'jobs': [self.job_view(job) for job in self.jobs],
                                  'settings': self.settings, 'timestamp': time.time()})

    def change(self, row, **values):
        with self.lock:
            # A cancelled row must never be resurrected by a late worker update.
            if row['status'] == 'cancelled' and values.get('status', 'cancelled') != 'cancelled':
                return
            row.update(values, updated=time.time())
            self.save()

    # ---------------------------------------------------------------- step ledger
    def begin_step(self, row, step, stage_before):
        with self.lock:
            steps = row.setdefault('steps', [])
            while len(steps) < step:
                steps.append(blank_step(len(steps) + 1))
            record = steps[step - 1]
            record.update(status='running', started=time.time(), finished=None, duration=None, error='',
                          skipped=False, stage_before=stage_before, stage_after=None)
            self.save()
            return record

    def finish_step(self, row, step, skipped, stage_after, detail):
        with self.lock:
            record = row['steps'][step - 1]
            record.update(status='skipped' if skipped else 'ok', skipped=bool(skipped), finished=time.time(),
                          duration=round(time.time() - (record['started'] or time.time()), 2),
                          stage_after=stage_after, detail=detail)
            self.save()

    def fail_step(self, row, step, error):
        with self.lock:
            steps = row.setdefault('steps', [])
            if step - 1 < len(steps) and steps[step - 1]['status'] == 'running':
                record = steps[step - 1]
                record.update(status='failed', error=error, finished=time.time(),
                              duration=round(time.time() - (record['started'] or time.time()), 2))
            self.save()

    def close_open_steps(self, row, status, reason):
        with self.lock:
            for record in row.get('steps', []):
                if record['status'] == 'running':
                    record.update(status=status, error=reason, finished=time.time(),
                                  duration=round(time.time() - (record['started'] or time.time()), 2))
            self.save()

    def note(self, row, level, message, step=None):
        with self.lock:
            row.setdefault('events', []).append({'time': time.time(), 'level': level, 'step': step or row.get('step') or 0, 'message': message})
            del row['events'][:-200]
            self.save()

    # ---------------------------------------------------------------- job creation
    def create_job(self, data):
        identifiers = data.get('vehicles', [])
        if not isinstance(identifiers, list) or not identifiers or len(identifiers) > 50 or len(set(identifiers)) != len(identifiers):
            raise FleetError('请选择 1–50 辆车辆')
        with self.lock:
            occupied = {row['vehicle_id'] for job in self.jobs for row in job['rows'] if row['status'] not in TERMINAL}
            rows = []
            for identifier in identifiers:
                vehicle = self.vehicle(identifier)
                if identifier in occupied:
                    raise FleetError(f"{vehicle['name']} 已有进行中的任务，请先取消或放弃该任务")
                cached = self.states.get(identifier, {})
                if not cached.get('online'):
                    raise FleetError(f"{vehicle['name']} 离线：{cached.get('error') or '等待连接'}")
                state = cached['state']
                files = data.get('files', {}).get(identifier, {})
                map_name = files.get('map', state['selected_map'])
                route = files.get('route', state['selected_route'])
                if map_name not in [f['name'] for f in state['maps']]:
                    raise FleetError(f"请为 {vehicle['name']} 选择地图")
                if route not in [f['name'] for f in state['routes']]:
                    raise FleetError(f"请为 {vehicle['name']} 选择路径")
                if state['emergency']:
                    raise FleetError(f"{vehicle['name']} 处于急停状态，请先在车端解除")
                if state['busy']:
                    raise FleetError(f"{vehicle['name']} 正在执行车端动作（{state['busy']}），请稍后再试")
                if state['current_stage'] >= 6:
                    raise FleetError(f"{vehicle['name']} 已在循迹运行，请先在本页复位该车或直接在车端接管")
                rows.append(blank_row(vehicle, map_name, route))
            # A batch always walks the whole six-step flow, one step for every car
            # before the next step starts. There is no per-car finish line.
            job = {'id': uuid.uuid4().hex, 'created': time.time(), 'target': len(ACTIONS), 'phase': 0,
                   'phase_state': 'queued', 'rows': rows}
            self.jobs.insert(0, job)
            self.save()
        threading.Thread(target=self.run_job, args=(job,), daemon=True).start()
        return {'ok': True, 'id': job['id']}

    def active_rows(self, job):
        return [row for row in job['rows'] if row['status'] not in TERMINAL]

    def set_phase(self, job, phase, phase_state):
        with self.lock:
            job.update(phase=phase, phase_state=phase_state, updated=time.time())
            self.save()

    def run_job(self, job):
        """Step-synchronised batch: every car finishes step N before step N+1 starts.

        Each step runs one thread per car; joining them is the barrier. A car that
        fails the step drops out of the batch while the rest continue. Vehicle
        actions stay serialised by dispatch_lock, so only one car is commanded at
        a time even though the step itself is prepared in parallel.
        """
        for step in range(1, job['target'] + 1):
            active = self.active_rows(job)
            if not active:
                break
            self.set_phase(job, step, 'running')
            threads = [threading.Thread(target=self.run_row_step, args=(job, row, step), daemon=True) for row in active]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join()
        self.set_phase(job, job['target'], 'done')

    def wait_state(self, row, predicate):
        deadline = time.monotonic() + self.settings['step_timeout']
        misses = 0
        while time.monotonic() < deadline:
            if row['status'] == 'cancelled':
                return False
            result = self.refresh(row['vehicle_id'])
            if not result['online']:
                # A single dropped poll on a flaky Wi-Fi link must not fail the row.
                misses += 1
                if misses >= 3:
                    raise FleetError('车辆连接中断：' + result['error'])
                time.sleep(0.5)
                continue
            misses = 0
            state = result['state']
            if state['emergency']:
                raise FleetError('车端已急停')
            if state['last_error'] and not state['busy']:
                raise FleetError(state['last_error'])
            if not state['busy'] and predicate(state):
                return state
            time.sleep(0.5)
        raise FleetError('等待车端状态超时')

    @staticmethod
    def step_detail(state, step, fallback):
        workflow = state.get('workflow') or []
        for item in workflow:
            if item.get('id') == step and item.get('detail'):
                return item['detail']
        return fallback

    def wait_localization(self, row, vehicle):
        """Wait for the human, but reconcile with what the car actually did."""
        limit = int(self.settings.get('localization_timeout') or 0)
        deadline = time.monotonic() + limit if limit else None
        self.change(row, waiting_since=time.time())
        while not row['manual_confirmed']:
            if row['status'] == 'cancelled':
                return False
            result = self.refresh(row['vehicle_id'])
            if result['online']:
                state = result['state']
                if state['emergency']:
                    raise FleetError('车端已急停')
                if state['current_stage'] >= 5:
                    self.change(row, manual_confirmed=True, status='running', message='车端已自行进入路径配置，平台对账放行')
                    self.note(row, 'WARN', f"人工定位已由车端完成（当前阶段 {state['current_stage']}），平台自动对账放行", 4)
                    return True
            if deadline and time.monotonic() > deadline:
                raise FleetError(f'等待人工定位超过 {limit // 60} 分钟，已释放该车；可重新发起任务')
            time.sleep(0.5)
        return True

    def run_row_step(self, job, row, step):
        """Run exactly one step of the flow for one car and settle it."""
        try:
            if row['status'] in TERMINAL:
                return
            vehicle = self.vehicle(row['vehicle_id'])
            self.change(row, status='running', started=row.get('started') or time.time(), finished=None,
                        attempts=int(row.get('attempts') or 0) + 1)
            state = self.request(vehicle)
            if state['emergency'] or state['busy']:
                raise FleetError('车端急停或正在执行其他动作')
            if state['current_stage'] >= 6 and step < 6:
                raise FleetError('车辆已在其他会话进入循迹，已停止本任务')
            # Map and route choices must be applied even when ROS already reached that stage.
            skip = step <= 3 and state['current_stage'] >= step
            self.begin_step(row, step, state['current_stage'])
            self.change(row, status='running', step=step, message=TITLES[step - 1])
            if step == 6:
                self.change(row, status='awaiting_start', message='等待循迹安全确认', waiting_since=time.time())
                while not row['safety_confirmed']:
                    if row['status'] == 'cancelled':
                        self.close_open_steps(row, 'cancelled', '任务已取消')
                        return
                    time.sleep(0.2)
            if not skip:
                with self.dispatch_lock:
                    if row['status'] == 'cancelled':
                        self.close_open_steps(row, 'cancelled', '任务已取消')
                        return
                    self.request(vehicle, '/api/action', {'action': ACTIONS[step - 1], 'map': row['map'], 'route': row['route'], 'safety_confirmed': row['safety_confirmed']})
                self.note(row, 'INFO', f"已下发动作 {ACTIONS[step - 1]}", step)
                if step == 4:
                    self.wait_state(row, lambda s: True)
                    self.change(row, status='awaiting_localization', message='等待人工定位')
                    if not self.wait_localization(row, vehicle):
                        self.close_open_steps(row, 'cancelled', '任务已取消')
                        return
                state = self.wait_state(row, lambda s: s['current_stage'] >= step)
                if state is False:
                    self.close_open_steps(row, 'cancelled', '任务已取消')
                    return
            else:
                self.note(row, 'INFO', f"车端已处于阶段 {state['current_stage']}，跳过该步动作", step)
            after = self.refresh(row['vehicle_id'])
            stage_after = after['state']['current_stage'] if after['online'] else None
            detail = self.step_detail(after['state'], step, '已跳过（车端已就绪）' if skip else '状态校验通过') if after['online'] else '状态校验通过'
            self.finish_step(row, step, skip, stage_after, detail)
            self.note(row, 'INFO', f"{TITLES[step - 1]}：{detail}", step)
            if step == job['target']:
                self.change(row, status='completed', message='六步流程完成', finished=time.time(), waiting_since=None)
            else:
                self.change(row, status='running', message=TITLES[step - 1] + '完成，等待其他车辆同步')
        except Exception as exc:
            message = str(exc)
            self.fail_step(row, step, message)
            self.close_open_steps(row, 'failed', message)
            self.note(row, 'ERROR', message, step)
            self.change(row, status='failed', message=message, finished=time.time(), waiting_since=None)

    # ---------------------------------------------------------------- row actions
    def job_action(self, identifier, data):
        with self.lock:
            job = next((j for j in self.jobs if j['id'] == identifier), None)
            if not job:
                raise FleetError('任务不存在')
            row = next((r for r in job['rows'] if r['vehicle_id'] == data.get('vehicle_id')), None)
        if not row:
            raise FleetError('该车辆不在本任务中')
        action = data.get('action')
        if action == 'localization_done':
            if row['status'] != 'awaiting_localization':
                raise FleetError('该车当前不在人工定位环节')
            state = self.request(self.vehicle(row['vehicle_id']))
            if state['busy'] or state['emergency'] or state['current_stage'] < 4 or (not state.get('simulated') and '/current_pose' not in state['live_topics']):
                raise FleetError('定位数据尚未就绪，请完成 RViz 初始位姿标定')
            self.note(row, 'INFO', '人工确认：定位数据校验通过', 4)
            self.change(row, manual_confirmed=True, status='running', message='定位已确认，正在继续流程', waiting_since=None)
        elif action == 'confirm_start':
            if row['status'] != 'awaiting_start' or data.get('safety_confirmed') is not True:
                raise FleetError('该车当前不在循迹确认环节')
            self.note(row, 'WARN', '现场安全已确认，开始循迹', 6)
            self.change(row, safety_confirmed=True, status='running', message='正在启动循迹', waiting_since=None)
        elif action == 'cancel':
            if row['status'] in TERMINAL:
                raise FleetError('该车辆任务已结束')
            with self.dispatch_lock:
                self.close_open_steps(row, 'cancelled', '用户取消')
                self.note(row, 'WARN', '用户取消了后续步骤；车端已运行的节点保持不动')
                self.change(row, status='cancelled', message='已取消后续步骤；车端现有节点保持运行',
                            finished=time.time(), waiting_since=None)
        elif action == 'retry':
            if row['status'] not in TERMINAL:
                raise FleetError('仅失败、中断或已取消的任务可以重试')
            if row.get('retried'):
                raise FleetError('该车辆记录已重试过，请在最新任务中继续操作')
            vehicle = self.vehicle(row['vehicle_id'])
            cached = self.states.get(row['vehicle_id'], {})
            if not cached.get('online'):
                raise FleetError(f"{vehicle['name']} 离线，无法重试")
            state = cached['state']
            if state['emergency'] or state['busy']:
                raise FleetError(f"{vehicle['name']} 急停或正在执行车端动作，无法重试")
            if state['current_stage'] >= 6:
                raise FleetError(f"{vehicle['name']} 已在循迹运行，请先复位该车")
            new_row = blank_row(vehicle, row['map'], row['route'])
            new_row['attempts'] = int(row.get('attempts') or 0)
            self.change(row, retried=True)
            new_job = {'id': uuid.uuid4().hex, 'created': time.time(), 'target': job['target'], 'phase': 0,
                       'phase_state': 'queued', 'rows': [new_row]}
            with self.lock:
                self.jobs.insert(0, new_job)
                self.save()
            threading.Thread(target=self.run_job, args=(new_job,), daemon=True).start()
            return {'ok': True, 'id': new_job['id']}
        else:
            raise FleetError('当前状态不支持该操作')
        return {'ok': True}

    def cancel_job(self, identifier):
        """Stop a batch: cancel every car that has not reached a terminal state."""
        with self.lock:
            job = next((j for j in self.jobs if j['id'] == identifier), None)
            if not job:
                raise FleetError('任务不存在')
            live = [row for row in job['rows'] if row['status'] not in TERMINAL]
            if not live:
                raise FleetError('该任务已结束')
        with self.dispatch_lock:
            for row in live:
                self.close_open_steps(row, 'cancelled', '整批取消')
                self.note(row, 'WARN', '用户取消了整批任务；车端已运行的节点保持不动')
                self.change(row, status='cancelled', message='已取消后续步骤；车端现有节点保持运行',
                            finished=time.time(), waiting_since=None)
        self.set_phase(job, job.get('phase', 0), 'done')
        return {'ok': True, 'cancelled': len(live)}

    def delete_job(self, identifier):
        with self.lock:
            job = next((j for j in self.jobs if j['id'] == identifier), None)
            if not job:
                raise FleetError('任务不存在')
            live = [row['name'] for row in job['rows'] if row['status'] not in TERMINAL]
            if live:
                raise FleetError('仍有进行中的车辆，请先取消或放弃：' + '、'.join(live))
            self.jobs = [j for j in self.jobs if j['id'] != identifier]
            self.save()
        return {'ok': True}

    def clear_jobs(self):
        with self.lock:
            live = [job['id'] for job in self.jobs if any(row['status'] not in TERMINAL for row in job['rows'])]
            if live:
                raise FleetError(f'仍有 {len(live)} 个任务包含进行中的车辆，请先处理')
            removed = len(self.jobs)
            self.jobs = []
            self.save()
        return {'ok': True, 'removed': removed}

    def reset_vehicle(self, identifier):
        """Ask the car console to reset its own workflow so it can be batched again."""
        vehicle = self.vehicle(identifier)
        with self.lock:
            busy = [row['name'] for row in (r for j in self.jobs for r in j['rows'])
                    if row['vehicle_id'] == identifier and row['status'] not in TERMINAL]
            if busy:
                raise FleetError(f"{vehicle['name']} 仍有进行中的任务，请先取消或放弃")
        return self.request(vehicle, '/api/action', {'action': 'restart_workflow'})

    def emergency(self, identifier):
        with self.dispatch_lock:
            with self.lock:
                for job in self.jobs:
                    for row in job['rows']:
                        if row['vehicle_id'] == identifier and row['status'] not in TERMINAL:
                            self.close_open_steps(row, 'cancelled', '急停')
                            self.change(row, status='cancelled', message='已取消任务并请求急停',
                                        finished=time.time(), waiting_since=None)
            return self.request(self.vehicle(identifier), '/api/action', {'action': 'emergency_stop'})


class Handler(SimpleHTTPRequestHandler):
    fleet: Fleet

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT / 'dist'), **kwargs)

    def log_message(self, fmt, *args):
        if '/api/fleet/state' not in self.path:
            super().log_message(fmt, *args)

    def valid_origin(self):
        host = self.headers.get('Host', '')
        origin = self.headers.get('Origin')
        return host in (f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}') and (not origin or origin in (f'http://{host}', f'https://{host}'))

    def reply(self, body, status=200):
        encoded = json.dumps(body, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(encoded)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(encoded)

    def do_GET(self):
        if not self.valid_origin():
            self.reply({'error': '仅允许本机同源访问'}, 403)
            return
        try:
            parts = urlparse(self.path).path.strip('/').split('/')
            if self.path == '/api/fleet/state':
                self.reply(self.fleet.snapshot())
            elif self.path == '/api/convoy':
                # ⭐ 协同（跟驰）状态：每车位置/位姿健康度 + 两两跟驰建议。
                #    只读接口，不下发任何车端动作。仅本机同源可访问（valid_origin 已校验）。
                self.reply(self.fleet.convoy_view())
            elif len(parts) == 7 and parts[:3] == ['api', 'fleet', 'vehicles'] and parts[4:] == ['proxy', 'api', 'screen']:
                self.screen(self.fleet.vehicle(parts[3]))
            elif self.path.startswith('/api/'):
                self.reply({'error': '接口不存在'}, 404)
            else:
                if self.path == '/':
                    self.send_response(302)
                    self.send_header('Location', '/fleet')
                    self.end_headers()
                    return
                if self.path.startswith('/fleet'):
                    self.path = '/index.html'
                super().do_GET()
        except Exception as exc:
            self.reply({'error': str(exc)}, 400)

    def do_POST(self):
        # ⭐ 车端 observer 是从 192.168.31.x 推过来的：既不是本机同源、也没有 Origin，
        #    所以必须在 valid_origin 之前单独放行 —— 它改用独立令牌（X-Observer-Token）把关。
        if urlparse(self.path).path == '/api/telemetry':
            self.telemetry()
            return
        if not self.valid_origin() or 'application/json' not in self.headers.get('Content-Type', ''):
            self.reply({'error': '仅允许本机同源 JSON 请求'}, 403)
            return
        try:
            length = int(self.headers.get('Content-Length', 0))
            if not 0 < length <= 65536:
                raise FleetError('无效请求长度')
            data = json.loads(self.rfile.read(length))
            if not isinstance(data, dict):
                raise FleetError('无效 JSON 对象')
            parts = urlparse(self.path).path.strip('/').split('/')
            if parts == ['api', 'fleet', 'vehicles']:
                result = self.fleet.add(data)
            elif parts == ['api', 'fleet', 'jobs']:
                result = self.fleet.create_job(data)
            elif parts == ['api', 'fleet', 'jobs', 'clear']:
                result = self.fleet.clear_jobs()
            elif len(parts) == 5 and parts[:3] == ['api', 'fleet', 'jobs'] and parts[4] == 'delete':
                result = self.fleet.delete_job(parts[3])
            elif len(parts) == 5 and parts[:3] == ['api', 'fleet', 'jobs'] and parts[4] == 'cancel':
                result = self.fleet.cancel_job(parts[3])
            elif len(parts) == 4 and parts[:3] == ['api', 'fleet', 'jobs']:
                result = self.fleet.job_action(parts[3], data)
            elif parts == ['api', 'fleet', 'settings']:
                settings = {key: int(data[key]) for key in ('vehicle_port', 'poll_seconds', 'step_timeout', 'localization_timeout')}
                if not 1 <= settings['vehicle_port'] <= 65535 or not 2 <= settings['poll_seconds'] <= 60 or not 10 <= settings['step_timeout'] <= 600 or not 0 <= settings['localization_timeout'] <= 86400:
                    raise FleetError('设置值超出允许范围')
                with self.fleet.lock:
                    self.fleet.settings = settings
                    self.fleet.save()
                result = {'ok': True}
            elif len(parts) >= 5 and parts[:3] == ['api', 'fleet', 'vehicles']:
                vehicle = self.fleet.vehicle(parts[3])
                if parts[4:] == ['proxy', 'api', 'screen-ticket']:
                    result = self.fleet.request(vehicle, '/api/screen-ticket', {})
                elif parts[4:] == ['emergency']:
                    result = self.fleet.emergency(parts[3])
                elif parts[4:] == ['reset']:
                    result = self.fleet.reset_vehicle(parts[3])
                elif parts[4:] == ['remove']:
                    with self.fleet.lock:
                        if any(r['vehicle_id'] == parts[3] and r['status'] not in TERMINAL for j in self.fleet.jobs for r in j['rows']):
                            raise FleetError('请先结束该车辆任务')
                        del self.fleet.vehicles[parts[3]]
                        self.fleet.states.pop(parts[3], None)
                        self.fleet.save()
                    result = {'ok': True}
                elif parts[4:] == ['update']:
                    name = str(data.get('name', '')).strip()
                    if not name or len(name) > 60:
                        raise FleetError('车辆名称须为 1–60 个字符')
                    with self.fleet.lock:
                        self.fleet.vehicles[parts[3]]['name'] = name
                        if data.get('token'):
                            self.fleet.vehicles[parts[3]]['token'] = str(data['token'])
                        self.fleet.save()
                    result = {'ok': True}
                else:
                    raise FleetError('接口不存在')
            else:
                raise FleetError('接口不存在')
            self.reply(result)
        except Exception as exc:
            self.reply({'error': str(exc)}, 400)

    def telemetry(self):
        """接收车端 convoy_observer.py 的遥测（只读）。

        和别的接口不同，这个走【独立令牌】而不是 valid_origin：
        车端从 192.168.31.x 推过来，Host/Origin 都不可能是本机。
        """
        try:
            if 'application/json' not in self.headers.get('Content-Type', ''):
                raise ConvoyError('需要 JSON 请求体')
            if not self.fleet.convoy.check_token(self.headers.get('X-Observer-Token')):
                self.reply({'error': '观察器令牌无效'}, 401)
                return
            length = int(self.headers.get('Content-Length', 0))
            if not 0 < length <= 65536:
                raise ConvoyError('无效请求长度')
            payload = json.loads(self.rfile.read(length))
            self.reply(self.fleet.telemetry(payload))
        except Exception as exc:
            self.reply({'error': str(exc)}, 400)

    def screen(self, vehicle):
        if self.headers.get('Upgrade', '').lower() != 'websocket':
            raise FleetError('需要 WebSocket')
        query = urlparse(self.path).query
        upstream = socket.create_connection((vehicle['ip'], vehicle['port']), timeout=10)
        try:
            headers = [f'GET /api/screen?{query} HTTP/1.1', f"Host: {vehicle['ip']}:{vehicle['port']}", 'Connection: Upgrade', 'Upgrade: websocket']
            for key in ('Sec-WebSocket-Key', 'Sec-WebSocket-Version', 'Sec-WebSocket-Protocol'):
                if self.headers.get(key):
                    headers.append(f'{key}: {self.headers[key]}')
            upstream.sendall(('\r\n'.join(headers) + '\r\n\r\n').encode())
            upstream.settimeout(None)
            self.close_connection = True
            while True:
                readable, _, _ = select.select([upstream, self.connection], [], [], 60)
                if not readable:
                    continue
                for source in readable:
                    chunk = source.recv(65536)
                    if not chunk:
                        return
                    (self.connection if source is upstream else upstream).sendall(chunk)
        finally:
            upstream.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8870)
    parser.add_argument('--data', type=Path, default=ROOT / 'runtime' / 'fleet.json')
    args = parser.parse_args()
    fleet = Fleet(args.data)
    handler = type('FleetHandler', (Handler,), {'fleet': fleet})
    server = ThreadingHTTPServer(('127.0.0.1', args.port), handler)
    threading.Thread(target=fleet.poll, daemon=True).start()

    # ⭐ 协同（跟驰）执行线程：只在配置里 arm=true 时才真跑。
    #    默认不 arm —— 因为"真的动车的速度/停它"必须是有人明确打开的决定。
    if fleet.convoy_armed:
        def convoy_loop():
            while True:
                try:
                    fleet.convoy_enforce()
                except Exception as exc:
                    print(f'[convoy] 执行循环异常：{exc}', flush=True)
                time.sleep(1.0)
        threading.Thread(target=convoy_loop, daemon=True).start()
        print('[convoy] ⚠️ 协同执行已开启（arm=true）—— 将按建议真的下发急停', flush=True)
    else:
        print('[convoy] 协同处于【只读】模式（arm=false）：只算建议，不下发任何车端动作', flush=True)

    print(f'智能驾驶管理平台 http://127.0.0.1:{args.port}/fleet', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
