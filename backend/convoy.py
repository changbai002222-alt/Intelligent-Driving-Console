"""两车协同（跟驰）调度器。

设计原则（沿用 convoy_policy.py 的原话）：
    Read-only route projection and following recommendations; never commands ROS.

本模块只做三件事：
    ① 接收车端 convoy_observer.py 推来的遥测（位姿 + 速度 + 年龄 + 地图/路径哈希）
    ② 把位姿可靠地投影到环形路径上（复用 convoy_policy：防跳变 / 朝向筛选 / 歧义检测）
    ③ 产出跟驰建议（stop / slow / clear）

它【绝不】下发任何车端动作。要不要执行、怎么执行，由调用方（fleet.py 的开关）决定。
这样即使本模块逻辑有误，也不会把车动起来。
"""
from __future__ import annotations

import threading
import time

from convoy_policy import FollowingAdvice, LoopRoute

# 位姿"还能用"的年龄上限（秒）。observer 会在 payload 里给 pose_age。
DEFAULT_FRESH_SECONDS = 1.5
# 同一台车两次遥测之间，允许的 s 跳变量（米/秒 * 秒 + 余量）—— 由 convoy_policy 内部处理，
# 这里只做"我们记不记得上一次"的记录。
HASH_KEYS = ('map_hash', 'route_hash')


class ConvoyError(ValueError):
    """协同模块自己的错误，和车端无关。"""


class VehicleTrack:
    """一台车的协同侧状态（和后端的车辆记录分开，避免耦合）。"""

    def __init__(self, name: str):
        self.name = name
        self.payload = None          # 最近一次遥测原文
        self.received = 0.0          # 本地收到时刻
        self.seq = 0                 # observer 的序号（可用于丢包检测）
        self.s = None                # 投影出的弧长
        self.error = None            # 投影误差（离路径的横向距离）
        self.problem = ''            # 不能用的原因（'' = 正常）
        self.last_s = None           # 上一次的 s，用于防跳变
        self.last_time = None        # 上一次的接收时刻

    def age(self, now: float) -> float:
        return now - self.received if self.received else float('inf')


class Convoy:
    """两台（及以上）车的跟驰协调。线程安全。

    调用方式：
        convoy.ingest(payload)     # 车端 observer 推进来
        convoy.status()            # 任何时候查（前端 / 调试 / 执行层）
    """

    def __init__(self, config=None):
        cfg = dict(config or {})
        self.observer_token = str(cfg.get('observer_token') or '')
        self.fresh_seconds = float(cfg.get('fresh_seconds', DEFAULT_FRESH_SECONDS))
        self.hold_seconds = float(cfg.get('hold_seconds', 0))
        self.lock = threading.RLock()
        self.tracks: dict[str, VehicleTrack] = {}
        self.route: LoopRoute | None = None
        self.route_name = ''
        self.route_error = ''
        # ⚠️ 每【一对】车必须有独立的 FollowingAdvice —— 它内部带状态
        #    （stopped_at / clear_since），多对共用会互相干扰、建议乱跳。
        self._advices: dict[str, FollowingAdvice] = {}
        self._history: list[dict] = []      # 最近若干次建议，便于排查

    # ────────────────────────── 路径 ──────────────────────────

    def load_route(self, points, name=''):
        """把车端拉回来的路径点交给 convoy_policy 做投影准备。

        points: [(x, y), ...]。必须是闭环，否则 LoopRoute 会拒绝 —— 这是好事，
        因为非闭环路径跑"跟驰"没有意义（车会开出去）。
        """
        try:
            route = LoopRoute([(float(x), float(y)) for x, y in points])
        except Exception as exc:
            with self.lock:
                self.route = None
                self.route_error = str(exc)
            raise ConvoyError(f'路径不可用：{exc}') from exc
        with self.lock:
            self.route = route
            self.route_name = name
            self.route_error = ''
            # 换路径了，之前的 s 不能再用
            for track in self.tracks.values():
                track.last_s = None
        return route.length

    # ────────────────────────── 遥测 ──────────────────────────

    def check_token(self, token) -> bool:
        """车端 observer 的令牌校验。

        ⚠️ 这里【不能】用 fleet.valid_origin()：那条规则只允许 127.0.0.1 同源，
        而 observer 是从车上（192.168.31.x）推过来的。所以单独用令牌把关。
        """
        if not self.observer_token:
            return False
        return str(token or '') == self.observer_token

    def ingest(self, payload) -> dict:
        """接收一条车端遥测。返回一个简短回执（不发任何车端动作）。"""
        if not isinstance(payload, dict):
            raise ConvoyError('遥测必须是 JSON 对象')
        name = str(payload.get('vehicle') or '').strip()
        if not name:
            raise ConvoyError('遥测缺少 vehicle 字段')
        if len(name) > 60:
            raise ConvoyError('vehicle 名称过长')
        now = time.time()
        with self.lock:
            track = self.tracks.get(name) or VehicleTrack(name)
            self.tracks[name] = track
            track.payload = dict(payload)
            track.received = now
            seq = payload.get('seq')
            if isinstance(seq, int):
                track.seq = seq
            problem = ''
            track.s = None
            track.error = None
            if self.route is None:
                problem = f'后端还没有可用路径（{self.route_error or "尚未加载"}）'
            else:
                try:
                    s, error = self._project(track, now)
                    track.s, track.error = s, error
                    track.last_s, track.last_time = s, now
                except ValueError as exc:
                    problem = str(exc)
                except Exception as exc:                      # 投影不该抛别的，兜住
                    problem = f'project_failed: {exc}'
            track.problem = problem
        return {'ok': True, 'vehicle': name, 'seq': track.seq,
                'accepted': not problem, 'problem': problem}

    def _project(self, track: VehicleTrack, now: float):
        """投影，并把 convoy_policy 需要的 previous/elapsed 传进去。"""
        pose = track.payload or {}
        try:
            x, y, yaw = float(pose['x']), float(pose['y']), float(pose['yaw'])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError('pose_missing') from exc
        elapsed = (now - track.last_time) if track.last_time else 0
        return self.route.project(x, y, yaw, previous=track.last_s, elapsed=elapsed)

    # ────────────────────────── 状态 ──────────────────────────

    def status(self) -> dict:
        """产出完整协同状态：每台车的位置/健康度 + 两两跟驰建议。

        纯计算，无副作用。
        """
        now = time.time()
        with self.lock:
            tracks = [self._track_view(t, now) for t in self.tracks.values()]
            route = self.route
            route_name, route_error = self.route_name, self.route_error
            usage = self._pair_advice(tracks, now)
        return {
            'timestamp': now,
            'route': {'name': route_name, 'length': route.length if route else None,
                      'error': route_error},
            'vehicles': tracks,
            'advice': usage,
            'armed': False,          # 本模块永远不下发；这一位由执行层覆盖
        }

    def _track_view(self, track: VehicleTrack, now: float) -> dict:
        payload = track.payload or {}
        age = track.age(now)
        pose_age = payload.get('pose_age')
        healthy = (not track.problem) and age <= self.fresh_seconds
        if age > self.fresh_seconds:
            reason = f'遥测过期 {age:.1f}s'
        else:
            reason = track.problem
        return {
            'vehicle': track.name,
            'seq': track.seq,
            'age': round(age, 2),
            'pose_age': pose_age,
            'source_age': payload.get('source_age'),
            'speed': payload.get('speed'),
            's': None if track.s is None else round(track.s, 3),
            'lateral': None if track.error is None else round(track.error, 3),
            'map': payload.get('map'),
            'route': payload.get('route'),
            'map_hash': payload.get('map_hash'),
            'route_hash': payload.get('route_hash'),
            'metadata_error': payload.get('metadata_error'),
            'healthy': healthy,
            'reason': reason or ('正常' if healthy else '未知'),
        }

    def _pair_advice(self, tracks, now) -> list[dict]:
        """对每一对可用的车给出跟驰建议。

        判据完全复用 convoy_policy.FollowingAdvice —— 我们不去重新发明阈值。
        注意：只有【两台都健康、且在同一张地图/同一条路径】时才计算，
        否则给 suspended（挂起），绝不基于可疑数据做判断。
        """
        usable = [t for t in tracks if t['healthy'] and t['s'] is not None]
        usable_names = {t['vehicle'] for t in usable}
        out = []
        # ⚠️ 数据不可信时【绝不能】安静地"没有建议" —— 调用方会把"看不清"
        #    误读成"没危险"，在安全逻辑里这是致命的。
        #    所以每台不可用的车都要显式说明原因（与 main_sync_loop 的位姿闸门同一原则）。
        if len(tracks) >= 2:
            for t in tracks:
                if t['vehicle'] not in usable_names:
                    out.append({'vehicle': t['vehicle'], 'state': 'suspended',
                                'reason': f"位姿不可用：{t['reason']}"})
        for i in range(len(usable)):
            for j in range(i + 1, len(usable)):
                a, b = usable[i], usable[j]
                pair = {'a': a['vehicle'], 'b': b['vehicle']}
                # 同路校验：哈希不一致说明两台车跑的不是同一条路径，跟驰无从谈起
                if a['route_hash'] and b['route_hash'] and a['route_hash'] != b['route_hash']:
                    out.append({**pair, 'state': 'suspended',
                                'reason': '两车 route_hash 不一致（不是同一条路径）'})
                    continue
                if a['map_hash'] and b['map_hash'] and a['map_hash'] != b['map_hash']:
                    out.append({**pair, 'state': 'suspended',
                                'reason': '两车 map_hash 不一致（不是同一张地图）'})
                    continue
                length = self.route.length
                signed = (a['s'] - b['s'] + length / 2) % length - length / 2
                ahead, behind = (a, b) if signed > 0 else (b, a)
                gap = abs(signed)
                velocity = float(behind.get('speed') or 0.0)
                # ⭐ 这一对车用自己独立的建议器（键与前后顺序无关，保证稳定）
                key = '|'.join(sorted((a['vehicle'], b['vehicle'])))
                advice = self._advices.get(key)
                if advice is None:
                    advice = self._advices[key] = FollowingAdvice(hold_seconds=self.hold_seconds)
                action, factor, stop_gap = advice.update(gap, velocity, now)
                out.append({**pair, 'state': action,
                            'ahead': ahead['vehicle'], 'behind': behind['vehicle'],
                            'gap': round(gap, 3), 'closing_speed': round(velocity, 3),
                            'stop_gap': round(stop_gap, 3), 'factor': round(factor, 3),
                            'reason': f'{behind["vehicle"]} 距 {ahead["vehicle"]} {gap:.2f} m '
                                      f'(停车阈值 {stop_gap:.2f} m)'})
        return out

    def reset(self):
        """清空（车端换了路径/地图，或者想重新开始统计时用）。"""
        with self.lock:
            self.tracks.clear()
            self.route = None
            self.route_name = ''
            self.route_error = ''
            self._advices = {}
